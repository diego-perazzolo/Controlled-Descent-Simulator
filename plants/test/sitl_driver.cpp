// =============================================================================
// Controlled Descent Simulator
// =============================================================================
//
// Copyright (c) 2026 Diego Perazzolo
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// =============================================================================
// File        : sitl_driver.cpp
// Description : Native integration test of the SITL plant against an in-process
//               fake ArduCopter speaking real MAVLink 2 over UDP loopback.
//               Exercises the whole link: session bring-up to READY, telemetry
//               decode and NED<->ENU conversion (both directions), the
//               two-regime frame offset (staging zeroed at origin, mission
//               aligned to the trajectory start), the readiness gate, the
//               setpoint stream and the silence watchdog. No Docker, no python.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <arpa/inet.h>

/* the fake speaks MAVLink through the same pinned headers as the plant */
#include "mavlink_pin.hpp"

#include "SitlPlant.hpp"
#include "UdpTransport.hpp"

using CDS::BasePlant;
using plants::SitlPlant;
using plants::UdpTransport;

static int _failures = 0;

#define CHECK(cond)                                                     \
    do                                                                  \
    {                                                                   \
        if (!(cond))                                                    \
        {                                                               \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            _failures++;                                                \
        }                                                               \
    } while (0)

static constexpr double PI = 3.14159265358979323846;

/* poll cond every 5 ms until true or timeout; returns true when satisfied */
static bool waitFor(const std::function<bool(void)>& cond, double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cond())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return cond();
}

/* In-process fake ArduCopter: streams HEARTBEAT + LOCAL_POSITION_NED +
   ATTITUDE to the plant and records what the plant sends back (telemetry
   stream requests and position setpoints). Its own MAVLink parsing uses
   channel 1 so it never clashes with the plant's channel 0. */
class FakeSitl
{
    public:

    FakeSitl(const std::string& plantHost, uint16_t plantPort,
             uint16_t ownPort)
        : m_plantHost(plantHost), m_plantPort(plantPort), m_ownPort(ownPort),
          m_run(false), m_nedX(0), m_nedY(0), m_nedZ(0),
          m_nedVx(0), m_nedVy(0), m_nedVz(0), m_yawNed(0),
          m_reqLpos(false), m_reqAttitude(false), m_haveSetpoint(false),
          m_spX(0), m_spY(0), m_spZ(0), m_spVx(0), m_spVy(0), m_spVz(0),
          m_spYaw(0), m_spMask(0), m_spFrame(0)
    {
    }

    ~FakeSitl() { Stop(); }

    bool Start(void)
    {
        if (m_socket.Open(m_plantHost, m_ownPort, 0.02))
        {
            return true;
        }

        std::memset(&m_target, 0, sizeof(m_target));
        m_target.sin_family = AF_INET;
        m_target.sin_port = htons(m_plantPort);
        if (::inet_pton(AF_INET, m_plantHost.c_str(), &m_target.sin_addr) != 1)
        {
            return true;
        }

        m_epoch = std::chrono::steady_clock::now();
        m_run = true;
        m_thread = std::thread(&FakeSitl::_loop, this);
        return false;
    }

    void Stop(void)
    {
        if (m_thread.joinable())
        {
            m_run = false;
            m_thread.join();
        }
        m_socket.Close();
    }

    /* teleport the reported vehicle pose (NED position + heading) */
    void SetPose(double nedX, double nedY, double nedZ, double yawNed)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_nedX = nedX; m_nedY = nedY; m_nedZ = nedZ; m_yawNed = yawNed;
    }

    /* when set, the fake follows commanded position setpoints (teleport) */
    void SetTrackSetpoints(bool on) { m_trackSetpoints = on; }

    bool StreamsRequested(void)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_reqLpos && m_reqAttitude;
    }

    bool LastSetpoint(double& x, double& y, double& z,
                      double& vx, double& vy, double& vz, double& yaw,
                      uint16_t& mask, uint8_t& frame)
    {
        std::lock_guard<std::mutex> lock(m_mutex);
        if (!m_haveSetpoint)
        {
            return false;
        }
        x = m_spX; y = m_spY; z = m_spZ;
        vx = m_spVx; vy = m_spVy; vz = m_spVz; yaw = m_spYaw;
        mask = m_spMask; frame = m_spFrame;
        return true;
    }

    private:

    void _loop(void)
    {
        auto lastTx = std::chrono::steady_clock::now();
        uint8_t buffer[2048];
        sockaddr_in from = {};

        while (m_run)
        {
            const auto now = std::chrono::steady_clock::now();
            if (std::chrono::duration<double>(now - lastTx).count() >= 0.02)
            {
                _sendTelemetry();
                lastTx = now;
            }

            /* short blocking receive paces the loop and catches replies */
            int received = m_socket.Recv(buffer, sizeof(buffer), from);
            while (received > 0)
            {
                for (int i = 0; i < received; i++)
                {
                    _parseByte(buffer[i]);
                }
                received = m_socket.TryRecv(buffer, sizeof(buffer), from);
            }
        }
    }

    void _sendTelemetry(void)
    {
        double nedX, nedY, nedZ, yawNed;
        bool sendLpos, sendAttitude;
        {
            std::lock_guard<std::mutex> lock(m_mutex);
            nedX = m_nedX; nedY = m_nedY; nedZ = m_nedZ; yawNed = m_yawNed;
            /* like a real ArduPilot: stream a message only once it has been
               requested via SET_MESSAGE_INTERVAL */
            sendLpos = m_reqLpos;
            sendAttitude = m_reqAttitude;
        }

        /* realistic monotonic on-board time, milliseconds since boot */
        const uint32_t bootMs = static_cast<uint32_t>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - m_epoch).count());

        mavlink_message_t message;
        uint8_t out[MAVLINK_MAX_PACKET_LEN];

        mavlink_msg_heartbeat_pack(1, MAV_COMP_ID_AUTOPILOT1, &message,
                                   MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_ARDUPILOTMEGA,
                                   0, 0, MAV_STATE_ACTIVE);
        m_socket.Send(out, mavlink_msg_to_send_buffer(out, &message), m_target);

        if (sendLpos)
        {
            mavlink_msg_local_position_ned_pack(1, MAV_COMP_ID_AUTOPILOT1,
                                                &message, bootMs, nedX, nedY,
                                                nedZ, 0, 0, 0);
            m_socket.Send(out, mavlink_msg_to_send_buffer(out, &message),
                          m_target);
        }

        if (sendAttitude)
        {
            mavlink_msg_attitude_pack(1, MAV_COMP_ID_AUTOPILOT1, &message,
                                      bootMs, 0, 0, yawNed, 0, 0, 0);
            m_socket.Send(out, mavlink_msg_to_send_buffer(out, &message),
                          m_target);
        }
    }

    void _parseByte(uint8_t byte)
    {
        mavlink_message_t message;
        mavlink_status_t status;
        if (mavlink_parse_char(MAVLINK_COMM_1, byte, &message, &status) !=
            MAVLINK_FRAMING_OK)
        {
            return;
        }

        if (message.msgid == MAVLINK_MSG_ID_COMMAND_LONG)
        {
            mavlink_command_long_t command;
            mavlink_msg_command_long_decode(&message, &command);
            if (command.command == MAV_CMD_SET_MESSAGE_INTERVAL)
            {
                std::lock_guard<std::mutex> lock(m_mutex);
                const int id = static_cast<int>(command.param1);
                if (id == MAVLINK_MSG_ID_LOCAL_POSITION_NED) m_reqLpos = true;
                if (id == MAVLINK_MSG_ID_ATTITUDE) m_reqAttitude = true;
            }
            else if (command.command == MAV_CMD_DO_SET_MODE ||
                     command.command == MAV_CMD_COMPONENT_ARM_DISARM ||
                     command.command == MAV_CMD_NAV_TAKEOFF)
            {
                /* accept every staging command; on takeoff, climb to the
                   requested altitude (teleport up, still) */
                if (command.command == MAV_CMD_NAV_TAKEOFF)
                {
                    std::lock_guard<std::mutex> lock(m_mutex);
                    m_nedZ = -static_cast<double>(command.param7);
                }
                mavlink_message_t ack;
                /* ack is addressed back to the plant (sysid 254); the plant
                   does not check the target, so the exact value is not load-
                   bearing */
                mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, &ack,
                                             command.command, MAV_RESULT_ACCEPTED,
                                             0, 0, 254, 0);
                uint8_t out[MAVLINK_MAX_PACKET_LEN];
                m_socket.Send(out, mavlink_msg_to_send_buffer(out, &ack),
                              m_target);
            }
        }
        else if (message.msgid == MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED)
        {
            mavlink_set_position_target_local_ned_t sp;
            mavlink_msg_set_position_target_local_ned_decode(&message, &sp);
            std::lock_guard<std::mutex> lock(m_mutex);
            m_spX = sp.x; m_spY = sp.y; m_spZ = sp.z;
            m_spVx = sp.vx; m_spVy = sp.vy; m_spVz = sp.vz;
            m_spYaw = sp.yaw; m_spMask = sp.type_mask;
            m_spFrame = sp.coordinate_frame;
            m_haveSetpoint = true;
            /* a GUIDED vehicle always yaws to the commanded heading: this lets
               the staging yaw maneuver reach the target heading (STAGED gate) */
            m_yawNed = sp.yaw;
            /* optionally follow the commanded position (used to exercise the
               airborne re-staging climb) */
            if (m_trackSetpoints) { m_nedX = sp.x; m_nedY = sp.y; m_nedZ = sp.z; }
        }
    }

    std::string m_plantHost;
    uint16_t m_plantPort;
    uint16_t m_ownPort;
    UdpTransport m_socket;
    sockaddr_in m_target;
    std::thread m_thread;
    std::atomic<bool> m_run;
    std::atomic<bool> m_trackSetpoints{false};
    std::mutex m_mutex;
    std::chrono::steady_clock::time_point m_epoch;

    /* reported pose (NED) */
    double m_nedX, m_nedY, m_nedZ, m_nedVx, m_nedVy, m_nedVz, m_yawNed;

    /* observations of what the plant sent */
    bool m_reqLpos, m_reqAttitude, m_haveSetpoint;
    double m_spX, m_spY, m_spZ, m_spVx, m_spVy, m_spVz, m_spYaw;
    uint16_t m_spMask;
    uint8_t m_spFrame;
};

static SitlPlant::sitlParams_t params(uint16_t port)
{
    return {.host = "127.0.0.1",
            .port = port,
            .setpointPeriod_seconds = 0.05,
            .telemetryPeriod_seconds = 0.02,
            .linkTimeout_seconds = 1.0,
            .stabilityVelThreshold_ms = 0.3,
            .stabilityHoldTime_seconds = 0.5};
}

static BasePlant::plantCommands_t command(double px, double py, double pz,
                                          double vx, double vy, double vz,
                                          double yaw)
{
    BasePlant::plantCommands_t c = {};
    c.reference.pos = {px, py, pz};
    c.reference.vel = {vx, vy, vz};
    c.reference.yaw = yaw;
    return c;
}

int main(void)
{
    const uint16_t plantPort = 14570;
    const uint16_t fakePort = 14571;

    SitlPlant plant;
    CHECK(plant.SetPlantParams(params(plantPort)) == false);

    /* mission on a disconnected link is refused */
    CHECK(plant.Start() == true);
    CHECK(plant.Connect() == false);

    /* immediately after connect the link is not identified yet: Start refused */
    CHECK(plant.IsReadyToStart() == false);
    CHECK(plant.Start() == true);

    FakeSitl fake("127.0.0.1", plantPort, fakePort);
    /* first pose: on the ground at the origin, facing North (yaw_ned 0) */
    fake.SetPose(0, 0, 0, 0);
    CHECK(fake.Start() == false);

    /* session reaches READY on the flight controller heartbeat */
    CHECK(waitFor([&] {
        return plant.GetLinkState() == SitlPlant::linkState_t::READY;
    }, 3.0));

    /* the plant asks for the telemetry streams it consumes */
    CHECK(waitFor([&] { return fake.StreamsRequested(); }, 3.0));

    /* staging regime: the ground pose is zeroed at the CDS origin */
    BasePlant::plantMeasurements_t meas = {};
    CHECK(waitFor([&] {
        return plant.PullMeasurements(meas) == false && meas.sequence > 0;
    }, 3.0));
    CHECK(std::abs(meas.state.x) < 1e-3);
    CHECK(std::abs(meas.state.y) < 1e-3);
    CHECK(std::abs(meas.state.z) < 1e-3);

    /* inbound NED→ENU: moving to (N 3, E 5, up 30) must move the ghost by the
       converted delta (E 5 → x, N 3 → y, up 30 → z) */
    fake.SetPose(3, 5, -30, 0);
    CHECK(waitFor([&] {
        return plant.PullMeasurements(meas) == false &&
               std::abs(meas.state.x - 5) < 1e-3 &&
               std::abs(meas.state.y - 3) < 1e-3 &&
               std::abs(meas.state.z - 30) < 1e-3;
    }, 2.0));

    /* back on the ground for the staging phase */
    fake.SetPose(0, 0, 0, 0);
    CHECK(waitFor([&] {
        return plant.PullMeasurements(meas) == false && std::abs(meas.state.z) < 1e-3;
    }, 2.0));

    /* auto-staging from the ground: BeginStaging drives GUIDED → arm →
       takeoff → climb → yaw to the mission heading (0.1 rad ENU); the fake
       acks each command, climbs to the requested altitude on takeoff and yaws
       to the commanded heading. Start is refused until STAGED, allowed after */
    CHECK(plant.IsReadyToStart() == false);
    CHECK(plant.Start() == true);
    CHECK(plant.BeginStaging(40.0, 0.1) == false);
    CHECK(waitFor([&] {
        return plant.GetStagingState() == SitlPlant::stagingState_t::STAGED;
    }, 5.0));
    CHECK(plant.IsReadyToStart());

    /* the vehicle now hovers at ENU (0,0,40) — NED (0,0,-40) — facing 0.1 */

    /* ref0 present before Start: first fetch captures the mission offset */
    plant.PushCommands(command(1, 2, 3, 0, 0, 0, 0.1));
    CHECK(plant.Start() == false);

    /* mission regime: the ghost coincides with the trajectory's first point */
    CHECK(waitFor([&] {
        return plant.PullMeasurements(meas) == false &&
               std::abs(meas.state.x - 1) < 1e-3 &&
               std::abs(meas.state.y - 2) < 1e-3 &&
               std::abs(meas.state.z - 3) < 1e-3 &&
               std::abs(meas.state.yaw - 0.1) < 1e-3;
    }, 2.0));

    /* stream refB: the setpoint on the wire is the NED conversion of
       (refB position + missionOffset), with the reference heading commanded
       absolutely (no yaw offset — the vehicle was staged to it). raw ENU pose
       = (0,0,40) facing 0.1; ref0 = (1,2,3) → position offset = (-1,-2,37).
       refB = (11,12,13) vel (1,2,3) yaw 0.1 → target ENU (10,10,50) yaw 0.1
       → NED North 10, East 10, Down -50, vN 2 vE 1 vD -3, yaw_ned pi/2 - 0.1 */
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(1);
    while (std::chrono::steady_clock::now() < deadline)
    {
        plant.PushCommands(command(11, 12, 13, 1, 2, 3, 0.1));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    double x, y, z, vx, vy, vz, yaw;
    uint16_t mask;
    uint8_t frame;
    CHECK(fake.LastSetpoint(x, y, z, vx, vy, vz, yaw, mask, frame));
    CHECK(std::abs(x - 10) < 1e-3);
    CHECK(std::abs(y - 10) < 1e-3);
    CHECK(std::abs(z + 50) < 1e-3);
    CHECK(std::abs(vx - 2) < 1e-3);
    CHECK(std::abs(vy - 1) < 1e-3);
    CHECK(std::abs(vz + 3) < 1e-3);
    CHECK(std::abs(yaw - (PI / 2.0 - 0.1)) < 1e-3);
    CHECK(mask == (POSITION_TARGET_TYPEMASK_AX_IGNORE |
                   POSITION_TARGET_TYPEMASK_AY_IGNORE |
                   POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE));
    CHECK(frame == MAV_FRAME_LOCAL_NED);

    /* mission stop must brake the vehicle: a zero-velocity hold at its
       current NED pose (0,0,-40), not leave it coasting on the last
       commanded velocity */
    plant.Stop();
    CHECK(waitFor([&] {
        double hx, hy, hz, hvx, hvy, hvz, hyaw;
        uint16_t hm; uint8_t hf;
        return fake.LastSetpoint(hx, hy, hz, hvx, hvy, hvz, hyaw, hm, hf) &&
               std::abs(hvx) < 1e-3 && std::abs(hvy) < 1e-3 &&
               std::abs(hvz) < 1e-3 && std::abs(hz + 40) < 1e-3;
    }, 2.0));

    /* mission stop clears staging: the vehicle descended during the mission
       so it is no longer staged and must climb back up before another mission
       (guards against a descent commanded below ground) */
    CHECK(plant.GetStagingState() == SitlPlant::stagingState_t::IDLE);
    CHECK(plant.IsReadyToStart() == false);

    /* re-staging the already-airborne vehicle (still at NED -40): skip arm +
       takeoff (invalid in flight) and CLIMB to the new altitude via a GUIDED
       position setpoint. The fake now follows the setpoints, so it climbs to
       50 m and reaches STAGED — the recovery/repeat path after a maneuver */
    fake.SetTrackSetpoints(true);
    CHECK(plant.BeginStaging(50.0, 0.1) == false);
    /* STAGED requires |assembled.z - 50| < tol, so reaching it proves the
       vehicle actually climbed to the new altitude */
    CHECK(waitFor([&] {
        return plant.GetStagingState() == SitlPlant::stagingState_t::STAGED;
    }, 5.0));
    CHECK(plant.IsReadyToStart());

    /* StopStaging aborts a completed staging back to IDLE (armed hold) */
    CHECK(plant.StopStaging() == false);
    CHECK(waitFor([&] {
        return plant.GetStagingState() == SitlPlant::stagingState_t::IDLE;
    }, 2.0));
    CHECK(plant.IsReadyToStart() == false);
    fake.SetTrackSetpoints(false);

    /* silence watchdog: once the fake goes quiet the session must fall back
       to DISCONNECTED within the link timeout */
    fake.Stop();
    CHECK(waitFor([&] {
        return plant.GetLinkState() == SitlPlant::linkState_t::DISCONNECTED;
    }, 3.0));

    plant.Disconnect();

    if (_failures)
    {
        std::printf("SITL DRIVER FAILED (%d checks)\n", _failures);
        return 1;
    }

    std::printf("SITL DRIVER ALL OK\n");
    return 0;
}

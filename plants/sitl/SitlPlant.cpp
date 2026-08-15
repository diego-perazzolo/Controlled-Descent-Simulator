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
// File        : SitlPlant.cpp
// Description : ArduPilot SITL plant: MAVLink 2 over UDP, Guided-mode
//               setpoints out, local-frame telemetry back as measurements
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "SitlPlant.hpp"
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"

/* sole gateway to the vendored MAVLink headers: compiling it here keeps the
   protocol pins guarding every build of this library */
#include "mavlink_pin.hpp"

#include "UdpTransport.hpp"

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>

using namespace plants;

using Clock = std::chrono::steady_clock;
using FpSeconds = std::chrono::duration<double>;

namespace
{

    static const auto logger = cds_log::registry().module("SITL plant");
    static const auto profile = cds_profile::registry().module("SITL plant");

    // Plant data recorder (black-box wide CSV, server-side): every published
    // measurement — plant time, sample sequence, and the full 12-field core
    // state. One plant records at a time; it runs alongside the active model
    // recorder during a mission.
    static cds_record::Recorder<double, 14, 4096> recorder("SITL plant", {{
        "t_plant", "seq",
        "x", "y", "z", "x_dot", "y_dot", "z_dot",
        "roll", "pitch", "yaw", "roll_dot", "pitch_dot", "yaw_dot",
    }});

    // Record one published sample (communication thread, right after
    // PublishMeasurements). seq mirrors the publish count so gaps show.
    static void recordSample(const core_state_t& s, double t)
    {
#if CDS_RECORD_ENABLED
        static std::uint64_t seq = 0;
        const std::array<double, 14> row{{
            t, static_cast<double>(seq++),
            s.x, s.y, s.z, s.x_dot, s.y_dot, s.z_dot,
            s.roll, s.pitch, s.yaw, s.roll_dot, s.pitch_dot, s.yaw_dot,
        }};
        recorder.record(row);
#else
        (void)s; (void)t;
#endif
    }

    /* our identity on the wire: a ground-control-station peer. System id 254,
       not the 255 that ground stations (QGroundControl, MAVProxy) default to:
       sharing 255/190 with a co-connected GCS makes the flight controller
       conflate the two nodes and arbitrate commands erratically. */
    constexpr uint8_t OUR_SYSTEM_ID = 254;
    constexpr uint8_t OUR_COMPONENT_ID = MAV_COMP_ID_MISSIONPLANNER;

    constexpr double HEARTBEAT_PERIOD_S = 1.0;
    constexpr double BIND_RETRY_PERIOD_S = 1.0;

    /* UDP is lossy: keep asking for the telemetry streams until they flow */
    constexpr double TELEMETRY_REREQUEST_PERIOD_S = 2.0;

    /* ArduCopter flight mode number for GUIDED — a firmware constant, NOT a
       MAVLink enum, hence defined here rather than pinned in mavlink_pin.hpp */
    constexpr float ARDUCOPTER_GUIDED_MODE = 4.0f;

    /* auto-staging: resend a command with no ACK after this long, and the
       altitude window within which the climb counts as "at altitude" */
    constexpr double STAGE_CMD_RETRY_S = 1.0;
    constexpr double STAGE_ALT_TOL_M = 0.5;

    /* heading window (rad) within which the vehicle counts as "on the staging
       heading": STAGED is withheld until the vehicle has yawed to it */
    constexpr double STAGE_YAW_TOL_RAD = 0.05;

    /* above this height (m) the vehicle is already airborne: staging then
       skips arm + takeoff (which ArduCopter rejects in flight) and only
       settles to a stable hover */
    constexpr double STAGE_MIN_AIRBORNE_M = 0.3;

    /* setpoints command position + velocity + yaw; acceleration and yaw rate
       are ignored (yaw rate is left out for ArduCopter GUIDED compatibility,
       which does not accept a combined yaw + yaw-rate target) */
    constexpr uint16_t SETPOINT_TYPE_MASK =
        POSITION_TARGET_TYPEMASK_AX_IGNORE |
        POSITION_TARGET_TYPEMASK_AY_IGNORE |
        POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;

    constexpr double PI = 3.14159265358979323846;

    /* wrap an angle to (-pi, pi] */
    double wrapPi(double angle)
    {
        constexpr double TWO_PI = 2.0 * PI;
        angle = std::fmod(angle + PI, TWO_PI);
        if (angle <= 0.0)
        {
            angle += TWO_PI;
        }
        return angle - PI;
    }

}

/* communication-thread context: everything the loop needs, on its stack */
struct SitlPlant::Link
{
    UdpTransport transport;

    /* peer endpoint (latest valid MAVLink 2 speaker) */
    bool peerKnown;
    sockaddr_in peer;

    /* identity of the flight controller, latched from its heartbeat */
    uint8_t fcSystemId;
    uint8_t fcComponentId;

    /* loop-epoch timers, in seconds */
    Clock::time_point epoch;
    double lastValidRx_seconds;
    double lastHeartbeatTx_seconds;
    double lastTelemetryReq_seconds;
    double lastSetpointTx_seconds;
    double lastStageCmdTx_seconds;   /* auto-staging command retry timer */

    /* telemetry assembly: LOCAL_POSITION_NED (position + velocity) and
       ATTITUDE (angles + rates) arrive as separate messages; they are
       merged, already converted to CDS/ENU (but before any frame offset),
       into this running raw sample */
    core_state_t assembled;
    bool havePosition;
    double lastPositionRx_seconds;

    /* mission-readiness: instant the vehicle last became "still"; negative
       means it is currently moving */
    double stableSince_seconds;

    /* frame alignment, two regimes (see file header). Offsets are the raw
       CDS/ENU POSITION that maps to the CDS reference of the regime:
       subtracted from measurements, added to outbound setpoints. Heading is
       NOT offset: the vehicle is staged to the trajectory's initial heading
       and the mission commands the reference yaw absolutely, so the ghost's
       reported roll/pitch stay consistent with its rendered heading. */
    bool haveStagingRef;   /* staging: raw position that maps to the CDS origin */
    double stagingRefX, stagingRefY, stagingRefZ;
    bool haveMissionOffset;/* mission: raw position that maps to the trajectory t0 */
    double missionOffX, missionOffY, missionOffZ;

    /* edge detector on the mission toggle, to re-anchor on transitions */
    bool missionWasRunning;

    /* last raw pose the vehicle reported, in its own NED frame (straight
       from LOCAL_POSITION_NED / ATTITUDE, no conversion): the target of the
       safety hold "stay exactly here" on mission stop / disconnect */
    double lastNedX, lastNedY, lastNedZ, lastNedYaw;

    /* true while we are actively streaming setpoints: gates the one-shot
       safety hold so it is sent only when we were really driving */
    bool commanding;

    /* on-board timestamp of the last accepted sample of each stream: a
       message arriving older than this is a UDP reordering artefact and is
       dropped, so the ghost never jumps backwards */
    bool haveLposTime;
    uint32_t lastLposBootMs;
    bool haveAttTime;
    uint32_t lastAttBootMs;

    /* staging began with the vehicle already airborne: arm + takeoff were
       skipped, so STAGED needs only a stable hover (no altitude match) */
    bool stageSkipTakeoff;

    Link() : peerKnown(false),
             peer({}),
             fcSystemId(0),
             fcComponentId(0),
             epoch(Clock::now()),
             lastValidRx_seconds(0.0),
             lastHeartbeatTx_seconds(0.0),
             /* negative: the first stream request fires as soon as READY */
             lastTelemetryReq_seconds(-TELEMETRY_REREQUEST_PERIOD_S),
             lastSetpointTx_seconds(0.0),
             lastStageCmdTx_seconds(0.0),
             assembled({}),
             havePosition(false),
             lastPositionRx_seconds(0.0),
             stableSince_seconds(-1.0),
             haveStagingRef(false),
             stagingRefX(0.0), stagingRefY(0.0), stagingRefZ(0.0),
             haveMissionOffset(false),
             missionOffX(0.0), missionOffY(0.0), missionOffZ(0.0),
             missionWasRunning(false),
             lastNedX(0.0), lastNedY(0.0), lastNedZ(0.0), lastNedYaw(0.0),
             commanding(false),
             haveLposTime(false), lastLposBootMs(0),
             haveAttTime(false), lastAttBootMs(0),
             stageSkipTakeoff(false)
    {
        CDS_LOG_INFO(logger, "Link created");
    }

    /* seconds elapsed since the loop epoch */
    double Now(void) const
    {
        return FpSeconds(Clock::now() - epoch).count();
    }
};

SitlPlant::SitlPlant() : m_params({.host = "0.0.0.0",
                                   .port = 14550,
                                   .setpointPeriod_seconds = 0.05,
                                   .telemetryPeriod_seconds = 0.02,
                                   .linkTimeout_seconds = 2.0,
                                   .stabilityVelThreshold_ms = 0.3,
                                   .stabilityHoldTime_seconds = 3.0}),
                         m_threadRun(false),
                         m_missionRun(false),
                         m_linkState(linkState_t::DISCONNECTED),
                         m_stagingState(stagingState_t::IDLE),
                         m_stageRequested(false),
                         m_stageAltitude(0.0),
                         m_stageYaw(0.0)
{
    recorder.activateAsPlant(); // this plant owns the plant data recorder
    CDS_LOG_INFO(logger, "Plant created");
}

SitlPlant::~SitlPlant()
{
    Stop();
    Disconnect();
    CDS_LOG_INFO(logger, "Plant released");
}

bool SitlPlant::SetPlantParams(const std::any& params)
{
    if (m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Cannot reconfigure plant while it is running");
        return true;
    }

    if (params.type() != typeid(sitlParams_t&))
    {
        CDS_LOG_ERROR(logger, "Wrong params type");
        return true;
    }

    const auto& p = std::any_cast<const sitlParams_t&>(params);

    if (p.host.empty() || p.port == 0 ||
        p.setpointPeriod_seconds <= 0 || p.telemetryPeriod_seconds <= 0 ||
        p.linkTimeout_seconds <= 0 ||
        p.stabilityVelThreshold_ms <= 0 || p.stabilityHoldTime_seconds <= 0)
    {
        CDS_LOG_ERROR(logger, "Invalid parameter value");
        return true;
    }

    CDS_LOG_INFO(logger, "Params succesfully set");
    m_params = p;

    // Recorder run metadata for this plant.
    recorder.clearMeta();
    recorder.addMeta("plant", "SITL (ArduCopter)");
    recorder.addMeta("host", p.host.c_str());
    recorder.addMeta("port", static_cast<long long>(p.port));
    recorder.addMeta("setpoint_period_s", p.setpointPeriod_seconds);
    recorder.addMeta("telemetry_period_s", p.telemetryPeriod_seconds);
    recorder.addMeta("link_timeout_s", p.linkTimeout_seconds);

    return false;
}

bool SitlPlant::Connect(void)
{
    if (m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Plant already connected");
        return true;
    }

    m_threadRun = true;
    m_thread = std::thread(&SitlPlant::_commLoop, this);
    CDS_LOG_INFO(logger, "Starting connection");
    return false;
}

bool SitlPlant::Disconnect(void)
{
    /* idempotent: disconnecting a disconnected plant is not an error */
    if (!m_thread.joinable())
    {
        CDS_LOG_WARN(logger, "Disconnect: no plant is currently connected");
        return false;
    }

    m_missionRun = false;
    m_stageRequested = false;
    m_threadRun = false;
    m_thread.join();
    m_linkState = linkState_t::DISCONNECTED;
    m_stagingState = stagingState_t::IDLE;

    CDS_LOG_INFO(logger, "Plant disconnected");

    return false;
}

bool SitlPlant::Start(void)
{
    if (!m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Cannot start mission on a disconnected plant");
        return true;
    }

    if (m_missionRun)
    {
        CDS_LOG_ERROR(logger, "Mission is already started");
        return true;
    }

    if (m_stagingState != stagingState_t::STAGED)
    {
        CDS_LOG_ERROR(logger, "Plant is not staged");
        return true;
    }

    CDS_LOG_INFO(logger, "Mission started");
    m_missionRun = true;
    return false;
}

bool SitlPlant::Stop(void)
{
    /* idempotent: stopping a stopped mission is not an error */
    m_missionRun = false;
    /* clear staging: after a mission the vehicle has descended and is no
       longer at the staging altitude, so it must be re-staged (climb back up)
       before another mission — otherwise a descent from a low altitude would
       command the vehicle below ground. The mission-stop edge holds it. */
    m_stageRequested = false;
    m_stagingState = stagingState_t::IDLE;
    CDS_LOG_INFO(logger, "Mission stopped");
    return false;
}

bool SitlPlant::BeginStaging(double altitude_m, double headingYaw)
{
    if (!m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Plant is disconnected, cannot stage");
        return true;
    }

    if (altitude_m <= 0.0)
    {
        CDS_LOG_ERROR(logger, "Invalid staging altitude: {}m", altitude_m);
        return true;
    }

    m_stageAltitude = altitude_m;
    m_stageYaw = headingYaw;
    /* force a fresh sequence even from STAGED, so staging can be repeated
       after a maneuver (the vehicle climbs back to the altitude) */
    m_stagingState = stagingState_t::IDLE;
    m_stageRequested = true;

    CDS_LOG_INFO(logger, "Begin staging");
    return false;
}

bool SitlPlant::StopStaging(void)
{
    /* clears the request; the communication loop aborts any in-progress (or
       completed) staging back to IDLE and holds the vehicle in place */
    m_stageRequested = false;
    CDS_LOG_INFO(logger, "Stop staging");
    return false;
}

SitlPlant::linkState_t SitlPlant::GetLinkState(void) const
{
    return m_linkState;
}

SitlPlant::stagingState_t SitlPlant::GetStagingState(void) const
{
    return m_stagingState;
}

bool SitlPlant::IsReadyToStart(void) const
{
    return m_stagingState == stagingState_t::STAGED;
}

void SitlPlant::_commLoop(void)
{
    Link link;

    while (m_threadRun)
    {
        CDS_PROFILE(profile, "Communication loop");

        if (!link.transport.IsOpen())
        {
            if (link.transport.Open(m_params.host, m_params.port,
                                    m_params.setpointPeriod_seconds))
            {
                /* cannot bind yet (e.g. port still busy): retry, no spin */
                std::this_thread::sleep_for(FpSeconds(BIND_RETRY_PERIOD_S));
                continue;
            }
        }

        /* the bounded blocking receive inside paces the whole loop */
        _processInbound(link);

        const double tNow = link.Now();

        /* silence watchdog: an incompatible peer (MAVLink 1, or diverged
           message definitions failing CRC) produces no valid traffic and
           ends up here — the link is torn down explicitly, never left
           half-working */
        if (m_linkState != linkState_t::DISCONNECTED &&
            tNow - link.lastValidRx_seconds > m_params.linkTimeout_seconds)
        {
            link.peerKnown = false;
            link.havePosition = false;
            link.stableSince_seconds = -1.0;
            link.haveStagingRef = false;
            link.haveMissionOffset = false;
            /* link lost: staging is void, require an explicit re-stage */
            m_stagingState = stagingState_t::IDLE;
            m_stageRequested = false;
            m_linkState = linkState_t::DISCONNECTED;
        }

        /* mission-toggle edges re-anchor the frame: on Start, drop the
           mission offset so the first command re-captures it; on Stop,
           re-zero to the CDS origin (staging regime) and, if we were driving,
           command the safety hold so the vehicle brakes instead of coasting
           to the last commanded target */
        const bool missionNow = m_missionRun;
        if (missionNow != link.missionWasRunning)
        {
            link.haveMissionOffset = false;
            if (!missionNow)
            {
                if (link.commanding && link.peerKnown)
                {
                    _sendHold(link);
                    link.commanding = false;
                }
                link.haveStagingRef = false;
            }
            link.missionWasRunning = missionNow;
        }

        /* drive the auto-staging sequence (GUIDED → arm → takeoff → climb →
           STAGED). Only meaningful on an identified link and before a mission */
        if (m_linkState == linkState_t::READY && !missionNow)
        {
            _runStaging(link);
        }

        if (link.peerKnown &&
            tNow - link.lastHeartbeatTx_seconds >= HEARTBEAT_PERIOD_S)
        {
            _sendHeartbeat(link);
            link.lastHeartbeatTx_seconds = tNow;
        }

        /* once the flight controller is identified, ask for the telemetry
           streams (ArduPilot does not stream LOCAL_POSITION_NED until asked)
           promptly, then keep re-asking until they actually flow (UDP loss) */
        const bool positionFlowing =
            link.havePosition &&
            (tNow - link.lastPositionRx_seconds <= TELEMETRY_REREQUEST_PERIOD_S);
        if (m_linkState == linkState_t::READY && !positionFlowing &&
            tNow - link.lastTelemetryReq_seconds >= TELEMETRY_REREQUEST_PERIOD_S)
        {
            _requestTelemetryStreams(link);
            link.lastTelemetryReq_seconds = tNow;
        }

        /* mission running: capture the mission offset from the first command
           (frame alignment), then stream setpoints at the configured rate */
        if (missionNow && link.havePosition)
        {
            plantCommands_t commands = {};
            if (!FetchCommands(commands))
            {
                if (!link.haveMissionOffset)
                {
                    /* offset = current raw position − trajectory t0 reference:
                       makes the ghost coincide with the trajectory start and
                       maps setpoints back into the vehicle's frame. Heading is
                       not offset (staging already aligned it) */
                    link.missionOffX = link.assembled.x - commands.reference.pos[0];
                    link.missionOffY = link.assembled.y - commands.reference.pos[1];
                    link.missionOffZ = link.assembled.z - commands.reference.pos[2];
                    link.haveMissionOffset = true;
                }

                if (tNow - link.lastSetpointTx_seconds >=
                    m_params.setpointPeriod_seconds)
                {
                    _sendSetpoint(link, commands);
                    link.lastSetpointTx_seconds = tNow;
                    link.commanding = true;
                }
            }
        }
    }

    /* dying act of the communication thread (covers Disconnect, where the
       mission-stop edge above is never processed): brake the vehicle while
       the socket is still open, so a detach never leaves it coasting */
    if (link.commanding && link.peerKnown)
    {
        _sendHold(link);
        link.commanding = false;
    }

    m_linkState = linkState_t::DISCONNECTED;
}

void SitlPlant::_processInbound(Link& link)
{
    CDS_PROFILE(profile, "Receiving messages");
    uint8_t buffer[2048];
    sockaddr_in from = {};

    /* first receive blocks up to the transport timeout (the loop pacing);
       the drain that follows never blocks */
    int received = link.transport.Recv(buffer, sizeof(buffer), from);

    while (received > 0)
    {
        for (int i = 0; i < received; i++)
        {
            mavlink_message_t message;
            mavlink_status_t status;

            if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i],
                                   &message, &status) != MAVLINK_FRAMING_OK)
            {
                /* incomplete, or a frame that failed CRC / signature: an
                   incompatible peer ends up here and is thus never counted
                   as valid traffic */
                continue;
            }

            if (status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)
            {
                /* MAVLink 1 peer: incompatible by policy — not counted as
                   valid traffic, the silence watchdog will surface it */
                continue;
            }

            /* valid MAVLink 2 message: freshest speaker owns the peer slot */
            link.lastValidRx_seconds = link.Now();
            link.peer = from;
            link.peerKnown = true;

            if (m_linkState == linkState_t::DISCONNECTED)
            {
                m_linkState = linkState_t::CONNECTED;
            }

            switch (message.msgid)
            {
                case MAVLINK_MSG_ID_HEARTBEAT:
                {
                    mavlink_heartbeat_t heartbeat;
                    mavlink_msg_heartbeat_decode(&message, &heartbeat);

                    /* the flight controller is identified by the heartbeat
                       of its autopilot component; a peer declaring a
                       different protocol generation is refused */
                    if (message.compid == MAV_COMP_ID_AUTOPILOT1 &&
                        heartbeat.autopilot != MAV_AUTOPILOT_INVALID &&
                        heartbeat.mavlink_version == MAVLINK_VERSION)
                    {
                        link.fcSystemId = message.sysid;
                        link.fcComponentId = message.compid;
                        m_linkState = linkState_t::READY;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_COMMAND_ACK:
                {
                    mavlink_command_ack_t ack;
                    mavlink_msg_command_ack_decode(&message, &ack);

                    const stagingState_t st = m_stagingState.load();

                    if (ack.result != MAV_RESULT_ACCEPTED)
                    {
                        /* A rejected NAV_TAKEOFF means the vehicle is already
                           flying (we only reach TAKEOFF once armed), so the
                           altitude-based airborne check was fooled — e.g. by a
                           low hover below STAGE_MIN_AIRBORNE_M. Retrying takeoff
                           is futile; fall back to the airborne path and drive
                           the climb with GUIDED setpoints. Any other rejection
                           is transient — _runStaging resends on its retry timer. */
                        if (st == stagingState_t::TAKEOFF &&
                            ack.command == MAV_CMD_NAV_TAKEOFF)
                        {
                            link.stageSkipTakeoff = true;
                            m_stagingState = stagingState_t::CLIMB;
                        }
                        break;
                    }

                    /* advance the staging sequence on the accepted command and
                       fire the next one immediately */
                    if (st == stagingState_t::SET_MODE &&
                        ack.command == MAV_CMD_DO_SET_MODE)
                    {
                        if (link.assembled.z > STAGE_MIN_AIRBORNE_M)
                        {
                            /* already flying: arm + takeoff would be rejected;
                               just settle to a stable hover */
                            link.stageSkipTakeoff = true;
                            m_stagingState = stagingState_t::CLIMB;
                        }
                        else
                        {
                            m_stagingState = stagingState_t::ARM;
                            _sendStageCommand(link, stagingState_t::ARM);
                        }
                    }
                    else if (st == stagingState_t::ARM &&
                             ack.command == MAV_CMD_COMPONENT_ARM_DISARM)
                    {
                        m_stagingState = stagingState_t::TAKEOFF;
                        _sendStageCommand(link, stagingState_t::TAKEOFF);
                    }
                    else if (st == stagingState_t::TAKEOFF &&
                             ack.command == MAV_CMD_NAV_TAKEOFF)
                    {
                        /* airborne: the climb is monitored by _runStaging */
                        m_stagingState = stagingState_t::CLIMB;
                    }
                    break;
                }

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&message, &attitude);

                    /* drop UDP-reordered stale samples (older on-board time) */
                    if (link.haveAttTime &&
                        attitude.time_boot_ms < link.lastAttBootMs)
                    {
                        break;
                    }
                    link.haveAttTime = true;
                    link.lastAttBootMs = attitude.time_boot_ms;

                    /* NED aircraft body → CDS/ENU baselink attitude. The
                       NED→ENU world frame (x↔y, z negated) and the FRD→FLU
                       body frame (y,z negated) are both proper rotations, and
                       their composition reduces the aerospace Z-Y-X Euler
                       angles EXACTLY to: roll unchanged, pitch negated, yaw =
                       π/2 − yaw_ned. This is not an approximation — it is
                       bit-identical to the full quaternion composition
                       (NED_ENU ⊗ q_ned ⊗ AIRCRAFT_BASELINK), and it matches
                       the convention the QuadRotor model itself reads back
                       from its body→ENU quaternion. */
                    link.assembled.roll = attitude.roll;
                    link.assembled.pitch = -attitude.pitch;
                    link.assembled.yaw = wrapPi(PI / 2.0 - attitude.yaw);

                    /* body angular rates: FRD → FLU flips the Y and Z axes,
                       an exact per-axis sign flip (no coupling) */
                    link.assembled.roll_dot = attitude.rollspeed;
                    link.assembled.pitch_dot = -attitude.pitchspeed;
                    link.assembled.yaw_dot = -attitude.yawspeed;

                    /* raw NED heading, kept for the safety hold */
                    link.lastNedYaw = attitude.yaw;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    mavlink_local_position_ned_t lpos;
                    mavlink_msg_local_position_ned_decode(&message, &lpos);

                    /* drop UDP-reordered stale samples (older on-board time),
                       so the published ghost never jumps backwards */
                    if (link.haveLposTime &&
                        lpos.time_boot_ms < link.lastLposBootMs)
                    {
                        break;
                    }
                    link.haveLposTime = true;
                    link.lastLposBootMs = lpos.time_boot_ms;

                    /* NED → CDS/ENU: East=N_y, North=N_x, Up=-N_z, same for
                       the velocity components */
                    link.assembled.x = lpos.y;
                    link.assembled.y = lpos.x;
                    link.assembled.z = -lpos.z;
                    link.assembled.x_dot = lpos.vy;
                    link.assembled.y_dot = lpos.vx;
                    link.assembled.z_dot = -lpos.vz;

                    /* raw NED position, kept for the safety hold */
                    link.lastNedX = lpos.x;
                    link.lastNedY = lpos.y;
                    link.lastNedZ = lpos.z;

                    link.havePosition = true;
                    link.lastPositionRx_seconds = link.Now();

                    /* stability tracking for the readiness gate: speed
                       magnitude is invariant under the axis permutation */
                    const double speed = std::sqrt(lpos.vx * lpos.vx +
                                                   lpos.vy * lpos.vy +
                                                   lpos.vz * lpos.vz);
                    if (speed <= m_params.stabilityVelThreshold_ms)
                    {
                        if (link.stableSince_seconds < 0.0)
                        {
                            link.stableSince_seconds = link.Now();
                        }
                    }
                    else
                    {
                        link.stableSince_seconds = -1.0;
                    }

                    /* staging regime: capture, once, the raw position whose
                       horizontal part maps to the CDS origin (the altitude is
                       shown as-is, see the publish below) */
                    if (!link.haveStagingRef && !m_missionRun)
                    {
                        link.stagingRefX = link.assembled.x;
                        link.stagingRefY = link.assembled.y;
                        link.stagingRefZ = link.assembled.z;
                        link.haveStagingRef = true;
                    }

                    /* publish the raw pose minus the active position offset
                       (mission offset once captured, else staging ref). Heading
                       is published as-is: the vehicle flies the trajectory's
                       actual heading, so its raw roll/pitch stay consistent
                       with the rendered yaw (no wrong-axis tilt). */
                    core_state_t out = link.assembled;
                    if (m_missionRun && link.haveMissionOffset)
                    {
                        out.x -= link.missionOffX;
                        out.y -= link.missionOffY;
                        out.z -= link.missionOffZ;
                    }
                    else if (link.haveStagingRef)
                    {
                        /* zero the horizontal position to the CDS origin, but
                           NOT the altitude: z is the physical height above the
                           takeoff point and maps directly to the CDS z.
                           Offsetting it by a captured reference would send the
                           ghost underground when the vehicle descends below
                           where the reference was taken. */
                        out.x -= link.stagingRefX;
                        out.y -= link.stagingRefY;
                    }

                    /* the vehicle's own boot time is the moment the sample
                       was TAKEN: publishing it makes the link latency
                       observable, exactly as the BasePlant contract wants */
                    PublishMeasurements(out, lpos.time_boot_ms / 1000.0);
                    recordSample(out, lpos.time_boot_ms / 1000.0);
                    break;
                }

                default:
                    break;
            }
        }

        received = link.transport.TryRecv(buffer, sizeof(buffer), from);
    }
}

void SitlPlant::_sendHeartbeat(Link& link)
{
    CDS_LOG_DEBUG(logger, "Send heartbeat");

    mavlink_message_t message;
    mavlink_msg_heartbeat_pack(OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message,
                               MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_ACTIVE);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);

    link.transport.Send(buffer, length, link.peer);
}

void SitlPlant::_requestTelemetryStreams(Link& link)
{
    CDS_LOG_INFO(logger, "Request telemetry streams");

    const float interval_us =
        static_cast<float>(m_params.telemetryPeriod_seconds * 1e6);

    const uint32_t messageIds[] = {MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                                   MAVLINK_MSG_ID_ATTITUDE};

    for (uint32_t messageId : messageIds)
    {
        mavlink_message_t message;
        mavlink_msg_command_long_pack(
            OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message,
            link.fcSystemId, link.fcComponentId,
            MAV_CMD_SET_MESSAGE_INTERVAL, 0 /* confirmation */,
            static_cast<float>(messageId), interval_us,
            0, 0, 0, 0, 0);

        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
        link.transport.Send(buffer, length, link.peer);
    }
}

void SitlPlant::_sendSetpoint(Link& link, const plantCommands_t& commands)
{
    CDS_LOG_DEBUG(logger, "Send setpoint");

    /* target pose in CDS/ENU: reference position plus the mission offset, so
       the trajectory frame maps onto the vehicle's frame. Heading is commanded
       absolutely (the reference yaw): the vehicle was staged to the
       trajectory's initial heading, so there is no yaw jump at mission start.
       Velocity is a translation, so it carries no offset */
    const double targetX = commands.reference.pos[0] + link.missionOffX;
    const double targetY = commands.reference.pos[1] + link.missionOffY;
    const double targetZ = commands.reference.pos[2] + link.missionOffZ;
    const double targetYaw = wrapPi(commands.reference.yaw);

    /* CDS/ENU → NED: North=E_y, East=E_x, Down=-E_z; heading measured from
       North instead of East (inverse of the inbound conversion) */
    const float nedX = static_cast<float>(targetY);
    const float nedY = static_cast<float>(targetX);
    const float nedZ = static_cast<float>(-targetZ);
    const float nedVx = static_cast<float>(commands.reference.vel[1]);
    const float nedVy = static_cast<float>(commands.reference.vel[0]);
    const float nedVz = static_cast<float>(-commands.reference.vel[2]);
    const float nedYaw = static_cast<float>(wrapPi(PI / 2.0 - targetYaw));

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_pack(
        OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message,
        0 /* time_boot_ms: unused by the setpoint consumer */,
        link.fcSystemId, link.fcComponentId, MAV_FRAME_LOCAL_NED,
        SETPOINT_TYPE_MASK,
        nedX, nedY, nedZ, nedVx, nedVy, nedVz,
        0, 0, 0 /* acceleration: ignored */,
        nedYaw, 0 /* yaw rate: ignored */);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    link.transport.Send(buffer, length, link.peer);
}

void SitlPlant::_sendHold(Link& link)
{
    CDS_LOG_INFO(logger, "Send hold");
    /* "stay exactly here": the vehicle's last reported NED pose with zero
       velocity. Offset-independent (a hold at the current position is the
       same in any frame origin), so it is safe even mid-mission */
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_pack(
        OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message,
        0 /* time_boot_ms: unused by the setpoint consumer */,
        link.fcSystemId, link.fcComponentId, MAV_FRAME_LOCAL_NED,
        SETPOINT_TYPE_MASK,
        static_cast<float>(link.lastNedX),
        static_cast<float>(link.lastNedY),
        static_cast<float>(link.lastNedZ),
        0, 0, 0 /* velocity: zero — brake and hold */,
        0, 0, 0 /* acceleration: ignored */,
        static_cast<float>(link.lastNedYaw), 0 /* yaw rate: ignored */);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    link.transport.Send(buffer, length, link.peer);
}

void SitlPlant::_sendClimbSetpoint(Link& link)
{
    CDS_LOG_DEBUG(logger, "Send climb setpoint");
    /* climb in place to the staging altitude and yaw to the staging heading:
       current NED horizontal position, target altitude, zero velocity, target
       heading — a GUIDED position target ArduCopter honours in flight (unlike
       NAV_TAKEOFF). CDS/ENU heading → NED heading (measured from North) */
    const float nedYaw =
        static_cast<float>(wrapPi(PI / 2.0 - m_stageYaw.load()));

    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_pack(
        OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message, 0,
        link.fcSystemId, link.fcComponentId, MAV_FRAME_LOCAL_NED,
        SETPOINT_TYPE_MASK,
        static_cast<float>(link.lastNedX),
        static_cast<float>(link.lastNedY),
        static_cast<float>(-m_stageAltitude.load()) /* down = -altitude */,
        0, 0, 0 /* velocity: zero */,
        0, 0, 0 /* acceleration: ignored */,
        nedYaw, 0 /* yaw rate: ignored */);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    link.transport.Send(buffer, length, link.peer);
}

void SitlPlant::_sendCommandLong(Link& link, uint16_t command,
                                 float p1, float p2, float p3, float p4,
                                 float p5, float p6, float p7)
{
    mavlink_message_t message;
    mavlink_msg_command_long_pack(OUR_SYSTEM_ID, OUR_COMPONENT_ID, &message,
                                  link.fcSystemId, link.fcComponentId,
                                  command, 0 /* confirmation */,
                                  p1, p2, p3, p4, p5, p6, p7);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(buffer, &message);
    link.transport.Send(buffer, length, link.peer);
}

void SitlPlant::_sendStageCommand(Link& link, stagingState_t state)
{
    switch (state)
    {
        case stagingState_t::SET_MODE:
            /* DO_SET_MODE: custom mode enabled, ArduCopter GUIDED */
            CDS_LOG_INFO(logger, "Guided mode");
            _sendCommandLong(link, MAV_CMD_DO_SET_MODE,
                             MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                             ARDUCOPTER_GUIDED_MODE);
            break;
        case stagingState_t::ARM:
            /* COMPONENT_ARM_DISARM: param1 = 1 (arm) */
            CDS_LOG_INFO(logger, "Armed mode");
            _sendCommandLong(link, MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
            break;
        case stagingState_t::TAKEOFF:
            /* NAV_TAKEOFF: param7 = altitude above the takeoff point */
            CDS_LOG_INFO(logger, "Take off");
            _sendCommandLong(link, MAV_CMD_NAV_TAKEOFF,
                             0, 0, 0, 0, 0, 0,
                             static_cast<float>(m_stageAltitude.load()));
            break;
        default:
            break;
    }
    link.lastStageCmdTx_seconds = link.Now();
}

void SitlPlant::_runStaging(Link& link)
{
    const double tNow = link.Now();
    const stagingState_t st = m_stagingState.load();

    /* abort: the request was cleared (StopStaging) — brake and reset. Also
       covers leaving the STAGED state back to a plain armed hold */
    if (!m_stageRequested)
    {
        if (st != stagingState_t::IDLE)
        {
            if (link.peerKnown)
            {
                _sendHold(link);
            }
            m_stagingState = stagingState_t::IDLE;
            CDS_LOG_INFO(logger, "Staging going idle");
        }
        return;
    }

    switch (st)
    {
        case stagingState_t::IDLE:
            /* kick off the sequence */
            link.stageSkipTakeoff = false;
            m_stagingState = stagingState_t::SET_MODE;
            CDS_LOG_INFO(logger, "Staging going set mode");
            _sendStageCommand(link, stagingState_t::SET_MODE);
            break;

        case stagingState_t::SET_MODE:
        case stagingState_t::ARM:
        case stagingState_t::TAKEOFF:
            /* waiting for the COMMAND_ACK (which advances the state); resend
               on the retry timer, since UDP loses */
            if (tNow - link.lastStageCmdTx_seconds > STAGE_CMD_RETRY_S)
            {
                _sendStageCommand(link, st);
            }
            break;

        case stagingState_t::CLIMB:
            /* Once airborne, drive a GUIDED setpoint that both holds the climb
               to the staging altitude and yaws the vehicle to the mission
               heading, so STAGED already faces the trajectory start (no yaw
               jump at Start). On the ground path NAV_TAKEOFF performs the
               liftoff (a position setpoint alone will not lift a landed
               vehicle); we only take over with setpoints once off the ground.
               When already airborne (takeoff skipped, e.g. re-staging after a
               maneuver) the setpoint drives the whole climb. assembled.z is
               the CDS "up" height above the takeoff origin */
            if ((link.stageSkipTakeoff ||
                 link.assembled.z > STAGE_MIN_AIRBORNE_M) &&
                tNow - link.lastSetpointTx_seconds >=
                    m_params.setpointPeriod_seconds)
            {
                _sendClimbSetpoint(link);
                link.lastSetpointTx_seconds = tNow;
            }
            /* STAGED: at the target altitude, on the target heading, in a
               stable hover held long enough */
            if (std::abs(link.assembled.z - m_stageAltitude.load()) <
                    STAGE_ALT_TOL_M &&
                std::abs(wrapPi(link.assembled.yaw - m_stageYaw.load())) <
                    STAGE_YAW_TOL_RAD &&
                link.stableSince_seconds >= 0.0 &&
                (tNow - link.stableSince_seconds >=
                 m_params.stabilityHoldTime_seconds))
            {
                m_stagingState = stagingState_t::STAGED;
                CDS_LOG_INFO(logger, "Staging completed");
            }
            break;

        case stagingState_t::STAGED:
            break;
    }
}

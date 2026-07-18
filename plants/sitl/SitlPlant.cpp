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

/* sole gateway to the vendored MAVLink headers: compiling it here keeps the
   protocol pins guarding every build of this library */
#include "mavlink_pin.hpp"

#include "UdpTransport.hpp"

#include <chrono>
#include <cmath>

using namespace plants;

using Clock = std::chrono::steady_clock;
using FpSeconds = std::chrono::duration<double>;

namespace
{
    /* our identity on the wire: a ground-control-station peer */
    constexpr uint8_t OUR_SYSTEM_ID = 255;
    constexpr uint8_t OUR_COMPONENT_ID = MAV_COMP_ID_MISSIONPLANNER;

    constexpr double HEARTBEAT_PERIOD_S = 1.0;
    constexpr double BIND_RETRY_PERIOD_S = 1.0;

    /* UDP is lossy: keep asking for the telemetry streams until they flow */
    constexpr double TELEMETRY_REREQUEST_PERIOD_S = 2.0;

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

    /* telemetry assembly: LOCAL_POSITION_NED (position + velocity) and
       ATTITUDE (angles + rates) arrive as separate messages; they are
       merged, already converted to CDS/ENU, into this running sample and
       published on every fresh LOCAL_POSITION_NED */
    core_state_t assembled;
    bool havePosition;
    double lastPositionRx_seconds;

    Link() : peerKnown(false),
             peer({}),
             fcSystemId(0),
             fcComponentId(0),
             epoch(Clock::now()),
             lastValidRx_seconds(0.0),
             lastHeartbeatTx_seconds(0.0),
             lastTelemetryReq_seconds(0.0),
             assembled({}),
             havePosition(false),
             lastPositionRx_seconds(0.0)
    {
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
                                   .linkTimeout_seconds = 2.0}),
                         m_threadRun(false),
                         m_missionRun(false),
                         m_linkState(linkState_t::DISCONNECTED)
{

}

SitlPlant::~SitlPlant()
{
    Stop();
    Disconnect();
}

bool SitlPlant::SetPlantParams(const std::any& params)
{
    if (m_thread.joinable())
    {
        // Cannot reconfigure while connected, error
        return true;
    }

    if (params.type() != typeid(sitlParams_t&))
    {
        // Err
        return true;
    }

    const auto& p = std::any_cast<const sitlParams_t&>(params);

    if (p.host.empty() || p.port == 0 ||
        p.setpointPeriod_seconds <= 0 || p.telemetryPeriod_seconds <= 0 ||
        p.linkTimeout_seconds <= 0)
    {
        // Invalid parameters, error
        return true;
    }

    m_params = p;
    return false;
}

bool SitlPlant::Connect(void)
{
    if (m_thread.joinable())
    {
        // Already connected, error
        return true;
    }

    m_threadRun = true;
    m_thread = std::thread(&SitlPlant::_commLoop, this);

    return false;
}

bool SitlPlant::Disconnect(void)
{
    /* idempotent: disconnecting a disconnected plant is not an error */
    if (!m_thread.joinable())
    {
        return false;
    }

    m_missionRun = false;
    m_threadRun = false;
    m_thread.join();
    m_linkState = linkState_t::DISCONNECTED;

    return false;
}

bool SitlPlant::Start(void)
{
    if (!m_thread.joinable())
    {
        // Mission on a disconnected link, error
        return true;
    }

    if (m_missionRun)
    {
        // Already started, error
        return true;
    }

    m_missionRun = true;
    return false;
}

bool SitlPlant::Stop(void)
{
    /* idempotent: stopping a stopped mission is not an error */
    m_missionRun = false;
    return false;
}

SitlPlant::linkState_t SitlPlant::GetLinkState(void) const
{
    return m_linkState;
}

void SitlPlant::_commLoop(void)
{
    Link link;

    while (m_threadRun)
    {
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
            m_linkState = linkState_t::DISCONNECTED;
        }

        if (link.peerKnown &&
            tNow - link.lastHeartbeatTx_seconds >= HEARTBEAT_PERIOD_S)
        {
            _sendHeartbeat(link);
            link.lastHeartbeatTx_seconds = tNow;
        }

        /* once the flight controller is identified, ask for the telemetry
           streams and keep re-asking until they actually flow (UDP loss) */
        if (m_linkState == linkState_t::READY &&
            tNow - link.lastPositionRx_seconds > TELEMETRY_REREQUEST_PERIOD_S &&
            tNow - link.lastTelemetryReq_seconds >= TELEMETRY_REREQUEST_PERIOD_S)
        {
            _requestTelemetryStreams(link);
            link.lastTelemetryReq_seconds = tNow;
        }

        /* m_missionRun gates the setpoint streaming (next phases) */
    }

    m_linkState = linkState_t::DISCONNECTED;
}

void SitlPlant::_processInbound(Link& link)
{
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

                case MAVLINK_MSG_ID_ATTITUDE:
                {
                    mavlink_attitude_t attitude;
                    mavlink_msg_attitude_decode(&message, &attitude);

                    /* NED aircraft body → CDS/ENU: roll unchanged, pitch and
                       yaw negated (Y and Z axes flip), heading measured from
                       East instead of North. Body rates follow the same axis
                       flip */
                    link.assembled.roll = attitude.roll;
                    link.assembled.pitch = -attitude.pitch;
                    link.assembled.yaw = wrapPi(PI / 2.0 - attitude.yaw);
                    link.assembled.roll_dot = attitude.rollspeed;
                    link.assembled.pitch_dot = -attitude.pitchspeed;
                    link.assembled.yaw_dot = -attitude.yawspeed;
                    break;
                }

                case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                {
                    mavlink_local_position_ned_t lpos;
                    mavlink_msg_local_position_ned_decode(&message, &lpos);

                    /* NED → CDS/ENU: East=N_y, North=N_x, Up=-N_z, same for
                       the velocity components */
                    link.assembled.x = lpos.y;
                    link.assembled.y = lpos.x;
                    link.assembled.z = -lpos.z;
                    link.assembled.x_dot = lpos.vy;
                    link.assembled.y_dot = lpos.vx;
                    link.assembled.z_dot = -lpos.vz;

                    link.havePosition = true;
                    link.lastPositionRx_seconds = link.Now();

                    /* the vehicle's own boot time is the moment the sample
                       was TAKEN: publishing it makes the link latency
                       observable, exactly as the BasePlant contract wants */
                    PublishMeasurements(link.assembled,
                                        lpos.time_boot_ms / 1000.0);
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

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

    Link() : peerKnown(false),
             peer({}),
             fcSystemId(0),
             fcComponentId(0),
             epoch(Clock::now()),
             lastValidRx_seconds(0.0),
             lastHeartbeatTx_seconds(0.0)
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
            m_linkState = linkState_t::DISCONNECTED;
        }

        if (link.peerKnown &&
            tNow - link.lastHeartbeatTx_seconds >= HEARTBEAT_PERIOD_S)
        {
            _sendHeartbeat(link);
            link.lastHeartbeatTx_seconds = tNow;
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
                                   &message, &status) == 0)
            {
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

            if (message.msgid == MAVLINK_MSG_ID_HEARTBEAT)
            {
                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&message, &heartbeat);

                /* the flight controller is identified by the heartbeat of
                   its autopilot component; a peer declaring a different
                   protocol generation is refused */
                if (message.compid == MAV_COMP_ID_AUTOPILOT1 &&
                    heartbeat.autopilot != MAV_AUTOPILOT_INVALID &&
                    heartbeat.mavlink_version == MAVLINK_VERSION)
                {
                    link.fcSystemId = message.sysid;
                    link.fcComponentId = message.compid;
                    m_linkState = linkState_t::READY;
                }
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

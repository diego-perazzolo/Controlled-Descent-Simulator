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
// File        : SitlPlant.hpp
// Description : ArduPilot SITL plant: drives an ArduPilot Copter SITL over
//               MAVLink 2 / UDP. Commands stream out as Guided-mode
//               SET_POSITION_TARGET_LOCAL_NED setpoints, telemetry
//               (LOCAL_POSITION_NED + ATTITUDE) comes back as measurements;
//               the NED <-> CDS (ENU) frame conversion is confined entirely
//               to this plant. MAVLink types never leak out of the .cpp.
//               Lifecycle: Connect (at attach) brings up the link — the
//               vehicle is expected connected, staged and hovering before
//               any mission; Start/Stop (from Run/Stop) only toggle the
//               setpoint streaming on the warm link.
//               Link layer: UDP listener (GCS-style, the peer is learned
//               from the source of the first valid MAVLink 2 datagram),
//               1 Hz heartbeat out, session state machine DISCONNECTED →
//               CONNECTED → READY with a silence watchdog: an incompatible
//               peer (MAVLink 1, or diverged message definitions failing
//               CRC) produces no valid traffic and is torn down explicitly.
//               On READY it requests LOCAL_POSITION_NED + ATTITUDE streams
//               (SET_MESSAGE_INTERVAL), decodes them into a measurement
//               (NED → CDS/ENU conversion confined here) and publishes it
//               timestamped with the vehicle's on-board time.
//               Frame alignment is two-regime (pose = position + yaw):
//               while staging (mission off) the pose is zeroed at the CDS
//               origin, so the ghost drifts near the origin; at mission
//               Start the first command captures a mission offset that makes
//               the ghost coincide with the trajectory's first point and
//               maps outbound setpoints back into the vehicle's frame.
//               Start is gated: the vehicle must be READY and held still
//               (stability params) long enough. This phase (4a) assumes the
//               vehicle already airborne in GUIDED; auto-staging is 4b.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "BasePlant.hpp"

namespace plants
{

    // ArduPilot SITL plant: MAVLink 2 over UDP, Guided-mode setpoints
    class SitlPlant : public CDS::BasePlant
    {
        public:

        typedef struct
        {
            /* local UDP endpoint the plant listens on (the SITL streams
               its MAVLink out to this address, e.g. --out udp:host:port;
               the peer is learned from the first valid datagram). Host must
               be a non-empty IPv4 literal, port must be non-zero */
            std::string host;
            uint16_t port;

            /* outbound SET_POSITION_TARGET_LOCAL_NED streaming period, in
               seconds — the clock of the communication loop. Must be > 0 */
            double setpointPeriod_seconds;

            /* inbound telemetry period requested to the SITL for
               LOCAL_POSITION_NED and ATTITUDE, in seconds. Must be > 0 */
            double telemetryPeriod_seconds;

            /* link watchdog: no valid inbound traffic for longer than this
               means the link is lost, in seconds. Must be > 0 */
            double linkTimeout_seconds;

            /* mission-readiness gate: the vehicle counts as "still" while its
               measured speed stays at or below this threshold (m/s), and is
               ready to start only after it has stayed still continuously for
               stabilityHoldTime_seconds. Start is refused otherwise. Both
               must be > 0 */
            double stabilityVelThreshold_ms;
            double stabilityHoldTime_seconds;
        } sitlParams_t;

        /* Link session state, observable from any thread */
        enum class linkState_t : uint8_t
        {
            DISCONNECTED = 0,   /* no valid MAVLink 2 traffic */
            CONNECTED,          /* valid traffic, flight controller not yet identified */
            READY               /* flight controller heartbeat identified */
        };

        SitlPlant();
        virtual ~SitlPlant();

        /* Set parameters (sitlParams_t), only while disconnected.
           Returns true on error */
        virtual bool SetPlantParams(const std::any& params) override;

        /* Bring up / tear down the communication thread (and, with the link
           layer, socket and MAVLink session). Disconnect is idempotent.
           Returns true on error */
        virtual bool Connect(void) override;
        virtual bool Disconnect(void) override;

        /* Mission toggles: enable / disable the setpoint streaming on the
           connected link. Start requires a connected link AND a ready
           vehicle (see IsReadyToStart); Stop is idempotent.
           Returns true on error */
        virtual bool Start(void) override;
        virtual bool Stop(void) override;

        /* Current link session state (any thread) */
        linkState_t GetLinkState(void) const;

        /* True when the vehicle may begin a mission: link READY and the
           vehicle held still (per the stability params) long enough. Any
           thread */
        bool IsReadyToStart(void) const;

        private:

        /* communication-thread context (transport, parser, peer, timers):
           lives on the thread stack, defined in the .cpp so that neither
           MAVLink nor socket types leak out of this header */
        struct Link;

        /* body of the communication thread */
        void _commLoop(void);

        /* one bounded blocking receive (the loop pacing) + drain, parsing
           every byte and driving the session state machine; decoded
           telemetry is converted and published from here */
        void _processInbound(Link& link);

        /* our 1 Hz GCS heartbeat towards the learned peer */
        void _sendHeartbeat(Link& link);

        /* request the telemetry streams we consume (LOCAL_POSITION_NED and
           ATTITUDE) at the configured rate, via SET_MESSAGE_INTERVAL */
        void _requestTelemetryStreams(Link& link);

        /* send one SET_POSITION_TARGET_LOCAL_NED for the given command,
           applying the mission offset and the CDS/ENU → NED conversion */
        void _sendSetpoint(Link& link, const plantCommands_t& commands);

        sitlParams_t m_params;
        std::thread m_thread;
        std::atomic<bool> m_threadRun;
        std::atomic<bool> m_missionRun;
        std::atomic<linkState_t> m_linkState;
        std::atomic<bool> m_readyToStart;
    };

} // namespace plants

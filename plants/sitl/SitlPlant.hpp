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
//               Skeleton phase: params + communication-thread lifecycle;
//               the link layer (socket, heartbeat, parser) lands next.
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
            /* MAVLink UDP endpoint of the SITL. Host must be non-empty,
               port must be non-zero */
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
        } sitlParams_t;

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
           connected link. Start requires a connected link; Stop is
           idempotent. Returns true on error */
        virtual bool Start(void) override;
        virtual bool Stop(void) override;

        private:

        /* body of the communication thread */
        void _commLoop(void);

        sitlParams_t m_params;
        std::thread m_thread;
        std::atomic<bool> m_threadRun;
        std::atomic<bool> m_missionRun;
    };

} // namespace plants

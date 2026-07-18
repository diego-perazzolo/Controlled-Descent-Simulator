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
// File        : BasePlant.hpp
// Description : Base plant class: contract between the SystemManager tick and
//               an external system under control. The mailbox exchange
//               (triple buffers, sequence numbering) is implemented here once
//               for every plant (NVI); implementations (SITL, HIL links with
//               their communication threads and third-party libraries) live
//               OUTSIDE the core and only provide the link lifecycle
//               (Connect/Disconnect), the mission toggles (Start/Stop) and
//               params, plus their communication thread, which talks to the
//               tick solely through the protected Fetch/Publish methods.
//               Lifecycle: the LINK lives from attach to detach (telemetry
//               flows and the vehicle can be staged while the simulation is
//               still stopped); the MISSION runs between Run() and Stop().
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <any>
#include <cstdint>

#include "../core_defs.hpp"
#include "TripleBuffer.hpp"

namespace CDS
{

    // Abstract plant class
    class BasePlant
    {
        public:

        /* Command sample: deposited by the tick, consumed by the plant at its
           own pace */
        typedef struct
        {
            /* simulation time at which the tick sampled the reference */
            core_coord_t time_seconds;

            /* trajectory reference at time_seconds (sampled by the tick) */
            Reference_t reference;
        } plantCommands_t;

        /* Measurement sample: produced by the plant, read by the tick.
           Freshness is explicit: the consumer detects re-reads (same
           sequence), overwritten samples (sequence gaps) and stale links
           (old plantTime_seconds) */
        typedef struct
        {
            /* increments on every fresh sample; gaps = overwritten samples */
            uint32_t sequence;

            /* plant-side monotonic time of the sample, in seconds */
            core_coord_t plantTime_seconds;

            /* last known state of the plant */
            core_state_t state;
        } plantMeasurements_t;

        BasePlant();

        /* Virtual methods */

        virtual ~BasePlant();

        /* Set plant parameters (implementation-specific), only while
           disconnected. Returns true on error */
        virtual bool SetPlantParams(const std::any& params) = 0;

        /* Link lifecycle, driven by attach/detach. Connect brings up the
           implementation's communication (thread, socket): telemetry may
           already flow and the vehicle be staged while the simulation is
           stopped. Disconnect tears it down and is idempotent; every
           implementation must ensure it in its destructor. Both are called
           under the SystemManager lock and must not block on I/O — link
           bring-up belongs to the communication thread, which may never
           call back into the core. Returns true on error */
        virtual bool Connect(void) = 0;
        virtual bool Disconnect(void) = 0;

        /* Mission toggles, driven by Run()/Stop(): enable / disable the
           tracking of commands on an already-connected link. Non-blocking
           (called under the SystemManager lock); Start on a disconnected
           plant is an error, Stop is idempotent. Returns true on error */
        virtual bool Start(void) = 0;
        virtual bool Stop(void) = 0;

        /* Tick-side exchange — called by the SystemManager under its lock at
           every tick. Non-virtual by design: implemented here on the triple
           buffers, so it is non-blocking and constant-time by construction
           (no I/O, no unbounded waits) and no derived class can break it. */

        /* Deposit the command sample for the plant. Latest-wins semantics: a
           fresh sample overwrites an unconsumed one (a real actuator only
           holds the last command). Returns true on error */
        bool PushCommands(const plantCommands_t& commands);

        /* Read the most recent measurement sample published by the plant.
           Non-consuming: re-reading returns the same sample (same sequence).
           Returns true on error (no sample published yet) */
        bool PullMeasurements(plantMeasurements_t& measurements);

        /* Read side, mirror of BaseModel: last known state / plant time of
           the last sample. Tick-side (same reader as PullMeasurements: call
           under the SystemManager lock). Returns true on error */
        bool GetState(core_state_t& state);
        bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds);

        protected:

        /* Communication-thread side of the mailboxes — for the SINGLE
           communication thread owned by the implementation. */

        /* Get the most recent command deposited by the tick. Non-consuming.
           Returns true on error (no command deposited yet) */
        bool FetchCommands(plantCommands_t& commands);

        /* Publish a fresh measurement sample. The base assigns the sequence
           number; plantTime_seconds is the implementation's measurement
           timestamp (when the sample was TAKEN, not when it is published:
           the difference is exactly the latency we want observable).
           Returns true on error */
        bool PublishMeasurements(const core_state_t& state, core_coord_t plantTime_seconds);

        private:

        cds_sync::TripleBuffer<plantCommands_t> m_commandsBuffer;
        cds_sync::TripleBuffer<plantMeasurements_t> m_measurementsBuffer;
        uint32_t m_publishSequence;   // communication-thread private
    };
}

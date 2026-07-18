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
// File        : LoopbackPlant.hpp
// Description : SITL loopback plant: echoes the commanded reference back as
//               measured state, with configurable sample period, latency and
//               dropout rate. Serves as the plumbing test double AND as the
//               reference implementation pattern for real plants (link
//               thread in Connect/Disconnect, mission flag in Start/Stop,
//               loop of FetchCommands → transform → PublishMeasurements).
//               While connected with the mission stopped it publishes the
//               held state (the vehicle hovering in place); once started it
//               tracks the commanded reference.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <atomic>
#include <deque>
#include <random>
#include <thread>

#include "BasePlant.hpp"

namespace plants
{

    // Loopback plant: perfect tracking of the commanded reference, delayed
    class LoopbackPlant : public CDS::BasePlant
    {
        public:

        typedef struct
        {
            /* plant-side sampling period (how often the communication loop
               fetches / publishes), in seconds. Must be > 0 */
            double samplePeriod_seconds;

            /* artificial measurement latency, in seconds. Must be >= 0 */
            double latency_seconds;

            /* probability [0, 1) of skipping a publication (sensor dropout) */
            double dropRate;
        } loopbackParams_t;

        LoopbackPlant();
        virtual ~LoopbackPlant();

        /* Set parameters (loopbackParams_t), only while disconnected.
           Returns true on error */
        virtual bool SetPlantParams(const std::any& params) override;

        /* Bring up / tear down the communication thread. Disconnect is
           idempotent. Returns true on error */
        virtual bool Connect(void) override;
        virtual bool Disconnect(void) override;

        /* Mission toggles: enable / disable the echo of commands. Start
           requires a connected link; Stop is idempotent.
           Returns true on error */
        virtual bool Start(void) override;
        virtual bool Stop(void) override;

        private:

        /* body of the communication thread */
        void _commLoop(void);

        /* one fetched command with its plant-side arrival time */
        typedef struct
        {
            double t_seconds;
            plantCommands_t cmd;
        } delayEntry_t;

        loopbackParams_t m_params;
        std::thread m_thread;
        std::atomic<bool> m_threadRun;
        std::atomic<bool> m_missionRun;

        /* communication-thread private */
        std::deque<delayEntry_t> m_delayLine;
        std::mt19937 m_rng;
        std::uniform_real_distribution<double> m_dist;
    };

} // namespace plants

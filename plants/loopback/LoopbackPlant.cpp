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
// File        : LoopbackPlant.cpp
// Description : SITL loopback plant: echoes the commanded reference back as
//               measured state, with configurable sample period, latency and
//               dropout rate
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "LoopbackPlant.hpp"
#include <chrono>

using namespace plants;

using Clock = std::chrono::steady_clock;
using FpSeconds = std::chrono::duration<double>;

LoopbackPlant::LoopbackPlant() : m_params({0.01, 0.0, 0.0}),
                                 m_threadRun(false),
                                 m_rng(std::random_device{}()),
                                 m_dist(0.0, 1.0)
{

}

LoopbackPlant::~LoopbackPlant()
{
    Stop();
}

bool LoopbackPlant::SetPlantParams(const std::any& params)
{
    if (m_thread.joinable())
    {
        // Cannot reconfigure while running, error
        return true;
    }

    if (params.type() != typeid(loopbackParams_t&))
    {
        // Err
        return true;
    }

    const auto& p = std::any_cast<const loopbackParams_t&>(params);

    if (p.samplePeriod_seconds <= 0 || p.latency_seconds < 0 ||
        p.dropRate < 0 || p.dropRate >= 1)
    {
        // Invalid parameters, error
        return true;
    }

    m_params = p;
    return false;
}

bool LoopbackPlant::Start(void)
{
    if (m_thread.joinable())
    {
        // Already started, error
        return true;
    }

    m_threadRun = true;
    m_thread = std::thread(&LoopbackPlant::_commLoop, this);

    return false;
}

bool LoopbackPlant::Stop(void)
{
    /* idempotent: stopping a stopped plant is not an error */
    if (!m_thread.joinable())
    {
        return false;
    }

    m_threadRun = false;
    m_thread.join();

    return false;
}

void LoopbackPlant::_commLoop(void)
{
    const auto start = Clock::now();
    m_delayLine.clear();

    while (m_threadRun)
    {
        std::this_thread::sleep_for(FpSeconds(m_params.samplePeriod_seconds));

        plantCommands_t cmd;
        if (FetchCommands(cmd))
        {
            // No command deposited yet
            continue;
        }

        const double tNow = FpSeconds(Clock::now() - start).count();
        m_delayLine.push_back({tNow, cmd});

        /* keep only the newest entry that already aged past the latency:
           that is the sample the "vehicle" is measured at now */
        while (m_delayLine.size() >= 2 &&
               tNow - m_delayLine[1].t_seconds >= m_params.latency_seconds)
        {
            m_delayLine.pop_front();
        }

        if (tNow - m_delayLine.front().t_seconds < m_params.latency_seconds)
        {
            // Nothing old enough to be observable yet
            continue;
        }

        if (m_dist(m_rng) < m_params.dropRate)
        {
            // Simulated sensor dropout
            continue;
        }

        const delayEntry_t& observed = m_delayLine.front();

        /* perfect tracking: the measured state IS the commanded reference */
        core_state_t state = {};
        state.x = observed.cmd.reference.pos[0];
        state.y = observed.cmd.reference.pos[1];
        state.z = observed.cmd.reference.pos[2];
        state.x_dot = observed.cmd.reference.vel[0];
        state.y_dot = observed.cmd.reference.vel[1];
        state.z_dot = observed.cmd.reference.vel[2];
        state.yaw = observed.cmd.reference.yaw;
        state.yaw_dot = observed.cmd.reference.yawRate;

        PublishMeasurements(state, observed.t_seconds);
    }
}

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
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"

#include <array>
#include <chrono>
#include <cstdint>

using namespace plants;

using Clock = std::chrono::steady_clock;
using FpSeconds = std::chrono::duration<double>;

static const auto logger = cds_log::registry().module("Loopback plant");
static const auto profile = cds_profile::registry().module("Loopback plant");

// Plant data recorder (black-box wide CSV, server-side): every published
// measurement — plant time, sample sequence, and the full 12-field core state.
// One plant records at a time (the active-plant slot); it runs alongside the
// active model recorder during a mission.
static cds_record::Recorder<double, 14, 4096> recorder("Loopback plant", {{
    "t_plant", "seq",
    "x", "y", "z", "x_dot", "y_dot", "z_dot",
    "roll", "pitch", "yaw", "roll_dot", "pitch_dot", "yaw_dot",
}});

// Record one published sample (called on the communication thread right after
// PublishMeasurements). seq mirrors the publish count so telemetry gaps show.
static void _recordSample(const core_state_t& s, double t)
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

LoopbackPlant::LoopbackPlant() : m_params({.samplePeriod_seconds = 0.01,
                                           .latency_seconds = 0.0,
                                           .dropRate = 0.0}),
                                 m_threadRun(false),
                                 m_missionRun(false),
                                 m_rng(std::random_device{}()),
                                 m_dist(0.0, 1.0)
{
    recorder.activateAsPlant(); // this plant owns the plant data recorder
    CDS_LOG_INFO(logger, "Plant created");
}

LoopbackPlant::~LoopbackPlant()
{
    Stop();
    Disconnect();
    CDS_LOG_INFO(logger, "Plant released");
}

bool LoopbackPlant::SetPlantParams(const std::any& params)
{
    if (m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Cannot reconfigure plant while it is running");
        return true;
    }

    if (params.type() != typeid(loopbackParams_t&))
    {
        CDS_LOG_ERROR(logger, "Wrong params type");
        return true;
    }

    const auto& p = std::any_cast<const loopbackParams_t&>(params);

    if (p.samplePeriod_seconds <= 0 || p.latency_seconds < 0 ||
        p.dropRate < 0 || p.dropRate >= 1)
    {
        CDS_LOG_ERROR(logger, "Invalid parameter value");
        return true;
    }

    CDS_LOG_INFO(logger, "Plant params succesfully set");
    m_params = p;

    // Recorder run metadata for this plant.
    recorder.clearMeta();
    recorder.addMeta("plant", "Loopback");
    recorder.addMeta("sample_period_s", p.samplePeriod_seconds);
    recorder.addMeta("latency_s", p.latency_seconds);
    recorder.addMeta("drop_rate", p.dropRate);

    return false;
}

bool LoopbackPlant::Connect(void)
{
    if (m_thread.joinable())
    {
        CDS_LOG_ERROR(logger, "Plant already connected");
        return true;
    }

    m_threadRun = true;
    m_thread = std::thread(&LoopbackPlant::_commLoop, this);

    CDS_LOG_INFO(logger, "Starting connection");

    return false;
}

bool LoopbackPlant::Disconnect(void)
{
    /* idempotent: disconnecting a disconnected plant is not an error */
    if (!m_thread.joinable())
    {
        CDS_LOG_WARN(logger, "Disconnect: no plant is currently connected");
        return false;
    }

    m_missionRun = false;
    m_threadRun = false;
    m_thread.join();

    CDS_LOG_INFO(logger, "Plant disconnected");
    return false;
}

bool LoopbackPlant::Start(void)
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

    CDS_LOG_INFO(logger, "Mission started");
    m_missionRun = true;
    return false;
}

bool LoopbackPlant::Stop(void)
{
    /* idempotent: stopping a stopped mission is not an error */
    CDS_LOG_INFO(logger, "Mission stopped");
    m_missionRun = false;
    return false;
}

void LoopbackPlant::_commLoop(void)
{
    const auto start = Clock::now();

    /* last echoed state: what the idle "vehicle" holds while the mission is
       stopped (initially at the origin) */
    core_state_t held = {};

    m_delayLine.clear();

    while (m_threadRun)
    {
        std::this_thread::sleep_for(FpSeconds(m_params.samplePeriod_seconds));

        // measure the per-cycle work only, not the pacing sleep above
        CDS_PROFILE(profile, "Communication loop");
        const double tNow = FpSeconds(Clock::now() - start).count();

        if (!m_missionRun)
        {
            /* connected, mission stopped: hover in place. Telemetry keeps
               flowing (that is what a live link does), commands are not
               tracked and stale ones must not leak into the next mission */
            m_delayLine.clear();

            if (!(m_dist(m_rng) < m_params.dropRate))
            {
                /* holding position: no residual rates */
                core_state_t idle = held;
                idle.x_dot = 0;
                idle.y_dot = 0;
                idle.z_dot = 0;
                idle.yaw_dot = 0;
                PublishMeasurements(idle, tNow);
                _recordSample(idle, tNow);
            }
            continue;
        }

        plantCommands_t cmd;
        if (FetchCommands(cmd))
        {
            // No command deposited yet
            continue;
        }

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

        held = state;
        PublishMeasurements(state, observed.t_seconds);
        _recordSample(state, observed.t_seconds);
    }
}

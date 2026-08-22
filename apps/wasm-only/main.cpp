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
// File        : main.cpp
// Description : entry point of the wasm-only application. Sets the timer used
//                  for simulating ticks used for physics or communication
//                  (emulation of a Real-time timer)
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <emscripten.h>
#include "core_defs.hpp"
#include "log.hpp"
#include "LogSinks.hpp"
#include "LogUiSink.hpp"
#include "profile.hpp"
#include <chrono>

using Clock = std::chrono::steady_clock;

/* Elapsed is measured as floating-point seconds: an integer duration_cast would
   truncate to zero once a frame gets faster than its unit, silently freezing the
   simulation */
using FpSeconds = std::chrono::duration<double>;

static const auto logger = cds_log::registry().module("Main");
static const auto profile = cds_profile::registry().module("Main");

static Clock::time_point _lastTime;
static int _is_sys_init = 0;
static double _tickAccumulator = 0.0; // pure-sim fixed-step accumulator (sim-seconds owed)

extern bool g_core_tick(core_coord_t dt_seconds); // global function from core.cpp
extern bool g_core_getTickPeriod(core_coord_t& tickPeriod_second); // global function from core.cpp
extern bool g_core_getTickRate(core_coord_t& rate); // global function from core.cpp
extern bool g_core_isPlantAttached(void); // global function from core.cpp

/* Real-time pacing, used while a plant is attached: the plant owns wall time, so
   the model gets one tick per elapsed period, fed the measured elapsed. A stall
   (backgrounded tab, model re-init) is clamped so it cannot feed a huge dt. */
static void _tickRealTime(double elapsed_seconds, core_coord_t tickPeriodSeconds, Clock::time_point now)
{
    if (elapsed_seconds < tickPeriodSeconds) return;

    _lastTime = now;
    core_coord_t dt_seconds = static_cast<core_coord_t>(elapsed_seconds);
    const core_coord_t dtMax_seconds = 3 * tickPeriodSeconds;
    if (dt_seconds > dtMax_seconds) dt_seconds = dtMax_seconds;

    CDS_PROFILE(profile, "Ticking");
    g_core_tick(dt_seconds);
    /* Single-threaded build: publish the profiler aggregates (writer side) and
       pump them into the UI cache (reader side) on this same thread */
    cds_profile::registry().publish();
    cds_profile::registry().pump();
}

/* Sim time owed for this frame (elapsed x rate), added to the backlog and capped.
   What the cap discards is what a machine that cannot sustain the rate actually
   loses, so it is reported as the share of the requested speed still simulated —
   rate-limited, since a saturated backlog saturates on every frame. */
static void _accrueSimTime(double elapsed_seconds, core_coord_t rate, core_coord_t tickPeriodSeconds)
{
    const double requested_seconds = elapsed_seconds * rate;
    _tickAccumulator += requested_seconds;

    /* Backlog cap: how much owed sim time may survive a stall, so a stall cannot
       burst into a catch-up storm. Kept at a few tick periods: a fixed cap
       smaller than the period would never be cleared by the sub-stepping loop,
       freezing the simulation outright. */
    const double MAX_BACKLOG_SECONDS = (4.0 * tickPeriodSeconds > 0.25)
                                     ? 4.0 * tickPeriodSeconds : 0.25;
    if (_tickAccumulator <= MAX_BACKLOG_SECONDS) return;

    /* Sim time asked for this frame that will never be simulated. The backlog is
       capped on every pass, so the loss cannot exceed what this frame requested:
       0 <= dropped_seconds <= requested_seconds. */
    const double dropped_seconds = _tickAccumulator - MAX_BACKLOG_SECONDS;
    _tickAccumulator = MAX_BACKLOG_SECONDS;

    static Clock::time_point lastRateWarn{};
    const auto nowW = Clock::now();
    if (requested_seconds <= 0.0 || FpSeconds(nowW - lastRateWarn).count() < 2.0) return;
    lastRateWarn = nowW;

    /* Integer percent: the logger's fallback formatter has no format specs */
    const double delivered_seconds = requested_seconds - dropped_seconds;
    int deliveredPercent = static_cast<int>(100.0 * delivered_seconds / requested_seconds + 0.5);
    if (deliveredPercent < 0)  deliveredPercent = 0;
    if (deliveredPercent > 99) deliveredPercent = 99; // reported only while falling behind
    CDS_LOG_WARN(logger, "Cannot keep up with the requested {}x speed: simulating at about {}% of it (model too slow for this build; the sim time not simulated is dropped, not queued)",
                 static_cast<double>(rate), deliveredPercent);
}

/* Spend the owed sim time in whole fixed steps, bounded by a wall-time budget
   (and a hard step cap) so an expensive model cannot stall the frame: it runs
   below the requested rate instead of freezing it. Returns the steps taken. */
static int _runFixedSteps(core_coord_t tickPeriodSeconds)
{
    constexpr double STEP_BUDGET_SECONDS = 0.008; // wall time to spend sub-stepping per frame
    constexpr int    MAX_STEPS = 2000;            // hard fallback cap

    const auto subStart = Clock::now();
    int steps = 0;

    CDS_PROFILE(profile, "Ticking");
    while (_tickAccumulator >= tickPeriodSeconds && steps < MAX_STEPS)
    {
        g_core_tick(tickPeriodSeconds);
        _tickAccumulator -= tickPeriodSeconds;
        ++steps;
        if (FpSeconds(Clock::now() - subStart).count() >= STEP_BUDGET_SECONDS) break;
    }
    return steps;
}

/* One animation frame: measure the elapsed wall time and hand it to the pacing
   policy — real-time when a plant paces the run, fixed rate-scaled steps in a
   pure simulation — then drain the log queue off the tick's critical work. */
static void _tick_generator(void)
{
    core_coord_t tickPeriodSeconds;
    if(g_core_getTickPeriod(tickPeriodSeconds) || tickPeriodSeconds <= 0)
    {
        /* No model yet, or tick period not configured: re-anchor the time
           base on the next valid pass */
        _is_sys_init = 0;
        _tickAccumulator = 0.0;
        return;
    }

    /* Only the first time */
    if (!_is_sys_init)
    {
        _is_sys_init = 1;
        _lastTime = Clock::now();
        _tickAccumulator = 0.0;
    }

    const auto t1 = Clock::now();
    const double elapsed_seconds = FpSeconds(t1 - _lastTime).count();

    if (g_core_isPlantAttached())
    {
        _tickRealTime(elapsed_seconds, tickPeriodSeconds, t1);
    }
    else
    {
        /* Pure simulation: the step stays at the nominal period (smooth and
           deterministic) while the rate multiplier sets how much sim time to
           cover per wall second (1.0 = real-time, 2.0 = 2x) */
        _lastTime = t1;
        core_coord_t rate = 1.0;
        g_core_getTickRate(rate);
        if (rate <= 0.0) rate = 1.0;

        _accrueSimTime(elapsed_seconds, rate, tickPeriodSeconds);
        if (_runFixedSteps(tickPeriodSeconds) > 0)
        {
            cds_profile::registry().publish();
            cds_profile::registry().pump();
        }
    }

    /* One thread here, so producer and consumer are the same: this is the log
       ring's single drain-point (no background thread as on the server) */
    cds_log::registry().drain();
}

int main()
{
    /* Log sink: stderr maps to the browser console in the wasm runtime. No file
       sink (no real filesystem) and no drain thread (single-threaded runtime) */
    static cds_log::ConsoleSink consoleSink(stderr);
    cds_log::registry().addSink(&consoleSink);
    cds_log::registry().addSink(&cds_log::uiSink()); // recent-lines buffer for the frontend

    /* Creating a timer which executes _tick_generator at the refresh rate of the screen */
    emscripten_set_main_loop(_tick_generator, 0, 0);
    return 0;
}
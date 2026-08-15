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

static const auto logger = cds_log::registry().module("Main");
static const auto profile = cds_profile::registry().module("Main");

static Clock::time_point _lastTime;
static int _is_sys_init = 0;

extern bool g_core_tick(core_coord_t dt_seconds); // global function from core.cpp
extern bool g_core_getTickPeriod(core_coord_t& tickPeriod_second); // global function from core.cpp

/* tick the system at rate 1/tick_period_ms, unless ticking system takes too much (> 1/tick_period_ms) */
static void _tick_generator(void)
{
    core_coord_t tickPeriodSeconds;
    if(g_core_getTickPeriod(tickPeriodSeconds) || tickPeriodSeconds <= 0)
    {
        /* No model yet, or tick period not configured: re-anchor the time
           base on the next valid pass */
        _is_sys_init = 0;
        return;
    }

    /* Only the first time */
    if (!_is_sys_init)
    {
        _is_sys_init = 1;
        _lastTime = Clock::now();
    }

    /* Elapsed measured as floating-point seconds: an integer duration_cast
       would truncate to zero once a frame gets faster than its unit,
       silently freezing the simulation */
    using FpSeconds = std::chrono::duration<double>;

    auto t1 = Clock::now();
    double elapsed_seconds = FpSeconds(t1 - _lastTime).count();

    if (elapsed_seconds >= tickPeriodSeconds)
    {
        _lastTime = t1;

        /* A stall (e.g. the browser tab left in background) must not feed a huge dt
           into the integrator */
        core_coord_t dt_seconds = static_cast<core_coord_t>(elapsed_seconds);
        const core_coord_t dtMax_seconds = 3 * tickPeriodSeconds;
        if (dt_seconds > dtMax_seconds)
        {
            dt_seconds = dtMax_seconds;
        }

        CDS_PROFILE(profile, "Ticking");
        /* Actually tick the system */
        g_core_tick(dt_seconds);

        /* Single-threaded build: publish the profiler aggregates (writer side)
           and pump them into the UI cache (reader side) right after the tick,
           on this same thread */
        cds_profile::registry().publish();
        cds_profile::registry().pump();
    }

    /* Drain the log queue at a fixed point outside the tick's critical work.
       One thread here, so producer and consumer are the same: this is the
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
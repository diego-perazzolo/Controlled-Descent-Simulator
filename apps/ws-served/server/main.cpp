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
// Description : cds_server entry point: wires the libs/ws WebSocket server to
//               the ws_protocol dispatcher (dispatch.cpp). The WASM proxy
//               built from apps/ws-served/client is the client.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <memory>
#include <thread>

#include "core_defs.hpp"
#include "BasePlant.hpp"
#include "loopback/LoopbackPlant.hpp"
#include "sitl/SitlPlant.hpp"
#include "dispatch.hpp"
#include "ws_protocol.hpp"
#include "ws_server.hpp"

using Clock = std::chrono::steady_clock;

static Clock::time_point _lastTime;
static int _is_sys_init = 0;
static std::atomic<bool> _run_rt_thread{true};

extern bool g_core_tick(core_coord_t dt_seconds);                  // global function from core.cpp
extern bool g_core_getTickPeriod(core_coord_t &tickPeriod_second); // global function from core.cpp
extern bool g_core_attachPlant(std::unique_ptr<CDS::BasePlant> plant); // global function from core.cpp

/* Build the plant selected on the command line (default: loopback). The
   composition lives here for now; a future ext command will let the frontend
   pick and configure the plant. Returns nullptr on unknown kind or on a
   parameter error. */
static std::unique_ptr<CDS::BasePlant> _makePlant(const char *kind)
{
    if (std::strcmp(kind, "loopback") == 0)
    {
        auto plant = std::make_unique<plants::LoopbackPlant>();
        if (plant->SetPlantParams((plants::LoopbackPlant::loopbackParams_t){
                .samplePeriod_seconds = 0.05,
                .latency_seconds = 0.05,
                .dropRate = 0.3}))
        {
            return nullptr;
        }
        return plant;
    }

    if (std::strcmp(kind, "sitl") == 0)
    {
        auto plant = std::make_unique<plants::SitlPlant>();
        if (plant->SetPlantParams((plants::SitlPlant::sitlParams_t){
                .host = "0.0.0.0",
                .port = 14550,
                .setpointPeriod_seconds = 0.05,
                .telemetryPeriod_seconds = 0.02,
                .linkTimeout_seconds = 2.0,
                .stabilityVelThreshold_ms = 0.3,
                .stabilityHoldTime_seconds = 3.0}))
        {
            return nullptr;
        }
        return plant;
    }

    // Unknown plant kind
    return nullptr;
}

/* tick the system at rate 1/tickPeriodSeconds, unless ticking system takes too much (> tickPeriodSeconds) */
static void _tick_generator(void)
{
    core_coord_t tickPeriodSeconds;
    if (g_core_getTickPeriod(tickPeriodSeconds) || tickPeriodSeconds <= 0)
    {
        /* No model yet, or tick period not configured: idle without spinning,
           and re-anchor the time base on the next valid pass */
        _is_sys_init = 0;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
    }

    /* Only the first time */
    if (!_is_sys_init)
    {
        _is_sys_init = 1;
        _lastTime = Clock::now();
    }

    /* Elapsed measured as floating-point seconds: an integer duration_cast
       would truncate to zero once an iteration gets faster than its unit,
       silently freezing the simulation */
    using FpSeconds = std::chrono::duration<double>;

    auto t1 = Clock::now();
    double elapsed_seconds = FpSeconds(t1 - _lastTime).count();

    /* Sleeping costs microseconds (syscall + late wakeup): worth it only when
       the remaining wait is much larger than that overhead. Below the margin,
       busy-wait on the clock instead — nanosecond-precise, and cheaper than a
       nap that oversleeps past the whole tick period */
    constexpr double SLEEP_MARGIN_SECONDS = 100e-6;

    const double remaining_seconds = tickPeriodSeconds - elapsed_seconds;
    if (remaining_seconds > SLEEP_MARGIN_SECONDS)
    {
        std::this_thread::sleep_for(FpSeconds(remaining_seconds));
    }
    else
    {
        while (FpSeconds(Clock::now() - _lastTime).count() < tickPeriodSeconds)
        {
            /* spin until the tick deadline */
        }
    }

    t1 = Clock::now();
    elapsed_seconds = FpSeconds(t1 - _lastTime).count();
    _lastTime = t1;

    /* A stall (e.g. a model re-init holding the core lock, scheduler hiccup)
       must not feed a huge dt into the integrator */
    core_coord_t dt_seconds = static_cast<core_coord_t>(elapsed_seconds);
    const core_coord_t dtMax_seconds = 3 * tickPeriodSeconds;
    if (dt_seconds > dtMax_seconds)
    {
        dt_seconds = dtMax_seconds;
    }

    /* Actually tick the system */
    g_core_tick(dt_seconds);
}

int main(int argc, char **argv)
{
    /* line-buffered logs even when stdout is redirected to a file */
    setvbuf(stdout, nullptr, _IOLBF, 0);

    uint16_t port = ws_proto::WS_DEFAULT_PORT;
    if (argc > 1)
    {
        port = (uint16_t)atoi(argv[1]);
    }

    /* optional second arg selects the plant: "loopback" (default) or "sitl" */
    const char *plantKind = (argc > 2) ? argv[2] : "loopback";

    /* Thread "Real-time", used for providing ticks to the System */
    std::thread rt([]
                   {
                    while (_run_rt_thread) 
                    { 
                        _tick_generator();
                    } 
                });

    WsServer server(port, server_dispatch);

    auto plant = _makePlant(plantKind);

    if (!plant || g_core_attachPlant(std::move(plant)))
    {
        // Unknown/misconfigured plant, or attach failed
        std::fprintf(stderr, "cds_server: cannot create plant '%s'\n", plantKind);
        _run_rt_thread = false;
        if (rt.joinable())
        {
            rt.join();
        }
        return 1;
    }

    if (server.Run())
    {
        // Err
        _run_rt_thread = false;
        if (rt.joinable())
        {
            rt.join();
        }
        return 1;
    }

    _run_rt_thread = false;
    if (rt.joinable())
    {
        rt.join();
    }

    return 0;
}

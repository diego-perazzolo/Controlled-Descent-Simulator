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
#include "log.hpp"
#include "LogSinks.hpp"
#include "LogUiSink.hpp"
#include "UniqueFile.hpp"
#include "profile.hpp"
#include "ProfileReport.hpp"
#include "Recorder.hpp"

#include <string>

using Clock = std::chrono::steady_clock;

static Clock::time_point _lastTime;
static int _is_sys_init = 0;
static double _tickAccumulator = 0.0; // pure-sim fixed-step accumulator (sim-seconds owed)
static std::atomic<bool> _run_rt_thread{true};
static std::atomic<bool> _run_drain_thread{true};

static const auto logger = cds_log::registry().module("Main");
static const auto profile = cds_profile::registry().module("Main");

extern bool g_core_tick(core_coord_t dt_seconds);                  // global function from core.cpp
extern bool g_core_getTickPeriod(core_coord_t &tickPeriod_second); // global function from core.cpp
extern bool g_core_getTickRate(core_coord_t &rate);                // global function from core.cpp
extern bool g_core_isPlantAttached(void);                          // global function from core.cpp
extern bool g_core_attachPlant(std::unique_ptr<CDS::BasePlant> plant); // global function from core.cpp

/* Build the plant of the requested kind ("loopback" / "sitl"). Returns nullptr
   on unknown kind or on a parameter error. */
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
        _tickAccumulator = 0.0;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        return;
    }

    /* Only the first time */
    if (!_is_sys_init)
    {
        _is_sys_init = 1;
        _lastTime = Clock::now();
        _tickAccumulator = 0.0;
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

    /* dt policy. With a plant attached the exchange is real-time: one tick fed
       the measured wall-clock elapsed (a stall — model re-init holding the core
       lock, scheduler hiccup — is clamped so it can't feed a huge dt). With NO
       plant it is a pure simulation: fixed-step sub-stepping, rate-scaled — the
       step stays at the nominal period (smooth, deterministic) while the rate
       multiplier sets how much sim-time to cover per wall-second (1.0 = real-
       time, 2.0 = 2x). A per-wake cap avoids the "spiral of death" under load. */
    if (g_core_isPlantAttached())
    {
        core_coord_t dt_seconds = static_cast<core_coord_t>(elapsed_seconds);
        const core_coord_t dtMax_seconds = 3 * tickPeriodSeconds;
        if (dt_seconds > dtMax_seconds) dt_seconds = dtMax_seconds;
        g_core_tick(dt_seconds);
    }
    else
    {
        core_coord_t rate = 1.0;
        g_core_getTickRate(rate);
        if (rate <= 0.0) rate = 1.0;

        _tickAccumulator += elapsed_seconds * rate;
        constexpr double MAX_BACKLOG_SECONDS = 0.25;      // clamp so a stall cannot burst
        if (_tickAccumulator > MAX_BACKLOG_SECONDS) _tickAccumulator = MAX_BACKLOG_SECONDS;

        constexpr double STEP_BUDGET_SECONDS = 0.008;     // wall time to spend sub-stepping per wake
        constexpr int    MAX_STEPS = 2000;                // hard fallback cap
        const auto subStart = Clock::now();
        int steps = 0;
        while (_tickAccumulator >= tickPeriodSeconds && steps < MAX_STEPS)
        {
            g_core_tick(tickPeriodSeconds);
            _tickAccumulator -= tickPeriodSeconds;
            ++steps;
            if (FpSeconds(Clock::now() - subStart).count() >= STEP_BUDGET_SECONDS) break;
        }
    }
    CDS_PROFILE(profile, "Ticking");

    /* Publish the profiler aggregates for readers (frontend / file dump):
       wait-free, done on the writer (this tick) thread */
    cds_profile::registry().publish();
}

/* Drive one data recorder's CSV lifecycle: open a fresh unique file when it
   turns on, flush pending rows, write the "# dropped N rows" trailer and close
   when it turns off (or its owner changes). f/owner are the drain-thread-local
   file state for this recorder slot. */
static void _driveRecorderCsv(cds_record::IRecorder *rec, const char *basePath,
                              std::FILE *&f, cds_record::IRecorder *&owner)
{
    const bool on = rec && rec->enabled();
    if (on && !f)
    {
        f = std::fopen(cds_log::uniqueFilePath(basePath).c_str(), "w");
        if (f) { rec->writeHeader(f); owner = rec; }
    }
    if (f && owner) { owner->drainRows(f); std::fflush(f); }
    if (f && (!on || rec != owner))
    {
        if (owner) owner->writeTrailer(f);
        std::fclose(f); f = nullptr; owner = nullptr;
    }
}

/* Consumer side of the diagnostics: drains queued log records to the sinks,
   streams the profiler raw samples and the per-tick data recorders (one for the
   active model, one for the active plant) to their CSVs, and — when
   CDS_PROFILE_FILE is set — periodically rewrites the profiler aggregate report.
   Runs off the tick path, so blocking I/O here is fine.

   Every output file gets a unique name (uniqueFilePath) so successive runs never
   overwrite each other; the streaming sinks (raw CSV, record CSVs) go further and
   reopen a FRESH file each time serialization is toggled back on. */
static void _drain_generator(const char *profilePath, const char *rawPath,
                             const char *recordPath, const char *recordPlantPath)
{
    auto lastProfileDump = Clock::now();
    // The aggregate report is rewritten in place; give it one unique name per run.
    const std::string reportPath = profilePath ? cds_log::uniqueFilePath(profilePath)
                                               : std::string();

    std::FILE *rawCsv = nullptr;
    std::FILE *recCsv = nullptr;        // model recorder CSV
    std::FILE *plantCsv = nullptr;      // plant recorder CSV
    cds_record::IRecorder *recOwner = nullptr;
    cds_record::IRecorder *plantOwner = nullptr;

    while (_run_drain_thread)
    {
        CDS_PROFILE(profile, "Profiler write to file thread");
        cds_log::registry().drain();

        /* single designated reader of the profiler mailbox: refresh the UI cache
           that the file dump and the ext command both read */
        cds_profile::registry().pump();

        /* raw profiler samples -> CSV for offline analysis. Open a fresh unique
           file when raw logging turns on; flush pending samples into the current
           file; close it when raw logging turns off (a restart opens a new one). */
        const bool rawOn = cds_profile::registry().rawLogging();
        if (rawOn && !rawCsv)
        {
            rawCsv = std::fopen(cds_log::uniqueFilePath(rawPath).c_str(), "w");
            if (rawCsv) cds_profile::writeRawHeader(rawCsv);
        }
        if (rawCsv)
        {
            cds_profile::registry().drainRaw([&](const cds_profile::RawSample &s) {
                cds_profile::writeRawSample(rawCsv, cds_profile::registry(), s);
            });
            std::fflush(rawCsv);
        }
        if (!rawOn && rawCsv) { std::fclose(rawCsv); rawCsv = nullptr; }

        /* per-tick data recorders -> wide black-box CSVs (model + plant), each
           with a metadata header on open and a "# dropped N rows" trailer on
           close, rotating on toggle or owner change. */
        _driveRecorderCsv(cds_record::activeModelRecorder(), recordPath, recCsv, recOwner);
        _driveRecorderCsv(cds_record::activePlantRecorder(), recordPlantPath, plantCsv, plantOwner);

        if (!reportPath.empty())
        {
            const auto now = Clock::now();
            if (now - lastProfileDump > std::chrono::milliseconds(500))
            {
                lastProfileDump = now;
                cds_profile::Snapshot snap;
                if (!cds_profile::registry().snapshot(snap))
                {
                    if (std::FILE *pf = std::fopen(reportPath.c_str(), "w"))
                    {
                        cds_profile::writeReport(pf, cds_profile::registry(), snap);
                        std::fclose(pf);
                    }
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    if (rawCsv) std::fclose(rawCsv);
    if (recCsv)
    {
        if (recOwner) recOwner->writeTrailer(recCsv);
        std::fclose(recCsv);
    }
    if (plantCsv)
    {
        if (plantOwner) plantOwner->writeTrailer(plantCsv);
        std::fclose(plantCsv);
    }
}

int main(int argc, char **argv)
{
    /* line-buffered (flush on new line) logs even when stdout is redirected to a file */
    setvbuf(stdout, nullptr, _IOLBF, 0);

    uint16_t port = ws_proto::WS_DEFAULT_PORT;
    if (argc > 1)
    {
        port = (uint16_t)atoi(argv[1]);
    }

    /* optional second arg selects the plant: "loopback" or "sitl". With NO
       second arg no plant is attached — the server runs a pure simulation
       (deterministic fixed-step tick; see _tick_generator). */
    const char *plantKind = (argc > 2) ? argv[2] : nullptr;

    /* Logger sinks: console always, the recent-lines UI buffer, and the file
       sink. The file sink is toggled from the frontend; its path comes from
       CDS_LOG_FILE (default cds.log) and it starts enabled only if that env was
       set. All are process-wide singletons so the ext commands toggle the very
       instances registered here. */
    cds_log::ConsoleSink consoleSink(stderr);
    cds_log::registry().addSink(&consoleSink);
    cds_log::registry().addSink(&cds_log::uiSink());

    const char *logPath = std::getenv("CDS_LOG_FILE");
    cds_log::fileSink().setPath(logPath ? logPath : "out_data/cds.log");
    if (logPath) cds_log::fileSink().setEnabled(true);
    cds_log::registry().addSink(&cds_log::fileSink());

    const char *profilePath = std::getenv("CDS_PROFILE_FILE");
    const char *rawPath = std::getenv("CDS_PROFILE_RAW_FILE");
    if (rawPath) cds_profile::registry().setRawLogging(true); // env preset
    if (!rawPath) rawPath = "out_data/cds_profile_raw.csv";

    /* Per-tick data recorder CSV base path (unique per run/toggle). Recording is
       toggled from the frontend once a model is running; nothing to preset here
       (no recorder is active until a model registers one). */
    const char *recordPath = std::getenv("CDS_RECORD_FILE");
    if (!recordPath) recordPath = "out_data/cds_record.csv";
    const char *recordPlantPath = std::getenv("CDS_RECORD_PLANT_FILE");
    if (!recordPlantPath) recordPlantPath = "out_data/cds_record_plant.csv";

    /* Thread "Real-time", used for providing ticks to the System */
    std::thread rt([]
                   {
                    while (_run_rt_thread)
                    {
                        _tick_generator();
                    }
                });

    /* Consumer thread: drains logs to the sinks and dumps the profiler report */
    std::thread drain([profilePath, rawPath, recordPath, recordPlantPath] {
        _drain_generator(profilePath, rawPath, recordPath, recordPlantPath);
    });

    auto shutdown = [&](int code) -> int
    {
        _run_rt_thread = false;
        _run_drain_thread = false;
        if (rt.joinable()) rt.join();
        if (drain.joinable()) drain.join();
        cds_log::registry().drain();       // flush any stragglers
        cds_log::registry().clearSinks();  // drop sink pointers before they die
        return code;
    };

    WsServer server(port, server_dispatch);

    /* Attach a plant only if one was requested on the command line; with none,
       run plant-less (a pure, deterministic simulation). */
    if (plantKind != nullptr)
    {
        auto plant = _makePlant(plantKind);
        if (!plant || g_core_attachPlant(std::move(plant)))
        {
            // Unknown/misconfigured plant, or attach failed
            CDS_LOG_ERROR(logger, "Cannot create plant {}", plantKind);
            return shutdown(1);
        }
        CDS_LOG_INFO(logger, "Plant attached: {}", plantKind);
    }
    else
    {
        CDS_LOG_INFO(logger, "No plant requested: running plant-less (pure simulation)");
    }

    if (server.Run())
    {
        CDS_LOG_ERROR(logger, "Cannot rin WebSocket server, shutting down");
        return shutdown(1);
    }

    return shutdown(0);
}

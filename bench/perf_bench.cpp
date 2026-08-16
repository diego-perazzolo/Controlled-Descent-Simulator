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
// File        : perf_bench.cpp
// Description : Micro-benchmark of the diagnostics infrastructure (libs/log,
//               libs/profile, libs/record): the per-call cost of CDS_LOG_* /
//               CDS_PROFILE / CDS_RECORD with the feature ON vs OFF at runtime,
//               the drain (deferred-format) cost, and — via a second build with
//               the compile-out macros set — the residual once the call sites
//               are stripped. All call sites use the public macros so the
//               compiled-out variant truly zeroes them. Not a correctness gate
//               (numbers are timing-dependent); run it to size the overhead:
//                 cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release
//                 cmake --build build-bench
//                 ./build-bench/perf_bench       # features compiled in
//                 ./build-bench/perf_bench_off   # features compiled out
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>

#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"

using Clock = std::chrono::steady_clock;

static inline long long ns(Clock::duration d)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

// A drain sink that only counts, so the logger-drain measurement is the
// deferred-format cost (format thunk + dispatch), not real I/O.
struct CountingSink : cds_log::Sink
{
    std::size_t n = 0;
    volatile std::size_t sink = 0;
    void Write(const cds_log::LogLine& line) override { ++n; sink += line.text.size(); }
};

// Log-enqueue cost with the level PASSING (real deferred serialization). Batched
// so the bounded ring (4096) never fills; the untimed drain empties it between
// batches. When CDS_LOG_COMPILE_LEVEL strips INFO, the call vanishes.
static double benchLogEnqueue(cds_log::moduleId_t mod, std::size_t iters)
{
    const std::size_t B = 1024;
    long long total = 0;
    std::size_t done = 0;
    while (done < iters)
    {
        const std::size_t k = std::min(B, iters - done);
        const auto t0 = Clock::now();
        for (std::size_t i = 0; i < k; ++i) CDS_LOG_INFO(mod, "iter {} val {}", int(i), 3.14159);
        const auto t1 = Clock::now();
        total += ns(t1 - t0);
        cds_log::registry().drain(); // untimed
        done += k;
    }
    return double(total) / double(iters);
}

// Log call REJECTED by the runtime level (TRACE while the module is at Info):
// the cheap early-out path.
static double benchLogFiltered(cds_log::moduleId_t mod, std::size_t iters)
{
    const auto t0 = Clock::now();
    for (std::size_t i = 0; i < iters; ++i) CDS_LOG_TRACE(mod, "iter {}", int(i));
    const auto t1 = Clock::now();
    return double(ns(t1 - t0)) / double(iters);
}

// Drain cost per record: fill (untimed), then time the drain (format + sink).
static double benchLogDrain(cds_log::moduleId_t mod, std::size_t iters)
{
    CountingSink s;
    cds_log::registry().addSink(&s);
    const std::size_t B = 1024;
    long long total = 0;
    std::size_t done = 0;
    while (done < iters)
    {
        const std::size_t k = std::min(B, iters - done);
        for (std::size_t i = 0; i < k; ++i) CDS_LOG_INFO(mod, "iter {} val {}", int(i), 3.14159);
        const auto t0 = Clock::now();
        cds_log::registry().drain();
        const auto t1 = Clock::now();
        total += ns(t1 - t0);
        done += k;
    }
    cds_log::registry().clearSinks();
    return double(total) / double(iters);
}

// Profiler scope cost (2 timestamps + fold, or a scopeActive early-out when the
// module is disabled). CDS_PROFILE_ENABLED=0 strips it entirely.
static double benchProfileScope(cds_profile::moduleId_t mod, std::size_t iters)
{
    (void)mod; // unused when CDS_PROFILE compiles out
    volatile std::size_t sink = 0;
    const auto t0 = Clock::now();
    for (std::size_t i = 0; i < iters; ++i)
    {
        CDS_PROFILE(mod, "bench_scope");
        sink += i;
    }
    const auto t1 = Clock::now();
    (void)sink;
    return double(ns(t1 - t0)) / double(iters);
}

// Recorder row cost: wait-free enqueue (or an enabled() early-out). Batched with
// an untimed drain to /dev/null so the ring never fills. CDS_RECORD_ENABLED=0
// strips it entirely.
template <typename Rec>
static double benchRecord(Rec& rec, std::size_t iters, std::FILE* devnull)
{
    std::array<double, 16> row{};
    const std::size_t B = 1024;
    long long total = 0;
    std::size_t done = 0;
    while (done < iters)
    {
        const std::size_t k = std::min(B, iters - done);
        const auto t0 = Clock::now();
        for (std::size_t i = 0; i < k; ++i) { row[0] = double(i); CDS_RECORD(rec, row); }
        const auto t1 = Clock::now();
        total += ns(t1 - t0);
        rec.drainRows(devnull); // untimed
        done += k;
    }
    return double(total) / double(iters);
}

int main()
{
    std::printf("perf_bench  [CDS_LOG_COMPILE_LEVEL=%d  CDS_PROFILE_ENABLED=%d  CDS_RECORD_ENABLED=%d]\n",
                CDS_LOG_COMPILE_LEVEL, CDS_PROFILE_ENABLED, CDS_RECORD_ENABLED);
    std::printf("%-40s %12s\n", "case", "ns/op");
    std::printf("%-40s %12s\n", "----", "-----");

    const std::size_t N = 2'000'000;

    // ---- logger ----
    const auto lmod = cds_log::registry().module("bench");
    cds_log::registry().setLevel(lmod, cds_log::Level::Info);

    // warm up (touch the ring, the sink path)
    for (int i = 0; i < 2000; ++i) CDS_LOG_INFO(lmod, "warm {}", i);
    cds_log::registry().drain();

    std::printf("%-40s %12.2f\n", "CDS_LOG_INFO enqueue (level passes)", benchLogEnqueue(lmod, N));
    std::printf("%-40s %12.2f\n", "CDS_LOG_TRACE filtered (level rejects)", benchLogFiltered(lmod, N));
    std::printf("%-40s %12.2f\n", "logger drain (format thunk + sink)", benchLogDrain(lmod, N));

    // ---- profiler ----
    const auto pmod = cds_profile::registry().module("bench");
    cds_profile::registry().setEnabled(pmod, true);
    std::printf("%-40s %12.2f\n", "CDS_PROFILE scope (enabled)", benchProfileScope(pmod, N));
    cds_profile::registry().setEnabled(pmod, false);
    std::printf("%-40s %12.2f\n", "CDS_PROFILE scope (disabled)", benchProfileScope(pmod, N));

    // ---- recorder ----
    std::FILE* devnull = std::fopen("/dev/null", "w");
    static cds_record::Recorder<double, 16, 4096> rec("bench", {{
        "t", "c1", "c2", "c3", "c4", "c5", "c6", "c7",
        "c8", "c9", "c10", "c11", "c12", "c13", "c14", "c15",
    }});
    rec.setEnabled(true);
    std::printf("%-40s %12.2f\n", "CDS_RECORD row (enabled)", benchRecord(rec, N, devnull));
    rec.setEnabled(false);
    std::printf("%-40s %12.2f\n", "CDS_RECORD row (disabled)", benchRecord(rec, N, devnull));
    if (devnull) std::fclose(devnull);

    return 0;
}

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
// File        : model_bench.cpp
// Description : Per-model integration benchmark: times one PerformIntegration
//               call (the physics tick — control + RK4, plus the MPC solve for
//               the MPC model) for every runtime model, at a fixed timestep, and
//               reports mean / p50 / p95 / max in microseconds. Diagnostics are
//               left at their defaults (profiler off, recording off), so this is
//               the real per-tick cost the simulation pays. Every model MUST be
//               represented here (AGENTS.md). Not a CI correctness gate — the
//               numbers are timing-dependent. Build and run:
//                 cmake -S bench -B build-bench -DCMAKE_BUILD_TYPE=Release
//                 cmake --build build-bench
//                 ./build-bench/model_bench
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <algorithm>
#include <any>
#include <chrono>
#include <cstdio>
#include <vector>

#include "core_defs.hpp"
#include "TrajectoryManager.hpp"
#include "Rocket.hpp"
#include "QuadRotor.hpp"
#include "QuadRotorMPC.hpp"
#include "RocketMPC.hpp"

using Clock = std::chrono::steady_clock;
static inline long long ns(Clock::duration d)
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(d).count();
}

// A moderate rest-to-target manoeuvre held for a long time, so the timed window
// always stays inside the trajectory. Positions are small and generic — the tick
// cost is dominated by the dynamics/solver, not by the reference.
static bool buildTrajectory(CDS::TrajectoryManager& tm)
{
    core_trajectoryPoly4Params_t poly{ .initialPos = {{0, 0, 0}}, .initialYaw = 0.0,
        .initialVel = {{0, 0, 0}}, .initialYawRate = 0.0,
        .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .finalVel = {{0, 0, 0}},
        .finalYawRate = 0.0, .finalAcc = {{0, 0, 0}}, .finalYawAcc = 0.0, .time_s = 2.0 };
    core_trajectoryPointParams_t hold{ .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .time_s = 120.0 };
    return tm.AppendPoly4(poly) || tm.AppendPoint(hold);
}

// Time N PerformIntegration calls on a freshly-built model; print the stats row.
template <typename Model, typename Params>
static bool benchModel(const char* name, const Params& params)
{
    Model model;
    if (model.SetModelParams(std::any(params))) { std::printf("%-16s  SetModelParams failed\n", name); return true; }

    CDS::TrajectoryManager tm;
    if (buildTrajectory(tm)) { std::printf("%-16s  trajectory build failed\n", name); return true; }
    if (model.SetTrajectoryManager(&tm)) { std::printf("%-16s  SetTrajectoryManager failed\n", name); return true; }

    const core_stepParams_t sp{ .timestep = 0.01, .user_fX = 0.0, .user_fY = 0.0, .user_fZ = 0.0 };

    for (int i = 0; i < 50; ++i) if (model.PerformIntegration(sp)) { std::printf("%-16s  warmup tick failed\n", name); return true; }

    const std::size_t N = 2000;
    std::vector<long long> t;
    t.reserve(N);
    for (std::size_t i = 0; i < N; ++i)
    {
        const auto a = Clock::now();
        const bool err = model.PerformIntegration(sp);
        const auto b = Clock::now();
        if (err) { std::printf("%-16s  tick failed at %zu\n", name, i); return true; }
        t.push_back(ns(b - a));
    }

    std::sort(t.begin(), t.end());
    long long sum = 0;
    for (long long v : t) sum += v;
    const double us = 1.0 / 1000.0;
    const double mean = double(sum) / double(N) * us;
    const double p50  = double(t[std::size_t(0.50 * N)]) * us;
    const double p95  = double(t[std::size_t(0.95 * N)]) * us;
    const double mx   = double(t.back()) * us;
    std::printf("%-16s %10.3f %10.3f %10.3f %10.3f\n", name, mean, p50, p95, mx);
    return false;
}

int main()
{
    std::printf("model integration cost — one PerformIntegration call at dt=0.01 s, N=2000\n");
    std::printf("(diagnostics at defaults: profiler off, recording off)\n\n");
    std::printf("%-16s %10s %10s %10s %10s\n", "model", "mean_us", "p50_us", "p95_us", "max_us");
    std::printf("%-16s %10s %10s %10s %10s\n", "-----", "-------", "------", "------", "------");

    const core_rocketParams_t rocket{ .m = 10.0, .Ix = 10.0 / 3, .Iy = 10.0 / 3, .Iz = 1.0,
        .g = 9.81, .c = 1.0, .cz = 0.02, .F1_max = 500, .F1_min = 0,
        .T1_max = 10, .T1_min = -10, .T2_max = 10, .T2_min = -10, .T3_max = 10, .T3_min = -10 };

    const core_quadRotorParams_t quad{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
        .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5, .kQ = 1.6e-7, .L = 0.275,
        .Irot = 3.0e-5, .Fm_max = 36.0, .Fm_min = 0.0 };

    bool err = false;
    err |= benchModel<CDS::Rocket>("Rocket", rocket);
    err |= benchModel<CDS::QuadRotor>("Quadrotor", quad);
    err |= benchModel<CDS::QuadRotorMPC>("Quadrotor MPC", quad);
    err |= benchModel<CDS::RocketMPC>("Rocket MPC", rocket);

    return err ? 1 : 0;
}

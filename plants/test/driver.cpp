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
// File        : driver.cpp
// Description : Integration test of the plant machinery: LoopbackPlant
//               standalone (echo, sequence, staleness, dropouts), attached
//               to a SystemManager (lifecycle + tick exchange), and the
//               SitlPlant skeleton (params validation + thread lifecycle)
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <thread>

#include "LoopbackPlant.hpp"
#include "SitlPlant.hpp"
#include "SystemManager.hpp"
#include "Rocket.hpp"

using namespace CDS;
using plants::LoopbackPlant;
using plants::SitlPlant;

static int _failures = 0;

#define CHECK(cond)                                              \
    do                                                           \
    {                                                            \
        if (!(cond))                                             \
        {                                                        \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
            _failures++;                                         \
        }                                                        \
    } while (0)

/* poll cond every 5 ms until true or timeout; returns true when satisfied */
static bool waitFor(const std::function<bool(void)>& cond, double timeout_s)
{
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout_s);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cond())
        {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return cond();
}

/* --- phase 1: LoopbackPlant standalone --------------------------------- */
static void testStandalone(void)
{
    LoopbackPlant plant;

    /* invalid params must be rejected */
    CHECK(plant.SetPlantParams(std::any(42)) == true);
    CHECK(plant.SetPlantParams(LoopbackPlant::loopbackParams_t{-1, 0, 0}) == true);
    CHECK(plant.SetPlantParams(LoopbackPlant::loopbackParams_t{0.005, 0.02, 0.0}) == false);

    /* nothing published while disconnected; mission on a disconnected link
       must fail */
    BasePlant::plantMeasurements_t meas = {};
    CHECK(plant.PullMeasurements(meas) == true);
    CHECK(plant.Start() == true);

    /* connect: idle telemetry of the held state (origin, no rates) must
       flow while the mission is still stopped */
    CHECK(plant.Connect() == false);
    CHECK(plant.Connect() == true);   // double connect is an error

    CHECK(waitFor([&] { return plant.PullMeasurements(meas) == false; }, 2.0));
    CHECK(std::abs(meas.state.x) < 1e-9);
    CHECK(std::abs(meas.state.x_dot) < 1e-9);

    /* push a command, start the mission, wait for the echo (latency 20 ms) */
    BasePlant::plantCommands_t cmd = {};
    cmd.time_seconds = 1.5;
    cmd.reference.pos = {10, 20, 30};
    cmd.reference.vel = {1, 2, 3};
    cmd.reference.yaw = 0.5;
    CHECK(plant.PushCommands(cmd) == false);

    CHECK(plant.Start() == false);
    CHECK(plant.Start() == true);   // double start is an error

    CHECK(waitFor([&] {
        return plant.PullMeasurements(meas) == false &&
               std::abs(meas.state.x - 10) < 1e-9;
    }, 2.0));

    /* echo correctness: measured state == commanded reference */
    CHECK(std::abs(meas.state.y - 20) < 1e-9);
    CHECK(std::abs(meas.state.z - 30) < 1e-9);
    CHECK(std::abs(meas.state.x_dot - 1) < 1e-9);
    CHECK(std::abs(meas.state.yaw - 0.5) < 1e-9);
    CHECK(meas.sequence >= 1);

    /* sequence must be monotone while running */
    const uint32_t seqEarly = meas.sequence;
    CHECK(waitFor([&] {
        BasePlant::plantMeasurements_t m2 = {};
        return plant.PullMeasurements(m2) == false && m2.sequence > seqEarly;
    }, 2.0));

    /* mission stop: the vehicle holds the last echoed position and the
       telemetry keeps flowing on the live link, with no residual rates */
    CHECK(plant.Stop() == false);
    CHECK(plant.Stop() == false);   // idempotent
    BasePlant::plantMeasurements_t a = {}, b = {};
    CHECK(plant.PullMeasurements(a) == false);
    CHECK(waitFor([&] {
        return plant.PullMeasurements(b) == false && b.sequence > a.sequence &&
               std::abs(b.state.x - 10) < 1e-9 &&
               std::abs(b.state.x_dot) < 1e-9;
    }, 2.0));

    /* staleness: once disconnected, re-reads return the same sequence */
    CHECK(plant.Disconnect() == false);
    CHECK(plant.PullMeasurements(a) == false);
    CHECK(plant.PullMeasurements(b) == false);
    CHECK(a.sequence == b.sequence);

    CHECK(plant.Disconnect() == false);   // idempotent

    std::printf("standalone OK (last seq=%u, plantTime=%.3f)\n",
                a.sequence, a.plantTime_seconds);
}

/* --- phase 2: attached to a SystemManager ------------------------------ */
static void testWithSystemManager(void)
{
    SystemManager sm;

    /* model: reference rocket */
    auto rocket = std::make_unique<Rocket>();
    core_rocketParams_t rPar = {};
    rPar.m = 10.0; rPar.Ix = 10.0 / 3; rPar.Iy = 10.0 / 3; rPar.Iz = 1.0;
    rPar.g = 9.81; rPar.c = 1.0; rPar.cz = 0.02;
    rPar.F1_max = 500; rPar.F1_min = 0;
    rPar.T1_max = 10; rPar.T1_min = -10;
    rPar.T2_max = 10; rPar.T2_min = -10;
    rPar.T3_max = 10; rPar.T3_min = -10;
    CHECK(rocket->SetModelParams(rPar) == false);
    CHECK(sm.InitModel(std::move(rocket)) == false);

    /* trajectory: reference descent */
    CHECK(sm.InitTrajectory() == false);
    core_trajectoryPoly4Params_t poly = {};
    poly.initialPos = {-50, 50, 80};
    poly.initialVel = {0, 5, -50};
    poly.finalPos = {0, 0, 0};
    poly.time_s = 20;
    CHECK(sm.MutateTrajectoryManager([&poly](TrajectoryManager& tM)
                                     { return tM.AppendPoly4(poly); }) == false);

    /* plant: slower than the tick, small latency, some dropouts */
    auto plant = std::make_unique<LoopbackPlant>();
    CHECK(plant->SetPlantParams(
              LoopbackPlant::loopbackParams_t{0.02, 0.05, 0.1}) == false);
    CHECK(sm.AttachPlant(std::move(plant)) == false);

    CHECK(sm.SetParameters({0.01}) == false);

    /* run must be refused with no... model is there: run OK. Detach while
       running must be refused */
    CHECK(sm.Run() == false);
    CHECK(sm.DetachPlant() == true);

    /* drive ~0.5 s of ticks at real pace: exchange must never error the tick */
    for (int i = 0; i < 50; i++)
    {
        CHECK(sm.ExecuteTick(0.01) == false);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    CHECK(sm.Stop() == false);
    CHECK(sm.DetachPlant() == false);
    CHECK(sm.DetachPlant() == true);   // nothing left to detach

    std::printf("system manager OK\n");
}

/* --- phase 3: SitlPlant skeleton --------------------------------------- */
static void testSitlSkeleton(void)
{
    SitlPlant plant;

    /* valid parameter set, mutated per-case below */
    const SitlPlant::sitlParams_t good = {.host = "127.0.0.1",
                                          .port = 14550,
                                          .setpointPeriod_seconds = 0.05,
                                          .telemetryPeriod_seconds = 0.02,
                                          .linkTimeout_seconds = 2.0};

    /* invalid params must be rejected */
    CHECK(plant.SetPlantParams(std::any(42)) == true);

    SitlPlant::sitlParams_t bad = good;
    bad.host = "";
    CHECK(plant.SetPlantParams(bad) == true);

    bad = good;
    bad.port = 0;
    CHECK(plant.SetPlantParams(bad) == true);

    bad = good;
    bad.setpointPeriod_seconds = -0.05;
    CHECK(plant.SetPlantParams(bad) == true);

    CHECK(plant.SetPlantParams(good) == false);

    /* lifecycle: mission requires a connected link, double connect/start
       are errors, no reconfigure while connected, stop and disconnect are
       idempotent */
    CHECK(plant.Start() == true);
    CHECK(plant.Connect() == false);
    CHECK(plant.Connect() == true);
    CHECK(plant.SetPlantParams(good) == true);
    CHECK(plant.Start() == false);
    CHECK(plant.Start() == true);
    CHECK(plant.Stop() == false);
    CHECK(plant.Stop() == false);
    CHECK(plant.Disconnect() == false);
    CHECK(plant.Disconnect() == false);

    /* the skeleton publishes nothing */
    BasePlant::plantMeasurements_t meas = {};
    CHECK(plant.PullMeasurements(meas) == true);

    std::printf("sitl skeleton OK\n");
}

int main(void)
{
    testStandalone();
    testWithSystemManager();
    testSitlSkeleton();

    if (_failures)
    {
        std::printf("DRIVER FAILED (%d checks)\n", _failures);
        return 1;
    }

    std::printf("DRIVER ALL OK\n");
    return 0;
}

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
// File        : offset_free_model_test.cpp
// Description : Acid test of the QuadRotorMPC offset-free disturbance observer.
//               A constant lateral force is applied while the vehicle holds a
//               fixed setpoint. Without the observer the MPC (which predicts a
//               zero external force) settles with a persistent steady-state
//               offset; with the observer on, d_hat integrates the residual and
//               the MPC predicts predForce = d_hat, driving the steady-state
//               error to ~zero. The test asserts the observer both removes the
//               offset and clearly beats the observer-off baseline. A second
//               run checks graceful degradation: with a position sensor axis
//               dropped at runtime the estimate coasts and stays finite.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "QuadRotorMPC.hpp"
#include "RocketMPC.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

namespace
{
    // Find a manifest row id by its group + label (the TSV is
    // "id\tgroup\tlabel\tflags\tvalue\n"). Returns -1 if not present. Used to
    // drive the ext-facing path: enabling the observer through SetControllerParam
    // rather than the direct C++ setter.
    int manifestId(const char* tsv, const char* group, const char* label)
    {
        const char* line = tsv;
        while (line && *line)
        {
            const char* nl = std::strchr(line, '\n');
            const char* end = nl ? nl : line + std::strlen(line);
            // fields separated by '\t': id, group, label, flags, value
            const char* t1 = std::strchr(line, '\t');                       // after id
            if (!t1 || t1 >= end) { line = nl ? nl + 1 : nullptr; continue; }
            const char* t2 = std::strchr(t1 + 1, '\t');                     // after group
            if (!t2 || t2 >= end) { line = nl ? nl + 1 : nullptr; continue; }
            const char* t3 = std::strchr(t2 + 1, '\t');                     // after label
            if (!t3 || t3 >= end) { line = nl ? nl + 1 : nullptr; continue; }
            const std::size_t glen = std::size_t(t2 - (t1 + 1));
            const std::size_t llen = std::size_t(t3 - (t2 + 1));
            if (glen == std::strlen(group) && std::strncmp(t1 + 1, group, glen) == 0 &&
                llen == std::strlen(label) && std::strncmp(t2 + 1, label, llen) == 0)
                return std::atoi(line);
            line = nl ? nl + 1 : nullptr;
        }
        return -1;
    }
    // airframe parameters (same as the notebook / mpc_model_test)
    core_quadRotorParams_t airframe()
    {
        return core_quadRotorParams_t{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                                       .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                                       .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                                       .Fm_max = 36.0, .Fm_min = 0.0 };
    }

    // Hold a fixed setpoint under a constant lateral force for hold_s seconds;
    // return the mean position error over the final second (the steady state).
    // dropAxisX drops the x position sensor mid-run to exercise predict-only.
    double steadyStateError(bool observerOn, double fx, bool dropAxisX, bool& finite_ok)
    {
        QuadRotorMPC model;
        auto p = airframe();
        model.SetModelParams(std::any(p));

        TrajectoryManager tm;
        core_trajectoryPointParams_t hold{ .finalPos = {{1.0, 0.5, 1.0}}, .finalYaw = 0.0, .time_s = 12.0 };
        tm.AppendPoint(hold);
        model.SetTrajectoryManager(&tm);
        model.SetObserverEnabled(observerOn);

        const double dt  = 0.01;
        const int    SIM = 1200;           // 12 s
        const int    TAIL = 100;           // last 1 s averaged
        double tailSum = 0.0;
        finite_ok = true;
        core_state_t st{};
        for (int t = 0; t < SIM; ++t)
        {
            if (dropAxisX && t == 600) model.PositionSensor().SetEnabled(0, false); // drop x at 6 s
            core_stepParams_t sp{ .timestep = dt, .user_fX = fx, .user_fY = 0.0, .user_fZ = 0.0 };
            if (model.PerformIntegration(sp)) { finite_ok = false; return 1e9; }
            model.GetState(st);
            if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(st.z)) finite_ok = false;
            if (t >= SIM - TAIL)
            {
                const double e = std::sqrt((st.x-1.0)*(st.x-1.0) + (st.y-0.5)*(st.y-0.5) + (st.z-1.0)*(st.z-1.0));
                tailSum += e;
            }
        }
        return tailSum / TAIL;
    }

    core_rocketParams_t rocketframe()
    {
        return core_rocketParams_t{ .m = 10.0, .Ix = 10.0/3, .Iy = 10.0/3, .Iz = 1.0,
                                    .g = 9.81, .c = 1.0, .cz = 0.02,
                                    .F1_max = 500, .F1_min = 0,
                                    .T1_max = 10, .T1_min = -10, .T2_max = 10, .T2_min = -10,
                                    .T3_max = 10, .T3_min = -10 };
    }

    // Same experiment for the rocket MPC (Euler-angle sibling): hold a setpoint
    // under a constant lateral force, mean position error over the final second.
    double rocketSteadyStateError(bool observerOn, double fx, bool dropAxisX, bool& finite_ok)
    {
        RocketMPC model;
        auto p = rocketframe();
        model.SetModelParams(std::any(p));

        TrajectoryManager tm;
        core_trajectoryPointParams_t hold{ .finalPos = {{0.6, 0.4, 0.9}}, .finalYaw = 0.0, .time_s = 12.0 };
        tm.AppendPoint(hold);
        model.SetTrajectoryManager(&tm);
        model.SetObserverEnabled(observerOn);

        const double dt  = 0.01;
        const int    SIM = 1200;           // 12 s
        const int    TAIL = 100;           // last 1 s averaged
        double tailSum = 0.0;
        finite_ok = true;
        core_state_t st{};
        for (int t = 0; t < SIM; ++t)
        {
            if (dropAxisX && t == 600) model.PositionSensor().SetEnabled(0, false);
            core_stepParams_t sp{ .timestep = dt, .user_fX = fx, .user_fY = 0.0, .user_fZ = 0.0 };
            if (model.PerformIntegration(sp)) { finite_ok = false; return 1e9; }
            model.GetState(st);
            if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(st.z)) finite_ok = false;
            if (t >= SIM - TAIL)
            {
                const double e = std::sqrt((st.x-0.6)*(st.x-0.6) + (st.y-0.4)*(st.y-0.4) + (st.z-0.9)*(st.z-0.9));
                tailSum += e;
            }
        }
        return tailSum / TAIL;
    }
}

// Offset-free criterion. The observer feeds d_hat into the MPC *prediction*
// (predForce), which removes the tracking error caused by the controller not
// knowing the external force -- a large, clear reduction over the baseline, and
// the estimate must coast when a sensor drops. It does NOT recompute the MPC
// *target*, so a residual set by the controller's own cost trade-off remains
// (holding position under a lateral force needs a steady tilt, which the upright
// attitude reference penalises: tiny for the soft-attitude quad, larger for the
// stiff-attitude rocket). So the test asserts the genuine property -- a strong
// relative improvement plus a coasting drop -- not an absolute near-zero that
// would depend on the controller tuning. Full target-recalculation offset-free
// is a documented later refinement.
static bool offsetFreePass(const char* tag, double errOff, double errOn, double errDrop,
                           bool finOff, bool finOn, bool finDrop)
{
    std::printf("[%s] steady-state error under a constant lateral force:\n", tag);
    std::printf("  observer OFF : %.4f m   ON : %.4f m   ON+drop : %.4f m\n", errOff, errOn, errDrop);
    const bool pass = finOff && finOn && finDrop
                    && errOff > 0.08                     // there really was an offset to fix
                    && errOn  < 0.25 * errOff            // observer clearly removes most of it
                    && errOn  < 0.10                     // and settles small in absolute terms
                    && std::isfinite(errDrop)
                    && errDrop < 1.5 * errOn + 0.02;     // dropped-sensor coast stays comparable
    std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
    return pass;
}

int main()
{
    bool ok = true;

    {
        const double fx = 6.0;    // constant lateral force [N]
        bool a = true, b = true, c = true;
        const double eOff  = steadyStateError(false, fx, false, a);
        const double eOn   = steadyStateError(true,  fx, false, b);
        const double eDrop = steadyStateError(true,  fx, true,  c);
        ok = offsetFreePass("QuadRotorMPC", eOff, eOn, eDrop, a, b, c) && ok;
    }
    {
        const double fx = 15.0;   // constant lateral force [N]
        bool a = true, b = true, c = true;
        const double eOff  = rocketSteadyStateError(false, fx, false, a);
        const double eOn   = rocketSteadyStateError(true,  fx, false, b);
        const double eDrop = rocketSteadyStateError(true,  fx, true,  c);
        ok = offsetFreePass("RocketMPC", eOff, eOn, eDrop, a, b, c) && ok;
    }

    // ---- ext-facing path: enable the observer THROUGH the manifest ----------
    // Proves the increment-5 wiring: the observer/sensor knobs ride the existing
    // controller-manifest ext channel, so toggling "Observer/enable" by id has
    // the same offset-free effect as the direct setter.
    {
        QuadRotorMPC model;
        core_quadRotorParams_t p{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                                  .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                                  .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                                  .Fm_max = 36.0, .Fm_min = 0.0 };
        model.SetModelParams(std::any(p));
        TrajectoryManager tm;
        core_trajectoryPointParams_t hold{ .finalPos = {{1.0, 0.5, 1.0}}, .finalYaw = 0.0, .time_s = 12.0 };
        tm.AppendPoint(hold);
        model.SetTrajectoryManager(&tm);

        char buf[2048] = {0};
        model.GetControllerManifest(buf, sizeof buf);
        const int id = manifestId(buf, "Observer", "enable (0|1)");
        bool viaManifest = (id >= 0) && !model.SetControllerParam(id, 1.0) && model.IsObserverEnabled();

        double tailSum = 0.0; core_state_t st{};
        for (int t = 0; t < 1200; ++t)
        {
            core_stepParams_t sp{ .timestep = 0.01, .user_fX = 6.0, .user_fY = 0.0, .user_fZ = 0.0 };
            model.PerformIntegration(sp);
            model.GetState(st);
            if (t >= 1100) tailSum += std::sqrt((st.x-1.0)*(st.x-1.0)+(st.y-0.5)*(st.y-0.5)+(st.z-1.0)*(st.z-1.0));
        }
        const double err = tailSum / 100.0;
        const bool pass = viaManifest && err < 0.03;
        std::printf("[manifest path] Observer/enable id=%d -> enabled=%d, offset-free err=%.4f m -> %s\n",
                    id, model.IsObserverEnabled(), err, pass ? "PASS" : "FAIL");
        ok = pass && ok;
    }

    // ---- sensor injection WITHOUT the observer -----------------------------
    // With the estimator off, the sensor now feeds the controller directly: a
    // position bias fools the MPC (no integrators), which drives the vehicle to
    // an offset of ~the bias. Proves sensors bite independently of the observer.
    {
        QuadRotorMPC model;
        core_quadRotorParams_t p{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                                  .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                                  .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                                  .Fm_max = 36.0, .Fm_min = 0.0 };
        model.SetModelParams(std::any(p));
        TrajectoryManager tm;
        core_trajectoryPointParams_t hold{ .finalPos = {{1.0, 0.5, 1.0}}, .finalYaw = 0.0, .time_s = 12.0 };
        tm.AppendPoint(hold);
        model.SetTrajectoryManager(&tm);
        model.SetObserverEnabled(false);          // estimator OFF
        model.PositionSensor().SetBias(0, 0.5);   // +0.5 m bias on the x sensor

        double sumX = 0.0; core_state_t st{};
        for (int t = 0; t < 1200; ++t)
        {
            core_stepParams_t sp{ .timestep = 0.01, .user_fX = 0.0, .user_fY = 0.0, .user_fZ = 0.0 };
            model.PerformIntegration(sp);
            model.GetState(st);
            if (t >= 1100) sumX += (1.0 - st.x);   // signed x tracking error
        }
        const double eX = sumX / 100.0;
        // fooled by +0.5 m bias, the true x settles ~0.5 m short of the target.
        const bool pass = std::fabs(eX - 0.5) < 0.1;
        std::printf("[sensor-no-observer] +0.5 m x-bias, observer OFF -> x offset %.4f m (expect ~0.5) -> %s\n",
                    eX, pass ? "PASS" : "FAIL");
        ok = pass && ok;
    }

    // ---- observer mistuning vs matched Rv ----------------------------------
    // Heavy position noise (std 1 m), observer ON. With the measurement-noise
    // covariance rPos left at its small default the filter over-trusts the sensor
    // (chases the noise, its disturbance state injects spurious forces); matching
    // rPos to the noise makes it filter properly and track much tighter. Proves
    // both the phenomenon and that the exposed covariance fixes it (set by id
    // through the manifest, so the ext path is exercised too).
    {
        auto noisyObserverErr = [](double rPos) {
            QuadRotorMPC model;
            core_quadRotorParams_t p{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                                      .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                                      .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                                      .Fm_max = 36.0, .Fm_min = 0.0 };
            model.SetModelParams(std::any(p));
            TrajectoryManager tm;
            core_trajectoryPointParams_t hold{ .finalPos = {{1.0, 0.5, 1.0}}, .finalYaw = 0.0, .time_s = 12.0 };
            tm.AppendPoint(hold);
            model.SetTrajectoryManager(&tm);
            model.SetObserverEnabled(true);
            for (std::size_t a = 0; a < 3; ++a) model.PositionSensor().SetNoiseStd(a, 1.0);
            char buf[2048] = {0}; model.GetControllerManifest(buf, sizeof buf);
            const int id = manifestId(buf, "Observer", "measurement noise");
            if (id >= 0) model.SetControllerParam(id, rPos);
            double s = 0.0; core_state_t st{};
            for (int t = 0; t < 1200; ++t) {
                core_stepParams_t sp{ .timestep = 0.01, .user_fX = 0.0, .user_fY = 0.0, .user_fZ = 0.0 };
                model.PerformIntegration(sp); model.GetState(st);
                if (t >= 1100) s += std::sqrt((st.x-1)*(st.x-1)+(st.y-0.5)*(st.y-0.5)+(st.z-1)*(st.z-1));
            }
            return s / 100.0;
        };
        const double eMistuned = noisyObserverErr(1.0e-4);   // default: too confident
        const double eMatched  = noisyObserverErr(1.0);      // rPos ~ noise variance
        const bool pass = std::isfinite(eMistuned) && std::isfinite(eMatched) && eMatched < eMistuned;
        std::printf("[observer tuning] std=1 noise -> err mistuned(rPos=1e-4)=%.4f m, matched(rPos=1)=%.4f m -> %s\n",
                    eMistuned, eMatched, pass ? "PASS" : "FAIL");
        ok = pass && ok;
    }

    std::printf("%s\n", ok ? "OFFSET-FREE MODEL TEST PASSED" : "OFFSET-FREE MODEL TEST FAILED");
    return ok ? 0 : 1;
}

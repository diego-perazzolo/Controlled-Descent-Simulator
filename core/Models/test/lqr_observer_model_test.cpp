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
// File        : lqr_observer_model_test.cpp
// Description : Acid test of the partial state-estimator integration on the
//               FF-LQR models (Rocket, QuadRotor). Those controllers already
//               integrate, so offset-free is theirs; here the observer's job is
//               sensor robustness: feeding the LQR feedback a filtered position/
//               velocity so a NOISY or DROPPED position sensor does not wreck
//               tracking. The test checks that (a) enabling the estimator with a
//               clean sensor barely changes the baseline, (b) under a noisy
//               sensor the estimator keeps tracking tight (it filters, rather
//               than piping the noise into the feedback), and (c) with a sensor
//               axis dropped mid-run the estimate coasts and tracking stays
//               finite and bounded.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <cmath>
#include <cstdio>

#include "Rocket.hpp"
#include "QuadRotor.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

namespace
{
    enum class Sensor { Clean, Noisy, Drop };

    // Drive a model holding a fixed setpoint; return the mean position error over
    // the final second. `observerOn` toggles the estimator; `mode` sets the sensor
    // condition (clean / noisy / dropped-x-at-6s). Templated over the model type
    // (Rocket, QuadRotor) since both share the interface.
    template <typename Model, typename Params>
    double meanFinalError(const Params& p, const std::array<double,3>& target,
                          bool observerOn, Sensor mode, bool& finite_ok)
    {
        Model model;
        model.SetModelParams(std::any(p));

        TrajectoryManager tm;
        core_trajectoryPointParams_t hold{ .finalPos = {{target[0], target[1], target[2]}},
                                           .finalYaw = 0.0, .time_s = 12.0 };
        tm.AppendPoint(hold);
        model.SetTrajectoryManager(&tm);

        model.SetObserverEnabled(observerOn);
        if (observerOn && mode == Sensor::Noisy)
            for (std::size_t a = 0; a < 3; ++a)
                model.PositionSensor().SetChannel(a, {true, 0.0, 0.03});   // 3 cm std noise

        const double dt  = 0.01;
        const int    SIM = 1200;   // 12 s
        const int    TAIL = 100;   // last 1 s averaged
        double tailSum = 0.0;
        finite_ok = true;
        core_state_t st{};
        for (int t = 0; t < SIM; ++t)
        {
            if (observerOn && mode == Sensor::Drop && t == 600)
                model.PositionSensor().SetEnabled(0, false);   // drop x sensor at 6 s
            core_stepParams_t sp{ .timestep = dt, .user_fX = 0.0, .user_fY = 0.0, .user_fZ = 0.0 };
            if (model.PerformIntegration(sp)) { finite_ok = false; return 1e9; }
            model.GetState(st);
            if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(st.z)) finite_ok = false;
            if (t >= SIM - TAIL)
            {
                const double e = std::sqrt((st.x-target[0])*(st.x-target[0])
                                         + (st.y-target[1])*(st.y-target[1])
                                         + (st.z-target[2])*(st.z-target[2]));
                tailSum += e;
            }
        }
        return tailSum / TAIL;
    }

    template <typename Model, typename Params>
    bool runModel(const char* tag, const Params& p, const std::array<double,3>& target)
    {
        bool f0 = true, f1 = true, f2 = true, f3 = true;
        const double eOff   = meanFinalError<Model>(p, target, false, Sensor::Clean, f0);
        const double eClean = meanFinalError<Model>(p, target, true,  Sensor::Clean, f1);
        const double eNoisy = meanFinalError<Model>(p, target, true,  Sensor::Noisy, f2);
        const double eDrop  = meanFinalError<Model>(p, target, true,  Sensor::Drop,  f3);

        std::printf("[%s] mean final tracking error:\n", tag);
        std::printf("  OFF %.4f m | ON clean %.4f m | ON noisy(3cm) %.4f m | ON drop-x %.4f m\n",
                    eOff, eClean, eNoisy, eDrop);

        const bool pass = f0 && f1 && f2 && f3
                        && eOff   < 0.02                       // LQR holds the point on true state
                        && std::fabs(eClean - eOff) < 0.02     // estimator does not disturb the baseline
                        && eNoisy < 0.05                       // filtered: 3 cm sensor noise stays out of tracking
                        && eDrop  < 0.05;                      // dropped sensor coasts, tracking holds
        std::printf("  -> %s\n", pass ? "PASS" : "FAIL");
        return pass;
    }

    core_quadRotorParams_t quadframe()
    {
        return core_quadRotorParams_t{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                                       .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                                       .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                                       .Fm_max = 36.0, .Fm_min = 0.0 };
    }
    core_rocketParams_t rocketframe()
    {
        return core_rocketParams_t{ .m = 10.0, .Ix = 10.0/3, .Iy = 10.0/3, .Iz = 1.0,
                                    .g = 9.81, .c = 1.0, .cz = 0.02,
                                    .F1_max = 500, .F1_min = 0,
                                    .T1_max = 10, .T1_min = -10, .T2_max = 10, .T2_min = -10,
                                    .T3_max = 10, .T3_min = -10 };
    }
}

int main()
{
    bool ok = true;
    ok = runModel<QuadRotor>("QuadRotor FF-LQR", quadframe(),  {{1.0, 0.5, 1.0}}) && ok;
    ok = runModel<Rocket>   ("Rocket FF-LQR",    rocketframe(), {{0.6, 0.4, 0.9}}) && ok;

    std::printf("%s\n", ok ? "LQR OBSERVER MODEL TEST PASSED" : "LQR OBSERVER MODEL TEST FAILED");
    return ok ? 0 : 1;
}

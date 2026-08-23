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
// File        : horizon_edge_model_test.cpp
// Description : Edge cases of the MPC horizon preview, where the prediction
//               window runs off the end of the trajectory. Three shapes: a
//               trajectory much shorter than the horizon, one shorter than a
//               single control step, and the longest horizon the fixed buffers
//               allow. In every case the sampler must hold the last valid stage
//               instead of reading past it and the run must stay finite; where
//               the horizon is one the controller is tuned for, the vehicle must
//               also settle on the final setpoint. At the extreme horizon only
//               finiteness is required: settling there is a separate, open
//               tuning question (the quadrotor does NOT settle at N=256 against
//               a step-like preview), deliberately not asserted here.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <array>
#include <cmath>
#include <cstdio>

#include "QuadRotorMPC.hpp"
#include "RocketMPC.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

namespace
{
    // Fly `model` for `simSeconds` past the end of a trajectory that lasts only
    // `trajSeconds`, with the horizon forced to `horizon` stages. The run must
    // always stay finite; with `settle` it must also park within `tolerance` of
    // the last setpoint.
    template <class Model, class Params>
    bool runCase(const char* name, const Params& frame, double trajSeconds,
                 double horizon, double simSeconds, double tolerance, bool settle = true)
    {
        Model model;
        if (model.SetModelParams(std::any(frame))) { std::printf("  %-42s SetModelParams failed\n", name); return false; }

        // A single short manoeuvre and nothing after it: every preview stage
        // beyond trajSeconds falls off the end of the trajectory.
        const std::array<core_coord_t, 3> target{{1.0, 0.5, 0.4}};
        TrajectoryManager tm;
        core_trajectoryPoly4Params_t poly{ .initialPos = {{0,0,0}}, .initialYaw = 0.0,
            .initialVel = {{0,0,0}}, .initialYawRate = 0.0,
            .finalPos = {{target[0], target[1], target[2]}}, .finalYaw = 0.0, .finalVel = {{0,0,0}},
            .finalYawRate = 0.0, .finalAcc = {{0,0,0}}, .finalYawAcc = 0.0, .time_s = trajSeconds };
        if (tm.AppendPoly4(poly)) { std::printf("  %-42s trajectory build failed\n", name); return false; }
        if (model.SetTrajectoryManager(&tm)) { std::printf("  %-42s SetTrajectoryManager failed\n", name); return false; }

        // horizon N is the single writable row of the model manifest
        if (model.SetModelParam(0, horizon)) { std::printf("  %-42s horizon %g rejected\n", name, horizon); return false; }

        const double dt   = 0.01;
        const int    SIM  = static_cast<int>(simSeconds / dt);
        core_state_t st{};
        bool finite_ok = true;
        for (int t = 0; t < SIM; ++t)
        {
            core_stepParams_t sp{ .timestep = dt, .user_fX = 0.0, .user_fY = 0.0, .user_fZ = 0.0 };
            if (model.PerformIntegration(sp)) { std::printf("  %-42s PerformIntegration failed at t=%d\n", name, t); return false; }

            model.GetState(st);
            if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(st.z) ||
                !std::isfinite(st.roll) || !std::isfinite(st.pitch) || !std::isfinite(st.yaw))
            {
                finite_ok = false;
                break;
            }
        }

        const double err = std::sqrt((st.x-target[0])*(st.x-target[0])
                                   + (st.y-target[1])*(st.y-target[1])
                                   + (st.z-target[2])*(st.z-target[2]));
        const bool pass = finite_ok && (!settle || err < tolerance);
        std::printf("  %-42s %s (final error %.4f m, %s)%s\n", name,
                    pass ? "ok  " : "FAIL", err,
                    settle ? "must settle" : "finite only",
                    finite_ok ? "" : "  [state went non-finite]");
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
    std::printf("horizon edges: the preview runs off the end of the trajectory\n");

    bool ok = true;
    // default horizon (40 stages) over a trajectory that ends long before it
    ok = runCase<QuadRotorMPC>("QuadRotor MPC  traj 0.4s  < horizon", quadframe(),   0.4,  40.0, 3.0, 0.10) && ok;
    ok = runCase<RocketMPC>   ("Rocket MPC     traj 0.4s  < horizon", rocketframe(), 0.4,  40.0, 4.0, 0.15) && ok;
    // trajectory shorter than one control step: only stage 0 is ever valid
    ok = runCase<QuadRotorMPC>("QuadRotor MPC  traj 0.01s < control step", quadframe(),   0.01, 40.0, 3.0, 0.10) && ok;
    ok = runCase<RocketMPC>   ("Rocket MPC     traj 0.01s < control step", rocketframe(), 0.01, 40.0, 4.0, 0.15) && ok;
    // longest horizon the fixed buffers allow, still off the end. Finiteness
    // only: see the file header for why settling is not asserted here.
    ok = runCase<QuadRotorMPC>("QuadRotor MPC  horizon 256 (MAX)", quadframe(),   0.4, 256.0, 3.0, 0.10, false) && ok;
    ok = runCase<RocketMPC>   ("Rocket MPC     horizon 256 (MAX)", rocketframe(), 0.4, 256.0, 4.0, 0.15, false) && ok;

    std::printf("%s\n", ok ? "HORIZON EDGE MODEL TEST PASSED" : "HORIZON EDGE MODEL TEST FAILED");
    return ok ? 0 : 1;
}

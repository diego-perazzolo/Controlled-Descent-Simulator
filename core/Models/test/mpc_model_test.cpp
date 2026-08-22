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
// File        : mpc_model_test.cpp
// Description : Native acid test of the QuadRotorMPC runtime model driven through
//               its public BaseModel interface: it tracks a Poly4 rest-to-rest
//               manoeuvre (exercising the horizon preview), rejects a mid-flight
//               force gust, and must arrive near the target with a settled
//               attitude while staying finite throughout.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <cmath>
#include <cstdio>

#include "QuadRotorMPC.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

int main()
{
    QuadRotorMPC model;

    // airframe parameters (same as the notebook)
    core_quadRotorParams_t p{ .m = 2.4, .Ix = 0.025, .Iy = 0.025, .Iz = 0.045,
                              .g = 9.81, .c = 0.20, .cz = 0.30, .kT = 1.0e-5,
                              .kQ = 1.6e-7, .L = 0.275, .Irot = 3.0e-5,
                              .Fm_max = 36.0, .Fm_min = 0.0 };
    if (model.SetModelParams(std::any(p))) { printf("SetModelParams failed\n"); return 1; }

    // rest-to-rest Poly4 to a target, then hold there
    TrajectoryManager tm;
    core_trajectoryPoly4Params_t poly{ .initialPos = {{0,0,0}}, .initialYaw = 0.0,
        .initialVel = {{0,0,0}}, .initialYawRate = 0.0,
        .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .finalVel = {{0,0,0}},
        .finalYawRate = 0.0, .finalAcc = {{0,0,0}}, .finalYawAcc = 0.0, .time_s = 2.0 };
    core_trajectoryPointParams_t hold{ .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .time_s = 5.0 };
    if (tm.AppendPoly4(poly) || tm.AppendPoint(hold)) { printf("trajectory build failed\n"); return 1; }
    if (model.SetTrajectoryManager(&tm)) { printf("SetTrajectoryManager failed\n"); return 1; }

    // Tunable-parameter interface, split by domain: controller (4 cost weights +
    // terminal + dt_mpc + max_iters = 8 rw rows), model (horizon, ro), observer
    // (enable + 4 covariances = 5) and sensor (3 axes x enable/bias/noise = 9).
    bool paramOk = true;
    {
        char buf[2048] = {0};
        auto rowsOf = [&](const char* b){ int r = 0; for (const char* q = b; *q; ++q) if (*q == '\n') ++r; return r; };

        if (model.GetControllerManifest(buf, sizeof buf) || rowsOf(buf) != 8) paramOk = false;
        if (model.SetControllerParam(0, 8.0)) paramOk = false;    // id 0 = weights/position (rw)
        if (!model.SetControllerParam(99, 1.0)) paramOk = false;  // bad id -> rejected
        model.SetControllerParam(0, 6.0);                         // restore the default weight

        if (model.GetModelManifest(buf, sizeof buf) || rowsOf(buf) != 1) paramOk = false;  // horizon (rw)
        if (model.SetModelParam(0, 60.0)) paramOk = false;        // horizon writable -> accepted
        if (!model.SetModelParam(0, 0.0)) paramOk = false;        // horizon < 1 -> rejected
        if (!model.SetModelParam(0, 9999.0)) paramOk = false;     // horizon > MAX_HORIZON -> rejected
        model.SetModelParam(0, 40.0);                             // restore the default horizon

        if (model.GetObserverManifest(buf, sizeof buf) || rowsOf(buf) != 5) paramOk = false;
        if (model.GetSensorManifest(buf, sizeof buf)   || rowsOf(buf) != 9) paramOk = false;
        printf("param manifests (controller/model/observer/sensor) checks = %s\n", paramOk ? "ok" : "FAIL");
    }

    const double dt  = 0.01;
    const int    SIM = 350;                 // 3.5 s
    double maxErr = 0.0, maxTilt = 0.0;
    bool finite_ok = true;
    core_state_t st{};
    for (int t = 0; t < SIM; ++t)
    {
        // lateral force gust between 1.0 s and 1.3 s
        const double fx = (t >= 100 && t < 130) ? 12.0 : 0.0;
        core_stepParams_t sp{ .timestep = dt, .user_fX = fx, .user_fY = 0.0, .user_fZ = 0.0 };
        if (model.PerformIntegration(sp)) { printf("PerformIntegration failed at t=%d\n", t); return 1; }

        model.GetState(st);
        core_coord_t now; model.GetCurrentTimeSeconds(now);
        Reference_t r; tm.GetReference(now, r);
        const double e = std::sqrt((st.x-r.pos[0])*(st.x-r.pos[0]) + (st.y-r.pos[1])*(st.y-r.pos[1]) + (st.z-r.pos[2])*(st.z-r.pos[2]));
        maxErr  = std::max(maxErr, e);
        maxTilt = std::max(maxTilt, std::sqrt(st.roll*st.roll + st.pitch*st.pitch) * 180.0 / M_PI);
        if (!std::isfinite(st.x) || !std::isfinite(st.y) || !std::isfinite(st.z)) finite_ok = false;
    }

    // final state vs the held target
    const double fErr = std::sqrt((st.x-2.0)*(st.x-2.0) + (st.y-1.0)*(st.y-1.0) + (st.z-0.5)*(st.z-0.5));
    const double fTilt = std::sqrt(st.roll*st.roll + st.pitch*st.pitch) * 180.0 / M_PI;
    printf("peak tracking error = %.3f m,  peak tilt = %.2f deg\n", maxErr, maxTilt);
    printf("final pos = (%.3f, %.3f, %.3f),  final tracking error = %.4f m,  final tilt = %.3f deg\n",
           st.x, st.y, st.z, fErr, fTilt);

    const bool ok = paramOk && finite_ok && (maxErr < 0.6) && (fErr < 0.10) && (fTilt < 3.0);
    printf("%s\n", ok ? "MPC MODEL TEST PASSED" : "MPC MODEL TEST FAILED");
    return ok ? 0 : 1;
}

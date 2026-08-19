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
// File        : rocket_mpc_model_test.cpp
// Description : Native acid test of the RocketMPC runtime model driven through
//               its public BaseModel interface: it tracks a Poly4 rest-to-rest
//               manoeuvre (exercising the horizon preview), rejects a mid-flight
//               force gust, and must arrive near the target -- position and
//               heading -- with a settled attitude while staying finite. The
//               rocket is torque-limited (thrust ~1e2 N vs torques ~1e1 N.m), so
//               the manoeuvre is gentle and the tolerances reflect the vehicle's
//               real authority, not a quadrotor's.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <cmath>
#include <cstdio>

#include "RocketMPC.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

int main()
{
    RocketMPC model;

    // airframe parameters (same as the notebook / runtime defaults)
    core_rocketParams_t p{ .m = 10.0, .Ix = 10.0/3, .Iy = 10.0/3, .Iz = 1.0,
                           .g = 9.81, .c = 1.0, .cz = 0.02,
                           .F1_max = 500, .F1_min = 0,
                           .T1_max = 10, .T1_min = -10, .T2_max = 10, .T2_min = -10,
                           .T3_max = 10, .T3_min = -10 };
    if (model.SetModelParams(std::any(p))) { printf("SetModelParams failed\n"); return 1; }

    // gentle rest-to-rest Poly4 (position + a yaw turn) to a target, then hold
    TrajectoryManager tm;
    core_trajectoryPoly4Params_t poly{ .initialPos = {{0,0,0}}, .initialYaw = 0.0,
        .initialVel = {{0,0,0}}, .initialYawRate = 0.0,
        .finalPos = {{0.6, 0.4, 0.9}}, .finalYaw = 0.3, .finalVel = {{0,0,0}},
        .finalYawRate = 0.0, .finalAcc = {{0,0,0}}, .finalYawAcc = 0.0, .time_s = 2.4 };
    core_trajectoryPointParams_t hold{ .finalPos = {{0.6, 0.4, 0.9}}, .finalYaw = 0.3, .time_s = 10.0 };
    if (tm.AppendPoly4(poly) || tm.AppendPoint(hold)) { printf("trajectory build failed\n"); return 1; }
    if (model.SetTrajectoryManager(&tm)) { printf("SetTrajectoryManager failed\n"); return 1; }

    // Controller-parameter interface: 4 state weights + 4 per-input control
    // weights + terminal + dt_mpc + max_iters (rw) + horizon (ro) = 12 rows; a
    // weight is settable, the horizon is not, a bad id fails.
    bool paramOk = true;
    {
        char buf[2048] = {0};
        if (model.GetControllerManifest(buf, sizeof buf)) paramOk = false;
        int rows = 0; for (const char* q = buf; *q; ++q) if (*q == '\n') ++rows;
        if (rows != 12) paramOk = false;
        if (model.SetControllerParam(0, 8.0)) paramOk = false;    // id 0 = Q/position (rw)
        if (!model.SetControllerParam(11, 100.0)) paramOk = false; // id 11 = solver/horizon (ro) -> rejected
        if (!model.SetControllerParam(99, 1.0)) paramOk = false;   // bad id -> rejected
        model.SetControllerParam(0, 6.0);                          // restore the default weight
        printf("controller manifest = %d rows, set/reject checks = %s\n", rows, paramOk ? "ok" : "FAIL");
    }

    const double dt  = 0.01;
    const int    SIM = 450;                 // 4.5 s
    double maxErr = 0.0, maxTilt = 0.0;
    bool finite_ok = true;
    core_state_t st{};
    for (int t = 0; t < SIM; ++t)
    {
        // lateral force gust between 1.5 s and 1.8 s (30 N on a 10 kg body)
        const double fx = (t >= 150 && t < 180) ? 30.0 : 0.0;
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

    // final state vs the held target (position + heading)
    const double fErr  = std::sqrt((st.x-0.6)*(st.x-0.6) + (st.y-0.4)*(st.y-0.4) + (st.z-0.9)*(st.z-0.9));
    const double fTilt = std::sqrt(st.roll*st.roll + st.pitch*st.pitch) * 180.0 / M_PI;
    double eyaw = st.yaw - 0.3; eyaw = std::atan2(std::sin(eyaw), std::cos(eyaw));
    const double fYaw  = std::fabs(eyaw) * 180.0 / M_PI;
    printf("peak tracking error = %.3f m,  peak tilt = %.2f deg\n", maxErr, maxTilt);
    printf("final pos = (%.3f, %.3f, %.3f),  final tracking error = %.4f m,  final tilt = %.3f deg,  final yaw error = %.3f deg\n",
           st.x, st.y, st.z, fErr, fTilt, fYaw);

    const bool ok = paramOk && finite_ok && (maxErr < 0.6) && (fErr < 0.10) && (fTilt < 3.0) && (fYaw < 2.0);
    printf("%s\n", ok ? "ROCKET MPC MODEL TEST PASSED" : "ROCKET MPC MODEL TEST FAILED");
    return ok ? 0 : 1;
}

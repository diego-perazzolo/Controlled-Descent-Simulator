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
// File        : determinism_model_test.cpp
// Description : Determinism regression for every runtime model. A pure
//               simulation is fixed-step by design, so the same model, fed the
//               same trajectory and the same per-tick inputs, must produce the
//               EXACT same trace -- bit for bit, not "close enough". The test
//               runs each model twice from a fresh instance and compares every
//               sample, with the observer on and the position sensor noisy so
//               the estimator and the sensor random stream are part of what is
//               being pinned. It catches the whole class of accidental
//               non-determinism: a reordered sensor draw, a global seed, state
//               leaking between instances, uninitialised memory.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <algorithm>   // std::max
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>     // std::atoi
#include <string>
#include <vector>

#include "QuadRotor.hpp"
#include "QuadRotorMPC.hpp"
#include "Rocket.hpp"
#include "RocketMPC.hpp"
#include "TrajectoryManager.hpp"

using namespace CDS;

namespace
{
    using ManifestGetter = bool (BaseModel::*)(char*, std::size_t);
    using ParamSetter    = bool (BaseModel::*)(int, double);

    // Write `value` into every writable manifest row whose label starts with
    // `label`. Returns the number of rows set, or -1 if the manifest is
    // unreadable. Driving the model by label keeps the test independent of the
    // parameter ids, which are free to be renumbered.
    int setRowsByLabel(BaseModel& model, ManifestGetter get, ParamSetter set,
                       const char* label, double value)
    {
        char buf[4096] = {0};
        if ((model.*get)(buf, sizeof buf)) return -1;

        int applied = 0;
        const std::string text(buf);
        std::size_t pos = 0;
        while (pos < text.size())
        {
            const std::size_t eol = text.find('\n', pos);
            const std::string line = text.substr(pos, eol == std::string::npos ? eol : eol - pos);
            pos = (eol == std::string::npos) ? text.size() : eol + 1;

            // row layout: id \t group \t label \t rw|ro \t value
            std::vector<std::string> field;
            std::size_t start = 0;
            for (;;)
            {
                const std::size_t tab = line.find('\t', start);
                field.push_back(line.substr(start, tab == std::string::npos ? tab : tab - start));
                if (tab == std::string::npos) break;
                start = tab + 1;
            }
            if (field.size() < 4) continue;
            if (field[3] != "rw") continue;
            if (field[2].rfind(label, 0) != 0) continue;
            if (!(model.*set)(std::atoi(field[0].c_str()), value)) ++applied;
        }
        return applied;
    }

    // One recorded run: the full state and tracking error at every tick, with a
    // lateral gust in the middle so the user-force path is exercised too.
    template <class Model, class Params>
    bool runTrace(const Params& frame, std::vector<double>& trace)
    {
        Model model;
        if (model.SetModelParams(std::any(frame))) return true;

        TrajectoryManager tm;
        core_trajectoryPoly4Params_t poly{ .initialPos = {{0,0,0}}, .initialYaw = 0.0,
            .initialVel = {{0,0,0}}, .initialYawRate = 0.0,
            .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .finalVel = {{0,0,0}},
            .finalYawRate = 0.0, .finalAcc = {{0,0,0}}, .finalYawAcc = 0.0, .time_s = 2.0 };
        core_trajectoryPointParams_t hold{ .finalPos = {{2.0, 1.0, 0.5}}, .finalYaw = 0.0, .time_s = 3.0 };
        if (tm.AppendPoly4(poly) || tm.AppendPoint(hold)) return true;
        if (model.SetTrajectoryManager(&tm)) return true;

        // Estimator ON and sensors noisy: the observer state and the sensor's
        // random stream then take part in the trace, so a reordered or reseeded
        // draw shows up as a mismatch.
        if (setRowsByLabel(model, &BaseModel::GetObserverManifest, &BaseModel::SetObserverParam,
                           "enable", 1.0) < 1) return true;
        if (setRowsByLabel(model, &BaseModel::GetSensorManifest, &BaseModel::SetSensorParam,
                           "noise std", 0.02) < 1) return true;

        const double dt  = 0.01;
        const int    SIM = 300;                 // 3 s
        for (int t = 0; t < SIM; ++t)
        {
            const double fx = (t >= 100 && t < 130) ? 8.0 : 0.0;   // gust
            core_stepParams_t sp{ .timestep = dt, .user_fX = fx, .user_fY = 0.0, .user_fZ = 0.0 };
            if (model.PerformIntegration(sp)) return true;

            core_state_t st{};
            core_trackingErrors_t err{};
            core_coord_t now = 0.0;
            model.GetState(st);
            model.GetTrackingErrors(err);
            model.GetCurrentTimeSeconds(now);

            trace.insert(trace.end(), {now,
                st.x, st.y, st.z, st.x_dot, st.y_dot, st.z_dot,
                st.roll, st.pitch, st.yaw, st.roll_dot, st.pitch_dot, st.yaw_dot,
                err.x, err.y, err.z, err.yaw});
        }
        return false;
    }

    // A trace that never moves would compare equal for the wrong reason: require
    // the run to have actually flown before trusting the comparison.
    bool traceMoved(const std::vector<double>& trace)
    {
        double span = 0.0;
        for (std::size_t i = 1; i < trace.size(); i += 17)          // channel 1 = x
            span = std::max(span, std::fabs(trace[i] - trace[1]));
        return span > 0.1;
    }

    template <class Model, class Params>
    bool checkModel(const char* name, const Params& frame)
    {
        std::vector<double> first, second;
        if (runTrace<Model>(frame, first) || runTrace<Model>(frame, second))
        {
            std::printf("  %-22s run failed\n", name);
            return false;
        }
        if (!traceMoved(first))
        {
            std::printf("  %-22s trace never moved -- the comparison would be vacuous\n", name);
            return false;
        }
        if (first.size() != second.size())
        {
            std::printf("  %-22s trace length differs (%zu vs %zu)\n", name, first.size(), second.size());
            return false;
        }

        for (std::size_t i = 0; i < first.size(); ++i)
        {
            if (first[i] != second[i])                              // bitwise, on purpose
            {
                std::printf("  %-22s DIVERGES at sample %zu: %.17g vs %.17g (delta %.3g)\n",
                            name, i, first[i], second[i], second[i] - first[i]);
                return false;
            }
        }
        std::printf("  %-22s identical over %zu samples\n", name, first.size());
        return true;
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
    std::printf("determinism: same model, same inputs, twice -- traces must match bit for bit\n");

    bool ok = true;
    ok = checkModel<QuadRotor>   ("QuadRotor FF-LQR", quadframe())   && ok;
    ok = checkModel<Rocket>      ("Rocket FF-LQR",    rocketframe()) && ok;
    ok = checkModel<QuadRotorMPC>("QuadRotor MPC",    quadframe())   && ok;
    ok = checkModel<RocketMPC>   ("Rocket MPC",       rocketframe()) && ok;

    std::printf("%s\n", ok ? "DETERMINISM MODEL TEST PASSED" : "DETERMINISM MODEL TEST FAILED");
    return ok ? 0 : 1;
}

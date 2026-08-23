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
// File        : sensor_model_test.cpp
// Description : Self-contained acid test of libs/sensor/sensor_model.hpp. Checks
//               the identity passthrough, exact bias with noise off, runtime
//               disable (measurement withheld / marked invalid), the statistics
//               of the injected noise (sample mean -> truth+bias, sample std ->
//               noiseStd) and PRNG determinism (same seed -> same stream). No
//               core, no external deps.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include "sensor_model.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using CDS::sensor::SensorModel;

namespace
{
    int g_failures = 0;

    void check(bool ok, const char* what)
    {
        if (!ok)
        {
            std::printf("  FAIL: %s\n", what);
            ++g_failures;
        }
    }
}

int main()
{
    // ---- 1. identity passthrough (fresh model touches nothing) --------------
    {
        SensorModel<3> s;
        std::array<double, 3> truth{{1.0, -2.0, 3.5}}, meas{};
        std::array<bool, 3>   valid{};
        s.Apply(truth, meas, valid);
        bool ok = true;
        for (std::size_t i = 0; i < 3; ++i)
            ok = ok && (meas[i] == truth[i]) && valid[i];
        check(ok, "identity passthrough leaves the truth untouched, all valid");
        std::printf("identity          : meas == truth, all valid = %s\n",
                    ok ? "yes" : "no");
    }

    // ---- 2. exact bias with noise off ---------------------------------------
    {
        SensorModel<2> s;
        s.SetChannel(0, {true,  0.75, 0.0});
        s.SetChannel(1, {true, -1.25, 0.0});
        std::array<double, 2> truth{{10.0, 20.0}}, meas{};
        std::array<bool, 2>   valid{};
        s.Apply(truth, meas, valid);
        check(meas[0] == 10.75 && meas[1] == 18.75, "exact additive bias, noise off");
        std::printf("bias (noise off)  : meas = [%.4f, %.4f]  (expect [10.7500, 18.7500])\n",
                    meas[0], meas[1]);
    }

    // ---- 3. runtime disable withholds the measurement -----------------------
    {
        SensorModel<3> s;
        s.SetChannel(0, {true, 0.0, 0.5});   // noisy but on
        s.SetEnabled(1, false);              // dropped sensor
        std::array<double, 3> truth{{5.0, 6.0, 7.0}}, meas{};
        std::array<bool, 3>   valid{};
        s.Apply(truth, meas, valid);
        check(valid[0] && !valid[1] && valid[2],
              "disabled channel reports invalid, others valid");
        check(meas[1] == 6.0, "disabled channel passes truth through (convenience)");
        std::printf("runtime disable   : valid = [%d, %d, %d]  (expect [1, 0, 1])\n",
                    int(valid[0]), int(valid[1]), int(valid[2]));
    }

    // ---- 4. noise statistics: sample mean -> truth+bias, std -> noiseStd -----
    {
        const double    bias  = 2.0;
        const double    sigma = 0.5;
        const double    truth = 100.0;
        const int       N     = 200000;
        SensorModel<1>  s;
        s.SetChannel(0, {true, bias, sigma});

        double sum = 0.0, sumsq = 0.0;
        std::array<double, 1> t{{truth}}, m{};
        std::array<bool, 1>   v{};
        for (int k = 0; k < N; ++k)
        {
            s.Apply(t, m, v);
            const double e = m[0] - (truth + bias);
            sum   += e;
            sumsq += e * e;
        }
        const double mean = sum / N;                       // -> 0
        const double var  = sumsq / N - mean * mean;       // -> sigma^2
        const double std  = std::sqrt(var);
        // ~1/sqrt(N) statistical tolerance, loosened for safety.
        check(std::fabs(mean) < 0.01,        "sample mean matches truth+bias");
        check(std::fabs(std - sigma) < 0.01, "sample std matches noiseStd");
        std::printf("noise stats       : mean_err = %.4e  std = %.5f  (expect ~0, %.5f)\n",
                    mean, std, sigma);
    }

    // ---- 5. determinism: same seed -> identical stream ----------------------
    {
        SensorModel<1> a(12345ull), b(12345ull);
        a.SetChannel(0, {true, 0.0, 1.0});
        b.SetChannel(0, {true, 0.0, 1.0});
        std::array<double, 1> t{{0.0}}, ma{}, mb{};
        std::array<bool, 1>   va{}, vb{};
        bool ok = true;
        for (int k = 0; k < 1000; ++k)
        {
            a.Apply(t, ma, va);
            b.Apply(t, mb, vb);
            ok = ok && (ma[0] == mb[0]);
        }
        check(ok, "identical seeds produce identical noise streams");
        std::printf("determinism       : same seed -> same stream = %s\n",
                    ok ? "yes" : "no");
    }

    if (g_failures == 0)
    {
        std::printf("SENSOR MODEL TEST PASSED\n");
        return EXIT_SUCCESS;
    }
    std::printf("SENSOR MODEL TEST FAILED (%d)\n", g_failures);
    return EXIT_FAILURE;
}

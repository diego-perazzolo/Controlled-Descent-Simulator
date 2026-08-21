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
// File        : tdo_test.cpp
// Description : Self-contained acid test of the reusable translational
//               disturbance observer (trans_disturbance_observer.hpp). A 2-axis
//               point mass under a constant unknown force is simulated; the
//               observer, fed the known force-free acceleration and the position
//               measurement, must recover the disturbance (d_hat -> d_true) and,
//               with one axis' sensor dropped mid-run, keep coasting finite. No
//               core, no external deps.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include "trans_disturbance_observer.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using CDS::estimate::TransDisturbanceObserver;
using CDS::estimate::Mat;

int main()
{
    constexpr std::size_t NP = 2;
    const double mass = 2.0;
    const double invm = 1.0 / mass;

    // disturbance-input coupling from "physics": Bd = (1/m) I
    Mat<NP, NP> Bd{}; for (auto& r : Bd) r.fill(0.0);
    Bd[0][0] = invm; Bd[1][1] = invm;

    // Fast, noise-free tuning: a large disturbance-to-measurement covariance ratio
    // places quick error poles so d_hat converges in ~1 s (this is a correctness
    // check of the helper, not a field tuning -- a real vehicle trades this off
    // against sensor noise).
    TransDisturbanceObserver<NP> tdo;
    if (tdo.Build(Bd, /*qPos*/1e-4, /*qVel*/1e-2, /*qDist*/1e1, /*rPos*/1e-4))
    {
        std::printf("TDO Build/synthesis failed\n");
        return 1;
    }

    // Truth: a point mass held at rest by a known acceleration that exactly
    // balances a constant unknown force d_true (a_known = -d_true/m). The vehicle
    // stays at the origin, so the observer must infer d_hat purely from the fact
    // that the mass does NOT accelerate despite the applied a_known -- isolating
    // disturbance estimation from any trajectory runaway.
    const std::array<double, NP> dTrue{{3.0, -1.5}};                 // external force [N]
    const std::array<double, NP> aKnown{{-dTrue[0]*invm, -dTrue[1]*invm}};
    std::array<double, NP> r{{0.0, 0.0}}, v{{0.0, 0.0}};
    tdo.Seed(r, v);

    const double dt   = 0.01;
    const int    SIM  = 2000;    // 20 s
    bool finite_ok = true;

    for (int k = 0; k < SIM; ++k)
    {
        // advance the truth: v_dot = a_known + d_true/m = 0  (stays at rest)
        for (std::size_t i = 0; i < NP; ++i)
        {
            v[i] += (aKnown[i] + dTrue[i] * invm) * dt;
            r[i] += v[i] * dt;
        }
        // measurement: exact position; drop axis 0 after 10 s -- well past
        // convergence -- so that axis must then coast on the held disturbance.
        std::array<double, NP> meas = r;
        std::array<bool, NP>   valid{{true, true}};
        if (k >= 1000) valid[0] = false;
        tdo.Step(aKnown, meas, valid, dt);

        for (std::size_t i = 0; i < NP; ++i)
            if (!std::isfinite(tdo.Disturbance()[i])) finite_ok = false;
    }

    const auto dHat = tdo.Disturbance();
    const auto rHat = tdo.Position();
    const double eD0 = std::fabs(dHat[0] - dTrue[0]);
    const double eD1 = std::fabs(dHat[1] - dTrue[1]);

    std::printf("disturbance recovery: d_hat = [%.4f, %.4f]  (true [%.4f, %.4f])\n",
                dHat[0], dHat[1], dTrue[0], dTrue[1]);
    std::printf("  |err| = [%.4e, %.4e]   pos coast (dropped axis) x_hat[0]=%.4f\n",
                eD0, eD1, rHat[0]);

    // axis 1 (always measured) must converge tightly; axis 0 (dropped at 20 s)
    // holds the value it had already learned -- still close, and finite.
    const bool converge = eD1 < 1e-2 && eD0 < 5e-2;
    const bool ok = finite_ok && converge;
    std::printf("checks: finite=%d converge=%d\n", finite_ok, converge);
    std::printf("%s\n", ok ? "TDO TEST PASSED" : "TDO TEST FAILED");
    return ok ? 0 : 1;
}

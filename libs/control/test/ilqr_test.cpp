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
// File        : ilqr_test.cpp
// Description : Self-contained acid test for the generic control-limited iLQR
//               solver (libs/control/ilqr.hpp). Uses a plain planar
//               double-integrator plant (no quaternions, no core) so it
//               exercises only the solver machinery: RK4 sensitivity, the
//               box-QP, the backward/forward passes and the receding-horizon
//               warm-start. Two closed-loop runs from a large offset: one with a
//               loose actuator box (must reach the origin) and one with a tight
//               box (must stay feasible, make progress and never diverge).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ilqr.hpp"
#include "rk4.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using namespace CDS;

namespace {

constexpr std::size_t NX = 4;   // [x, y, vx, vy]
constexpr std::size_t NU = 2;   // [Fx, Fy]
constexpr std::size_t N  = 30;  // horizon
constexpr double MASS = 1.0, DRAG = 0.10, DT = 0.05;

using State = std::array<double, NX>;
using Input = std::array<double, NU>;

// planar point mass with linear drag
State f(const State& x, const Input& u)
{
    return State{{ x[2], x[3], u[0] / MASS - DRAG * x[2], u[1] / MASS - DRAG * x[3] }};
}

void jac(const State&, const Input&, double fx[NX][NX], double fu[NX][NU])
{
    for (std::size_t i = 0; i < NX; ++i) { for (std::size_t j = 0; j < NX; ++j) fx[i][j] = 0.0;
                                           for (std::size_t a = 0; a < NU; ++a) fu[i][a] = 0.0; }
    fx[0][2] = 1.0; fx[1][3] = 1.0; fx[2][2] = -DRAG; fx[3][3] = -DRAG;
    fu[2][0] = 1.0 / MASS; fu[3][1] = 1.0 / MASS;
}

// quadratic regulation cost to the origin (weights chosen, not residual-derived)
constexpr double QP = 4.0, QV = 1.0, RU = 0.05, WTERM = 20.0;

control::StageCost<NX, NU> stageCost(const State& x, const Input& u, std::size_t)
{
    control::StageCost<NX, NU> c;
    c.lx = {{ QP * x[0], QP * x[1], QV * x[2], QV * x[3] }};
    c.lxx[0][0] = QP; c.lxx[1][1] = QP; c.lxx[2][2] = QV; c.lxx[3][3] = QV;
    c.lu = {{ RU * u[0], RU * u[1] }};
    c.luu[0][0] = RU; c.luu[1][1] = RU;
    c.val = 0.5 * (QP * (x[0]*x[0] + x[1]*x[1]) + QV * (x[2]*x[2] + x[3]*x[3]) + RU * (u[0]*u[0] + u[1]*u[1]));
    return c;
}

control::TerminalCost<NX> termCost(const State& x)
{
    control::TerminalCost<NX> c;
    c.lx = {{ WTERM * QP * x[0], WTERM * QP * x[1], WTERM * QV * x[2], WTERM * QV * x[3] }};
    c.lxx[0][0] = WTERM * QP; c.lxx[1][1] = WTERM * QP; c.lxx[2][2] = WTERM * QV; c.lxx[3][3] = WTERM * QV;
    c.val = 0.5 * WTERM * (QP * (x[0]*x[0] + x[1]*x[1]) + QV * (x[2]*x[2] + x[3]*x[3]));
    return c;
}

// closed-loop receding-horizon run; returns final position error, sets boxOK/finite
double runClosedLoop(double umax, bool& boxOK, bool& finite)
{
    State x{{ 4.0, -3.0, 0.0, 0.0 }};
    std::array<Input, N> warm; for (auto& u : warm) u.fill(0.0);
    const Input lo{{ -umax, -umax }}, hi{{ umax, umax }};
    auto noProj = [](State&){};
    boxOK = true; finite = true;

    for (int step = 0; step < 240; ++step)
    {
        Input u0;
        control::solve<NX, NU, N>(x, f, jac, noProj, stageCost, termCost, lo, hi, DT, 20, N, warm, u0);
        for (std::size_t a = 0; a < NU; ++a)
        {
            if (u0[a] < lo[a] - 1e-9 || u0[a] > hi[a] + 1e-9) boxOK = false;
            if (!std::isfinite(u0[a])) finite = false;
        }
        x = integrate::rk4_step<NX>(x, DT, [&](const State& s){ return f(s, u0); });
        for (double xi : x) if (!std::isfinite(xi)) finite = false;
    }
    return std::sqrt(x[0]*x[0] + x[1]*x[1]);
}

} // namespace

int main()
{
    bool ok = true;

    // Case 1: loose box -> must reach the origin.
    bool box1, fin1;
    const double err1 = runClosedLoop(8.0, box1, fin1);
    std::printf("loose box : final pos error = %.4f m,  box respected = %s\n", err1, box1 ? "yes" : "no");
    if (!(err1 < 0.05) || !box1 || !fin1) ok = false;

    // Case 2: tight box -> stay feasible, make progress, never diverge.
    bool box2, fin2;
    const double err2 = runClosedLoop(1.0, box2, fin2);
    std::printf("tight box : final pos error = %.4f m,  box respected = %s\n", err2, box2 ? "yes" : "no");
    if (!(err2 < 0.30) || !box2 || !fin2) ok = false;

    std::printf(ok ? "ILQR SOLVER TEST PASSED\n" : "ILQR SOLVER TEST FAILED\n");
    return ok ? 0 : 1;
}

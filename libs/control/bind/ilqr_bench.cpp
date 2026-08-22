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
// File        : ilqr_bench.cpp
// Description : C ABI shim that exposes the generic iLQR solver
//               (libs/control/ilqr.hpp) to Python (via ctypes) on a fixed,
//               vehicle-agnostic *synthetic benchmark* -- three coupled damped
//               oscillators with a cubic softening nonlinearity. The benchmark
//               has no physical meaning; it is sized and shaped to stress the
//               solver (nonlinearity + cross-coupling + a control box). The same
//               model is reproduced on the Python side by ilqr_conformance.py,
//               which certifies that the command sequence the C++ solver returns
//               is a genuine constrained optimum. Wire-crossing types are
//               `double` and `int` only; the solver template is instantiated
//               here with concrete dims.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ilqr.hpp"

#include <array>
#include <cmath>
#include <cstddef>

using namespace CDS;

namespace {

constexpr std::size_t NX = 6;   // [p0,v0, p1,v1, p2,v2]  -- three 2nd-order channels
constexpr std::size_t NU = 3;   // one actuator per channel
constexpr std::size_t N  = 25;  // horizon
constexpr double DT = 0.05;

// benchmark constants (deterministic, meaningless -- must match the Python side)
constexpr double OMEGA[3] = { 1.3, 0.8, 1.7 };   // natural frequencies
constexpr double ZETA[3]  = { 0.10, 0.05, 0.15 }; // damping ratios
constexpr double GAIN[3]  = { 1.0, 1.2, 0.9 };   // actuator gains
constexpr double BETA[3]  = { 0.20, 0.35, 0.15 }; // cubic softening coefficients
constexpr double KAPPA    = 0.4;                  // ring coupling p_{i+1}-p_i

// cost weights
constexpr double QP = 3.0, QV = 0.5, RU = 0.08, WTERM = 15.0;

using State = std::array<double, NX>;
using Input = std::array<double, NU>;

State benchF(const State& x, const Input& u)
{
    State d{};
    for (int i = 0; i < 3; ++i)
    {
        const double p = x[2*i], v = x[2*i + 1], pn = x[2*((i + 1) % 3)];
        d[2*i]     = v;
        d[2*i + 1] = -OMEGA[i]*OMEGA[i]*p - 2.0*ZETA[i]*OMEGA[i]*v
                     + GAIN[i]*u[i] - BETA[i]*p*p*p + KAPPA*(pn - p);
    }
    return d;
}

void benchJac(const State& x, const Input&, double fx[NX][NX], double fu[NX][NU])
{
    for (std::size_t i = 0; i < NX; ++i) { for (std::size_t j = 0; j < NX; ++j) fx[i][j] = 0.0;
                                           for (std::size_t a = 0; a < NU; ++a) fu[i][a] = 0.0; }
    for (int i = 0; i < 3; ++i)
    {
        const int pi = 2*i, vi = 2*i + 1, ni = 2*((i + 1) % 3);
        const double p = x[pi];
        fx[pi][vi] = 1.0;
        fx[vi][pi] = -OMEGA[i]*OMEGA[i] - 3.0*BETA[i]*p*p - KAPPA;
        fx[vi][vi] = -2.0*ZETA[i]*OMEGA[i];
        fx[vi][ni] += KAPPA;
        fu[vi][i]  = GAIN[i];
    }
}

control::StageCost<NX, NU> benchStage(const State& x, const Input& u, std::size_t)
{
    control::StageCost<NX, NU> c;
    for (int i = 0; i < 3; ++i)
    {
        c.lx[2*i]   = QP * x[2*i];   c.lxx[2*i][2*i]     = QP;
        c.lx[2*i+1] = QV * x[2*i+1]; c.lxx[2*i+1][2*i+1] = QV;
    }
    for (std::size_t a = 0; a < NU; ++a) { c.lu[a] = RU * u[a]; c.luu[a][a] = RU; }
    double s = 0.0;
    for (int i = 0; i < 3; ++i) s += QP*x[2*i]*x[2*i] + QV*x[2*i+1]*x[2*i+1];
    for (std::size_t a = 0; a < NU; ++a) s += RU*u[a]*u[a];
    c.val = 0.5 * s;
    return c;
}

control::TerminalCost<NX> benchTerm(const State& x)
{
    control::TerminalCost<NX> c;
    double s = 0.0;
    for (int i = 0; i < 3; ++i)
    {
        c.lx[2*i]   = WTERM * QP * x[2*i];   c.lxx[2*i][2*i]     = WTERM * QP;
        c.lx[2*i+1] = WTERM * QV * x[2*i+1]; c.lxx[2*i+1][2*i+1] = WTERM * QV;
        s += QP*x[2*i]*x[2*i] + QV*x[2*i+1]*x[2*i+1];
    }
    c.val = 0.5 * WTERM * s;
    return c;
}

} // namespace

extern "C" {

// Report the compiled-in dimensions so the caller can size its buffers.
void ilqr_bench_dims(int* nx, int* nu, int* n)
{
    *nx = static_cast<int>(NX); *nu = static_cast<int>(NU); *n = static_cast<int>(N);
}

// One warm-started solve. warm_in / warm_out are the N*NU command sequences
// (row-major, stage-major). The full converged sequence is recoverable from the
// return values as us[0] = u0_out, us[k] = warm_out[k-1] for k in [1, N).
void ilqr_bench_solve(const double* x0, double umax, int maxIters,
                      const double* warm_in, double* u0_out, double* warm_out)
{
    State x; for (std::size_t i = 0; i < NX; ++i) x[i] = x0[i];
    std::array<Input, N> warm;
    for (std::size_t k = 0; k < N; ++k) for (std::size_t a = 0; a < NU; ++a) warm[k][a] = warm_in[k*NU + a];

    Input lo, hi; lo.fill(-umax); hi.fill(umax);
    Input u0;
    control::solve<NX, NU, N>(x, benchF, benchJac, [](State&){}, benchStage, benchTerm,
                              lo, hi, DT, maxIters, N, warm, u0);

    for (std::size_t a = 0; a < NU; ++a) u0_out[a] = u0[a];
    for (std::size_t k = 0; k < N; ++k) for (std::size_t a = 0; a < NU; ++a) warm_out[k*NU + a] = warm[k][a];
}

} // extern "C"

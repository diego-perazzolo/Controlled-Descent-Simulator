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
// File        : lqr_bench.cpp
// Description : C ABI shim that exposes the generic continuous-time LQR synthesis
//               (libs/control/lqr.hpp) to Python (via ctypes). It is a thin,
//               vehicle-agnostic pass-through: the caller supplies the plant
//               (A, B) and weights (Q, R) as row-major flat arrays and gets back
//               the optimal gain K. The synthetic benchmark itself is defined on
//               the Python side (lqr_conformance.py), which then certifies K is a
//               genuine LQR optimum via an independent Lyapunov-stationarity
//               check -- no re-run of the sign-function solver. Wire-crossing
//               types are `double` and `int` only; the solver template is
//               instantiated here at fixed dims NX x NU.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "lqr.hpp"

#include <cstddef>

using namespace CDS;

namespace {
constexpr std::size_t NX = 6;   // synthetic benchmark: three 2nd-order channels
constexpr std::size_t NU = 3;   // one actuator per channel
} // namespace

extern "C" {

// Report the compiled-in dimensions so the caller can size its buffers.
void lqr_bench_dims(int* nx, int* nu)
{
    *nx = static_cast<int>(NX); *nu = static_cast<int>(NU);
}

// Solve one LQR problem. A (NX*NX), B (NX*NU), Q (NX*NX), R (NU*NU) are row-major
// flat inputs; K (NU*NX) is the row-major output. Returns 0 on success, 1 on a
// solver error (non-invertible R, sign iteration failure, singular normal eqs).
int lqr_bench_solve(const double* A, const double* B,
                    const double* Q, const double* R, double* K)
{
    control::Mat<NX, NX> Am, Qm;
    control::Mat<NX, NU> Bm;
    control::Mat<NU, NU> Rm;
    control::Mat<NU, NX> Km;

    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j) { Am[i][j] = A[i*NX + j]; Qm[i][j] = Q[i*NX + j]; }
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t a = 0; a < NU; ++a) Bm[i][a] = B[i*NU + a];
    for (std::size_t a = 0; a < NU; ++a)
        for (std::size_t b = 0; b < NU; ++b) Rm[a][b] = R[a*NU + b];

    if (control::lqr<NX, NU>(Am, Bm, Qm, Rm, Km)) return 1;

    for (std::size_t a = 0; a < NU; ++a)
        for (std::size_t j = 0; j < NX; ++j) K[a*NX + j] = Km[a][j];
    return 0;
}

} // extern "C"

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
// File        : observer_bench.cpp
// Description : C ABI shim that exposes the generic continuous-time observer
//               synthesis (libs/estimate/observer.hpp) to Python (via ctypes). It
//               is a thin, vehicle-agnostic pass-through: the caller supplies the
//               plant (A, C) and noise covariances (Qw, Rv) as row-major flat
//               arrays and gets back the steady-state observer gain L. The
//               synthetic benchmark itself is defined on the Python side
//               (observer_conformance.py), which then certifies L is a genuine
//               filter optimum via an independent Lyapunov-stationarity check --
//               the dual of the LQR certificate, no re-run of the sign-function
//               solver. Wire-crossing types are `double` and `int` only; the
//               solver template is instantiated here at fixed dims NX x NY.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "observer.hpp"

#include <cstddef>

using namespace CDS;

namespace {
constexpr std::size_t NX = 6;   // synthetic benchmark: three 2nd-order channels
constexpr std::size_t NY = 3;   // one position measurement per channel
} // namespace

extern "C" {

// Report the compiled-in dimensions so the caller can size its buffers.
void observer_bench_dims(int* nx, int* ny)
{
    *nx = static_cast<int>(NX); *ny = static_cast<int>(NY);
}

// Synthesise one observer gain. A (NX*NX), C (NY*NX), Qw (NX*NX), Rv (NY*NY) are
// row-major flat inputs; L (NX*NY) is the row-major output. Returns 0 on success,
// 1 on a solver error (non-invertible Rv, sign iteration failure, singular
// normal eqs) -- propagated from the underlying LQR solve.
int observer_bench_solve(const double* A, const double* C,
                         const double* Qw, const double* Rv, double* L)
{
    estimate::Mat<NX, NX> Am, Qm;
    estimate::Mat<NY, NX> Cm;
    estimate::Mat<NY, NY> Rm;
    estimate::Mat<NX, NY> Lm;

    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j) { Am[i][j] = A[i*NX + j]; Qm[i][j] = Qw[i*NX + j]; }
    for (std::size_t a = 0; a < NY; ++a)
        for (std::size_t j = 0; j < NX; ++j) Cm[a][j] = C[a*NX + j];
    for (std::size_t a = 0; a < NY; ++a)
        for (std::size_t b = 0; b < NY; ++b) Rm[a][b] = Rv[a*NY + b];

    if (estimate::observer_gain<NX, NY>(Am, Cm, Qm, Rm, Lm)) return 1;

    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t a = 0; a < NY; ++a) L[i*NY + a] = Lm[i][a];
    return 0;
}

} // extern "C"

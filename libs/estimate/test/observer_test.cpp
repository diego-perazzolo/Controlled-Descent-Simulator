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
// File        : observer_test.cpp
// Description : Self-contained acid test for the generic continuous-time observer
//               synthesis (libs/estimate/observer.hpp). No core, no scipy: the
//               synthesis cases mirror the LQR acid test by duality -- a scalar
//               plant with an exact gain, the double integrator observing position
//               only (dual of the classic LQR double integrator, L = [1; sqrt(3)]),
//               and an open-loop-unstable plant that the observer must render
//               detectable (the error dynamics A - L C must be Hurwitz). One
//               dynamic case drives Step() and checks the estimate converges to a
//               known constant state. The full cross-check against a numerical
//               oracle is the separate conformance certificate under bind/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "observer.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using namespace CDS;
using estimate::Mat;

namespace {

// A 2x2 matrix is Hurwitz iff its trace is negative and its determinant positive.
bool hurwitz2(const Mat<2, 2>& M)
{
    const double tr  = M[0][0] + M[1][1];
    const double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    return tr < 0.0 && det > 0.0;
}

// Error dynamics A - L C for a 2-state, 1-output observer.
Mat<2, 2> errDyn(const Mat<2, 2>& A, const Mat<2, 1>& L, const Mat<1, 2>& C)
{
    return {{ {{A[0][0] - L[0][0]*C[0][0], A[0][1] - L[0][0]*C[0][1]}},
              {{A[1][0] - L[1][0]*C[0][0], A[1][1] - L[1][0]*C[0][1]}} }};
}

} // namespace

int main()
{
    bool ok = true;

    // -- Case 1: scalar plant x' = a x + w, y = c x + v.  Dual of the scalar LQR
    //    case: L = (a + sqrt(a^2 + c^2 Qw / Rv)) / c.  a=0.5,c=2,Qw=3,Rv=1 -> L = 2.
    {
        const double a = 0.5, c = 2.0, qw = 3.0, rv = 1.0;
        const Mat<1, 1> A{{ {{a}} }}, C{{ {{c}} }}, Qw{{ {{qw}} }}, Rv{{ {{rv}} }};
        Mat<1, 1> L;
        const bool err = estimate::observer_gain<1, 1>(A, C, Qw, Rv, L);
        const double Lexp = (a + std::sqrt(a*a + c*c*qw/rv)) / c;    // = 2.0
        const double e = std::fabs(L[0][0] - Lexp);
        std::printf("scalar plant      : L = %.6f  (expect %.6f)  err = %.2e\n", L[0][0], Lexp, e);
        if (err || e > 1e-9) ok = false;
    }

    // -- Case 2: double integrator observing position only. Transpose of the LQR
    //    double integrator, so the known gain is L = [1; sqrt(3)]. State [x1, x2]
    //    with x1' = 0, x2' = x1; measure x2.
    {
        const Mat<2, 2> A{{ {{0.0, 0.0}}, {{1.0, 0.0}} }};
        const Mat<1, 2> C{{ {{0.0, 1.0}} }};
        const Mat<2, 2> Qw{{ {{1.0, 0.0}}, {{0.0, 1.0}} }};
        const Mat<1, 1> Rv{{ {{1.0}} }};
        Mat<2, 1> L;
        const bool err = estimate::observer_gain<2, 1>(A, C, Qw, Rv, L);
        const double e0 = std::fabs(L[0][0] - 1.0);
        const double e1 = std::fabs(L[1][0] - std::sqrt(3.0));
        const bool stable = hurwitz2(errDyn(A, L, C));
        std::printf("double integrator : L = [%.6f, %.6f]  (expect [1, 1.732051])  err = %.2e  stable = %s\n",
                    L[0][0], L[1][0], std::fmax(e0, e1), stable ? "yes" : "no");
        if (err || e0 > 1e-9 || e1 > 1e-9 || !stable) ok = false;
    }

    // -- Case 3: open-loop-unstable 2-state plant, measuring one state, must be
    //    made detectable: the error dynamics A - L C must be Hurwitz.
    //    A has eigenvalues +1 and +2 (both unstable).
    {
        const Mat<2, 2> A{{ {{1.0, 0.0}}, {{1.0, 2.0}} }};
        const Mat<1, 2> C{{ {{0.0, 1.0}} }};
        const Mat<2, 2> Qw{{ {{2.0, 0.0}}, {{0.0, 1.0}} }};
        const Mat<1, 1> Rv{{ {{0.5}} }};
        Mat<2, 1> L;
        const bool err = estimate::observer_gain<2, 1>(A, C, Qw, Rv, L);
        const Mat<2, 2> Aerr = errDyn(A, L, C);
        const bool stable = hurwitz2(Aerr);
        const bool finite = std::isfinite(L[0][0]) && std::isfinite(L[1][0]);
        std::printf("unstable plant    : L = [%.6f, %.6f]  error-dynamics stable = %s\n",
                    L[0][0], L[1][0], stable ? "yes" : "no");
        if (err || !stable || !finite) ok = false;
    }

    // -- Case 4: dynamic. Drive Step() on the (stable) double-integrator-dual
    //    observer with a plant sitting at a constant true state; the estimate,
    //    started wrong, must converge to it. u is unused here (B = 0).
    {
        const Mat<2, 2> A{{ {{0.0, 0.0}}, {{1.0, 0.0}} }};
        const Mat<1, 2> C{{ {{0.0, 1.0}} }};
        estimate::LinearObserver<2, 1, 1> obs;
        obs.A = A;
        obs.B = Mat<2, 1>{{ {{0.0}}, {{0.0}} }};
        obs.C = C;
        const Mat<2, 2> Qw{{ {{1.0, 0.0}}, {{0.0, 1.0}} }};
        const Mat<1, 1> Rv{{ {{1.0}} }};
        const bool err = obs.Synthesize(Qw, Rv);

        // True constant state x1 = 0 (so x2' = x1 = 0 keeps x2 fixed), x2 = 1.5.
        const std::array<double, 2> xTrue{{ 0.0, 1.5 }};
        std::array<double, 2> xhat{{ 3.0, -2.0 }};        // deliberately wrong
        const std::array<double, 1> u{{ 0.0 }};
        const std::array<double, 1> y{{ C[0][0]*xTrue[0] + C[0][1]*xTrue[1] }};  // = 1.5
        for (int k = 0; k < 4000; ++k) xhat = obs.Step(xhat, u, y, 0.01);        // 40 s

        const double e = std::fmax(std::fabs(xhat[0] - xTrue[0]), std::fabs(xhat[1] - xTrue[1]));
        std::printf("dynamic converge  : x_hat = [%.6f, %.6f]  (true [0, 1.5])  err = %.2e\n",
                    xhat[0], xhat[1], e);
        if (err || e > 1e-6) ok = false;
    }

    std::printf(ok ? "OBSERVER SOLVER TEST PASSED\n" : "OBSERVER SOLVER TEST FAILED\n");
    return ok ? 0 : 1;
}

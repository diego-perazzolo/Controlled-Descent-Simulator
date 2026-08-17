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
// File        : lqr_test.cpp
// Description : Self-contained acid test for the generic continuous-time LQR
//               synthesis (libs/control/lqr.hpp). No core, no scipy: it checks
//               the returned gain on cases with a closed-form answer (the 1-D
//               double integrator, K = [1, sqrt(3)]; a scalar plant with an
//               exact K) and certifies that an open-loop-unstable plant is
//               stabilised (the closed loop A - BK is Hurwitz). The full
//               cross-check against a numerical oracle (scipy / python-control)
//               is the separate conformance certificate under bind/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "lqr.hpp"

#include <array>
#include <cmath>
#include <cstdio>

using namespace CDS;
using control::Mat;

namespace {

// A 2x2 matrix is Hurwitz iff its trace is negative and its determinant positive.
bool hurwitz2(const Mat<2, 2>& M)
{
    const double tr  = M[0][0] + M[1][1];
    const double det = M[0][0] * M[1][1] - M[0][1] * M[1][0];
    return tr < 0.0 && det > 0.0;
}

} // namespace

int main()
{
    bool ok = true;

    // -- Case 1: 1-D double integrator, unit weights. Known gain K = [1, sqrt(3)].
    {
        const Mat<2, 2> A{{ {{0.0, 1.0}}, {{0.0, 0.0}} }};
        const Mat<2, 1> B{{ {{0.0}},      {{1.0}} }};
        const Mat<2, 2> Q{{ {{1.0, 0.0}}, {{0.0, 1.0}} }};
        const Mat<1, 1> R{{ {{1.0}} }};
        Mat<1, 2> K;
        const bool err = control::lqr<2, 1>(A, B, Q, R, K);
        const double e0 = std::fabs(K[0][0] - 1.0);
        const double e1 = std::fabs(K[0][1] - std::sqrt(3.0));
        // closed loop A_cl = A - B K
        const Mat<2, 2> Acl{{ {{A[0][0] - B[0][0]*K[0][0], A[0][1] - B[0][0]*K[0][1]}},
                              {{A[1][0] - B[1][0]*K[0][0], A[1][1] - B[1][0]*K[0][1]}} }};
        const bool stable = hurwitz2(Acl);
        std::printf("double integrator : K = [%.6f, %.6f]  (expect [1, 1.732051])  err = %.2e  stable = %s\n",
                    K[0][0], K[0][1], std::fmax(e0, e1), stable ? "yes" : "no");
        if (err || e0 > 1e-9 || e1 > 1e-9 || !stable) ok = false;
    }

    // -- Case 2: scalar plant x' = a x + b u, cost q x^2 + r u^2.
    //    K = (a + sqrt(a^2 + b^2 q / r)) / b.  a=0.5,b=2,q=3,r=1 -> K = 2 exactly.
    {
        const double a = 0.5, b = 2.0, q = 3.0, r = 1.0;
        const Mat<1, 1> A{{ {{a}} }}, B{{ {{b}} }}, Q{{ {{q}} }}, R{{ {{r}} }};
        Mat<1, 1> K;
        const bool err = control::lqr<1, 1>(A, B, Q, R, K);
        const double Kexp = (a + std::sqrt(a*a + b*b*q/r)) / b;    // = 2.0
        const double e = std::fabs(K[0][0] - Kexp);
        std::printf("scalar plant      : K = %.6f  (expect %.6f)  err = %.2e\n", K[0][0], Kexp, e);
        if (err || e > 1e-9) ok = false;
    }

    // -- Case 3: open-loop-unstable 2-state plant must be stabilised.
    //    A has eigenvalues +1 and +2 (both unstable); the closed loop must be Hurwitz.
    {
        const Mat<2, 2> A{{ {{1.0, 1.0}}, {{0.0, 2.0}} }};
        const Mat<2, 1> B{{ {{0.0}},      {{1.0}} }};
        const Mat<2, 2> Q{{ {{2.0, 0.0}}, {{0.0, 1.0}} }};
        const Mat<1, 1> R{{ {{0.5}} }};
        Mat<1, 2> K;
        const bool err = control::lqr<2, 1>(A, B, Q, R, K);
        const Mat<2, 2> Acl{{ {{A[0][0] - B[0][0]*K[0][0], A[0][1] - B[0][0]*K[0][1]}},
                              {{A[1][0] - B[1][0]*K[0][0], A[1][1] - B[1][0]*K[0][1]}} }};
        const bool stable = hurwitz2(Acl);
        const bool finite = std::isfinite(K[0][0]) && std::isfinite(K[0][1]);
        std::printf("unstable plant    : K = [%.6f, %.6f]  closed-loop stable = %s\n",
                    K[0][0], K[0][1], stable ? "yes" : "no");
        if (err || !stable || !finite) ok = false;
    }

    std::printf(ok ? "LQR SOLVER TEST PASSED\n" : "LQR SOLVER TEST FAILED\n");
    return ok ? 0 : 1;
}

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
// File        : observer.hpp
// Description : Generic continuous-time linear state observer (steady-state
//               Kalman-Bucy filter). Domain-agnostic numerics: given the
//               linearised plant (A, B, C), the process- and measurement-noise
//               covariances (Qw, Rv), it synthesises the optimal observer gain L
//               and propagates an estimate x_hat with the correction
//               x_hat' = A x_hat + B u + L (y - C x_hat). It knows nothing about
//               vehicles or disturbance models -- the caller assembles the
//               matrices (a disturbance-augmented model is just a larger A/B/C).
//
//               The gain is synthesised as the exact DUAL of the LQR problem: the
//               steady-state filter gain L = P C' Rv^-1, with P the stabilising
//               solution of the filtering CARE
//                   A P + P A' - P C' Rv^-1 C P + Qw = 0,
//               equals lqr(A', C', Qw, Rv)'. So this header reuses the already
//               certified matrix-sign Riccati solver in libs/control/lqr.hpp and
//               adds only the transpose plumbing -- new code kept minimal. This is
//               an init-time, one-shot synthesis; only Step() runs on the tick
//               path. Fixed compile-time dimensions (NX, NU, NY); no heap.
//
//               Two companions to the gain itself: observable() rank tests the
//               (A, C) pair -- the dual of the control layer's controllable() --
//               and is checked before every synthesis, so a measurement layout
//               that cannot see a state fails loudly; and the Riccati solution P,
//               the stationary error covariance, is kept and read back through
//               Covariance() / Sigma() for the per-state 1 sigma bands.
//               Header-only; depends only on <array>, <cstddef>, the LQR solver
//               (libs/control/lqr.hpp) and the RK4 step (libs/integrate/rk4.hpp),
//               so it may live under libs/. It ships a C++<->Python conformance
//               certificate (libs/estimate/bind/observer_conformance.py), the dual
//               of the LQR one.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cmath>     // std::sqrt, for the 1 sigma bands
#include <cstddef>

#include "lqr.hpp"                     // libs/control -- Mat + certified Riccati
#include "rk4.hpp"                     // libs/integrate -- fixed-control RK4 step

namespace CDS { namespace estimate {

// Reuse the row-major fixed-size dense matrix from the control layer so callers
// mix observer and controller matrices without conversions.
using CDS::control::Mat;

// -----------------------------------------------------------------------------
// Example (standalone) -- position observer for the 1-D double integrator
// [position, velocity] measuring position only, with unit noise covariances.
// The dual of the classic LQR double integrator; the known gain is L = [1; sqrt(3)]:
//
//   using CDS::estimate::Mat;
//   Mat<2,2> A{{ {{0,0}}, {{1,0}} }};      // x1' = 0,  x2' = x1
//   Mat<1,2> C{{ {{0,1}} }};               // measure the second state
//   Mat<2,2> Qw{{ {{1,0}}, {{0,1}} }};     // process-noise covariance
//   Mat<1,1> Rv{{ {{1}} }};                // measurement-noise covariance
//   Mat<2,1> L;
//   if (CDS::estimate::observer_gain<2,1>(A, C, Qw, Rv, L)) { /* synthesis failed */ }
//   // L ~= [1; 1.7320508]; the correction is L * (y - C x_hat).
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//  observable() -- Kalman rank test on the pair (A, C), the dual of the control
//  layer's controllable(): (A, C) is observable exactly when (A', C') is
//  controllable, so this is one transpose away from the certified helper rather
//  than a second implementation.
//
//  True when every state direction leaves a trace in the measurement. Sufficient
//  (not necessary) for the synthesis: a merely DETECTABLE pair -- unobservable
//  but already-stable modes -- also admits a stabilising observer, so a false
//  here means "look at your measurement", not "the synthesis cannot work".
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NY>
bool observable(const Mat<NX, NX>& A, const Mat<NY, NX>& C, double relTol = 1e-9)
{
    Mat<NX, NX> At;
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j) At[i][j] = A[j][i];

    Mat<NX, NY> Ct;
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t a = 0; a < NY; ++a) Ct[i][a] = C[a][i];

    return control::controllable<NX, NY>(At, Ct, relTol);
}

// -----------------------------------------------------------------------------
//  observer_gain() -- continuous-time steady-state (Kalman-Bucy) observer gain.
//
//  Returns the L minimising the stationary estimation-error covariance for
//      x' = A x + B u + w,   y = C x + v,   cov(w) = Qw,  cov(v) = Rv,
//  namely L = P C' Rv^-1 with P the stabilising solution of the filtering CARE
//      A P + P A' - P C' Rv^-1 C P + Qw = 0.
//
//  Pout, when given, receives P itself: the stationary covariance of the
//  estimation error, whose diagonal is the per-state variance (sqrt -> the 1
//  sigma band around each estimate).
//
//  By LQR/filter duality this is exactly the transpose of an LQR gain: with
//  Abar = A', Bbar = C', Qbar = Qw, Rbar = Rv, the LQR solver returns
//  Kbar = Rbar^-1 Bbar' X = Rv^-1 C P, whose transpose is L. So the whole Riccati
//  solve is delegated to the certified lqr() and only the transposes are new.
//
//  Requirements: Rv symmetric positive definite; Qw symmetric positive
//  semidefinite; (A, C) detectable and (A, Qw^{1/2}) stabilisable (so the dual
//  (A', C') is stabilisable / detectable, i.e. the Hamiltonian has no imaginary
//  eigenvalue). Returns true on error (propagated from lqr()); L is then left
//  unspecified.
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NY>
bool observer_gain(const Mat<NX, NX>& A, const Mat<NY, NX>& C,
                   const Mat<NX, NX>& Qw, const Mat<NY, NY>& Rv,
                   Mat<NX, NY>& L, int maxIters = 100, double tol = 1e-13,
                   Mat<NX, NX>* Pout = nullptr)
{
    // Dual LQR data: Abar = A' (NX x NX), Bbar = C' (NX x NY).
    Mat<NX, NX> At;
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j) At[i][j] = A[j][i];

    Mat<NX, NY> Ct;
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t a = 0; a < NY; ++a) Ct[i][a] = C[a][i];

    // Kbar = Rv^-1 C P  (NY x NX); L = Kbar'  (NX x NY). The dual problem's CARE
    // solution IS the estimation-error covariance P, so asking the control layer
    // for X is all it takes to hand P back.
    Mat<NY, NX> Kbar;
    if (control::lqr<NX, NY>(At, Ct, Qw, Rv, Kbar, maxIters, tol, Pout)) return true;

    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t a = 0; a < NY; ++a) L[i][a] = Kbar[a][i];
    return false;
}

// -----------------------------------------------------------------------------
//  LinearObserver -- a plant linearisation (A, B, C) plus its steady-state gain L,
//  advancing an estimate on the tick path. The gain is fixed (rate-invariant,
//  synthesised once from the covariances); a rate-varying / multi-rate Kalman
//  recursion is a deliberately separate future step. Members are plain data so a
//  caller may set L directly instead of synthesising it.
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NU, std::size_t NY>
struct LinearObserver
{
    Mat<NX, NX> A;
    Mat<NX, NU> B;
    Mat<NY, NX> C;
    Mat<NX, NY> L;

    // Filled by Synthesize(); read through Covariance() / Sigma(), which say so
    // when there is nothing to read yet.
    Mat<NX, NX> m_P{};
    bool        m_hasP{false};

    // Synthesise L from the noise covariances (dual LQR). Returns true on error;
    // A and C must already be populated. B is unused by the synthesis -- it only
    // feeds the deterministic prediction in Step(). The pair (A, C) is rank
    // tested first, so an unobservable measurement layout is refused up front
    // with a clear failure instead of surfacing as a Riccati that will not
    // converge -- or, worse, as a gain that quietly means nothing. On success the
    // stationary error covariance P is kept, and can be read back for the 1 sigma
    // bands (see Covariance() / Sigma()).
    bool Synthesize(const Mat<NX, NX>& Qw, const Mat<NY, NY>& Rv,
                    int maxIters = 100, double tol = 1e-13)
    {
        if (!observable<NX, NY>(A, C)) return true;
        m_hasP = false;
        if (observer_gain<NX, NY>(A, C, Qw, Rv, L, maxIters, tol, &m_P)) return true;
        m_hasP = true;
        return false;
    }

    // Stationary covariance of the estimation error, as synthesised. Returns true
    // on error (no successful Synthesize yet), leaving `P` untouched.
    bool Covariance(Mat<NX, NX>& P) const
    {
        if (!m_hasP) return true;
        P = m_P;
        return false;
    }

    // One-sigma bound on the estimate of state `i` -- sqrt of that state's
    // variance, i.e. how far off this estimate is expected to sit at steady
    // state. Returns a negative value when no covariance is available.
    double Sigma(std::size_t i) const
    {
        if (!m_hasP || i >= NX || m_P[i][i] < 0.0) return -1.0;
        return std::sqrt(m_P[i][i]);
    }

    // Advance the estimate by dt with the continuous correction
    //   x_hat' = A x_hat + B u + L (y - C x_hat),
    // holding the command u and the measurement y fixed over the step (RK4, the
    // same integrator the plant uses). Non-allocating; safe on the tick path.
    std::array<double, NX> Step(const std::array<double, NX>& xhat,
                                const std::array<double, NU>& u,
                                const std::array<double, NY>& y,
                                double dt) const
    {
        auto deriv = [&](const std::array<double, NX>& s) -> std::array<double, NX> {
            // innovation e = y - C s   (NY)
            std::array<double, NY> e{};
            for (std::size_t a = 0; a < NY; ++a)
            {
                double cs = 0.0;
                for (std::size_t j = 0; j < NX; ++j) cs += C[a][j] * s[j];
                e[a] = y[a] - cs;
            }
            // dx = A s + B u + L e   (NX)
            std::array<double, NX> dx{};
            for (std::size_t i = 0; i < NX; ++i)
            {
                double v = 0.0;
                for (std::size_t j = 0; j < NX; ++j) v += A[i][j] * s[j];
                for (std::size_t k = 0; k < NU; ++k) v += B[i][k] * u[k];
                for (std::size_t a = 0; a < NY; ++a) v += L[i][a] * e[a];
                dx[i] = v;
            }
            return dx;
        };
        return integrate::rk4_step<NX>(xhat, dt, deriv);
    }
};

}} // namespace CDS::estimate

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
// File        : lqr.hpp
// Description : Generic continuous-time LQR gain synthesis by the matrix-sign
//               function of the Hamiltonian. Domain-agnostic numerics: given the
//               plant (A, B), the state/input weights (Q, R), it returns the
//               optimal feedback K minimising the infinite-horizon cost
//               integral x'Qx + u'Ru with u = -Kx, i.e. K = R^-1 B' X where X is
//               the stabilising solution of the continuous algebraic Riccati
//               equation (CARE)  A'X + XA - X B R^-1 B' X + Q = 0. It knows
//               nothing about vehicles or references -- the caller assembles the
//               matrices. The stabilising X is read off the stable invariant
//               subspace of the Hamiltonian H = [[A,-G],[-Q,-A']], G = B R^-1 B',
//               obtained from sign(H) by a Newton iteration with Higham scaling
//               (quadratically convergent; needs only dense matmul and a 2n x 2n
//               inverse per step). This is an init-time, one-shot synthesis -- it
//               never runs on the tick path -- and the sibling of the runtime
//               iLQR in ilqr.hpp: symbolic model artifacts are generated, the
//               control algorithm is hand-written here and certified against a
//               Python oracle (see libs/control/bind/lqr_conformance.py).
//               Fixed compile-time dimensions (NX, NU); no heap. Header-only;
//               depends only on <array>, <cmath>, <cstddef>, so it may live under
//               libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <utility>   // std::swap, for the rank elimination

namespace CDS { namespace control {

// -----------------------------------------------------------------------------
// Example (standalone) -- LQR for the 1-D double integrator [position, velocity]
// with unit weights. The known optimal gain is K = [1, sqrt(3)]:
//
//   using CDS::control::Mat;
//   Mat<2,2> A{{ {{0,1}}, {{0,0}} }};      // x' = v,  v' = u
//   Mat<2,1> B{{ {{0}},   {{1}} }};
//   Mat<2,2> Q{{ {{1,0}}, {{0,1}} }};      // penalise position and velocity
//   Mat<1,1> R{{ {{1}} }};                 // penalise control
//   Mat<1,2> K;
//   if (CDS::control::lqr<2,1>(A, B, Q, R, K)) { /* synthesis failed */ }
//   // K[0] ~= [1.0, 1.7320508]; the control law is u = -(K[0][0]*x + K[0][1]*v).
// -----------------------------------------------------------------------------

// Row-major fixed-size dense matrix (R rows x C cols).
template <std::size_t R, std::size_t C>
using Mat = std::array<std::array<double, C>, R>;

namespace detail {

// ---- C = A * B  (R x K times K x N) -----------------------------------------
template <std::size_t R, std::size_t K, std::size_t N>
void mul(const Mat<R, K>& A, const Mat<K, N>& B, Mat<R, N>& C)
{
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < N; ++j)
        {
            double s = 0.0;
            for (std::size_t k = 0; k < K; ++k) s += A[i][k] * B[k][j];
            C[i][j] = s;
        }
}

// ---- C = A^T * B  (A is K x R, B is K x N -> C is R x N) ---------------------
template <std::size_t K, std::size_t R, std::size_t N>
void mulAtB(const Mat<K, R>& A, const Mat<K, N>& B, Mat<R, N>& C)
{
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < N; ++j)
        {
            double s = 0.0;
            for (std::size_t k = 0; k < K; ++k) s += A[k][i] * B[k][j];
            C[i][j] = s;
        }
}

// ---- Frobenius norm ---------------------------------------------------------
template <std::size_t R, std::size_t C>
double fro(const Mat<R, C>& A)
{
    double s = 0.0;
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j) s += A[i][j] * A[i][j];
    return std::sqrt(s);
}

// ---- inv: dense inverse by Gauss-Jordan with partial pivoting ----------------
// Returns true on error (singular / non-finite); Ainv is then left unspecified.
template <std::size_t K>
bool inv(const Mat<K, K>& A, Mat<K, K>& Ainv)
{
    Mat<K, K> M = A;                 // working copy, reduced to identity
    for (std::size_t i = 0; i < K; ++i)
        for (std::size_t j = 0; j < K; ++j) Ainv[i][j] = (i == j) ? 1.0 : 0.0;

    for (std::size_t col = 0; col < K; ++col)
    {
        // partial pivot: largest magnitude in this column at/below the diagonal
        std::size_t piv = col;
        double best = std::fabs(M[col][col]);
        for (std::size_t r = col + 1; r < K; ++r)
        {
            const double v = std::fabs(M[r][col]);
            if (v > best) { best = v; piv = r; }
        }
        if (!(best > 0.0)) return true;             // singular
        if (piv != col) { std::swap(M[piv], M[col]); std::swap(Ainv[piv], Ainv[col]); }

        const double d = M[col][col];
        const double invd = 1.0 / d;
        for (std::size_t j = 0; j < K; ++j) { M[col][j] *= invd; Ainv[col][j] *= invd; }

        for (std::size_t r = 0; r < K; ++r)
        {
            if (r == col) continue;
            const double f = M[r][col];
            if (f == 0.0) continue;
            for (std::size_t j = 0; j < K; ++j) { M[r][j] -= f * M[col][j]; Ainv[r][j] -= f * Ainv[col][j]; }
        }
    }

    for (std::size_t i = 0; i < K; ++i)
        for (std::size_t j = 0; j < K; ++j)
            if (!std::isfinite(Ainv[i][j])) return true;
    return false;
}

// ---- numerical rank by Gauss-Jordan elimination with partial pivoting. The
//      tolerance is relative to the largest pivot seen, so it scales with the
//      matrix instead of assuming unit entries.
template <std::size_t R, std::size_t C>
std::size_t rank(Mat<R, C> M, double relTol = 1e-9)
{
    double biggest = 0.0;
    for (std::size_t i = 0; i < R; ++i)
        for (std::size_t j = 0; j < C; ++j) biggest = std::fmax(biggest, std::fabs(M[i][j]));
    if (biggest == 0.0) return 0;
    const double tol = relTol * biggest;

    std::size_t rows = 0;
    for (std::size_t col = 0; col < C && rows < R; ++col)
    {
        std::size_t piv = rows;
        for (std::size_t i = rows + 1; i < R; ++i)
            if (std::fabs(M[i][col]) > std::fabs(M[piv][col])) piv = i;
        if (std::fabs(M[piv][col]) <= tol) continue;              // column adds nothing

        std::swap(M[rows], M[piv]);
        const double p = M[rows][col];
        for (std::size_t i = 0; i < R; ++i)
        {
            if (i == rows) continue;
            const double f = M[i][col] / p;
            if (f == 0.0) continue;
            for (std::size_t j = col; j < C; ++j) M[i][j] -= f * M[rows][j];
        }
        ++rows;
    }
    return rows;
}

} // namespace detail

// -----------------------------------------------------------------------------
//  controllable() -- Kalman rank test on the pair (A, B).
//
//  True when [B, AB, A^2 B, ..., A^(n-1) B] has full row rank, i.e. every state
//  direction can be driven. Sufficient (not necessary) for lqr() to have a
//  stabilising solution: a pair that is merely STABILISABLE -- uncontrollable
//  but already-stable modes -- is fine for the synthesis too, so a false here is
//  a reason to look, not proof that the synthesis will fail.
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NU>
bool controllable(const Mat<NX, NX>& A, const Mat<NX, NU>& B, double relTol = 1e-9)
{
    Mat<NX, NX * NU> ctrb{};
    Mat<NX, NU> block = B;                       // A^k B, starting at k = 0
    for (std::size_t k = 0; k < NX; ++k)
    {
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t a = 0; a < NU; ++a) ctrb[i][k * NU + a] = block[i][a];

        Mat<NX, NU> next{};                      // next block = A * block
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t a = 0; a < NU; ++a)
            {
                double s = 0.0;
                for (std::size_t j = 0; j < NX; ++j) s += A[i][j] * block[j][a];
                next[i][a] = s;
            }
        block = next;
    }
    return detail::rank<NX, NX * NU>(ctrb, relTol) == NX;
}

// -----------------------------------------------------------------------------
//  lqr() -- continuous-time infinite-horizon LQR gain.
//
//  Minimises  J = integral_0^inf ( x'Q x + u'R u ) dt  for  x' = A x + B u,
//  the optimal law being  u = -K x  with  K = R^-1 B' X  and X the stabilising
//  (symmetric, positive-semidefinite) solution of the CARE
//        A' X + X A - X B R^-1 B' X + Q = 0.
//
//  Method: X is the graph of the Hamiltonian's stable invariant subspace. With
//  G = B R^-1 B' and  H = [[A, -G], [-Q, -A']],  the stable subspace is the
//  eigenspace of sign(H) for the eigenvalue -1, i.e. null(sign(H) + I). Writing
//  the basis as columns [I; X], sign(H) is computed by the scaled Newton
//  iteration  Z <- 1/2 (c Z + (c Z)^-1),  c = sqrt(||Z^-1||_F / ||Z||_F),
//  converging quadratically to S = sign(H) whenever H has no eigenvalue on the
//  imaginary axis (guaranteed for (A,B) stabilisable and (A, Q^{1/2}) detectable).
//  X then solves the overdetermined  [S12; S22+I] X = -[S11+I; S21]  in the
//  least-squares sense; it is symmetrised before forming K.
//
//  Requirements: R symmetric positive definite; Q symmetric positive
//  semidefinite; (A,B) stabilisable and (A, Q^{1/2}) detectable. Returns true on
//  error (non-invertible R, sign iteration that did not converge, or a singular
//  normal-equation solve) -- K is then left unspecified.
//
//  Xout, when given, receives the CARE solution X itself: the optimal
//  cost-to-go x'Xx of the control problem, and -- through duality -- the
//  stationary estimation-error covariance of the filtering problem.
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NU>
bool lqr(const Mat<NX, NX>& A, const Mat<NX, NU>& B,
         const Mat<NX, NX>& Q, const Mat<NU, NU>& R,
         Mat<NU, NX>& K, int maxIters = 100, double tol = 1e-13,
         Mat<NX, NX>* Xout = nullptr)
{
    constexpr std::size_t M2 = 2 * NX;

    // R^-1 and G = B R^-1 B'  (n x n, symmetric PSD)
    Mat<NU, NU> Rinv;
    if (detail::inv<NU>(R, Rinv)) return true;
    Mat<NX, NU> BRinv;                          // B R^-1  (n x m)
    detail::mul<NX, NU, NU>(B, Rinv, BRinv);
    Mat<NX, NX> G;                              // (B R^-1) B'  =  BRinv * B'
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j)
        {
            double s = 0.0;
            for (std::size_t a = 0; a < NU; ++a) s += BRinv[i][a] * B[j][a];   // (BRinv * B^T)
            G[i][j] = s;
        }

    // Hamiltonian  Z = [[A, -G], [-Q, -A']]
    Mat<M2, M2> Z{};
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j)
        {
            Z[i][j]           =  A[i][j];
            Z[i][NX + j]      = -G[i][j];
            Z[NX + i][j]      = -Q[i][j];
            Z[NX + i][NX + j] = -A[j][i];        // -A'
        }

    // Scaled Newton iteration for the matrix sign function
    bool converged = false;
    for (int it = 0; it < maxIters; ++it)
    {
        Mat<M2, M2> Zinv;
        if (detail::inv<M2>(Z, Zinv)) return true;

        const double nZ = detail::fro(Z), nZi = detail::fro(Zinv);
        double c = 1.0;
        if (nZ > 0.0 && nZi > 0.0) c = std::sqrt(nZi / nZ);
        const double ic = 1.0 / c;

        double diff = 0.0, scale = 0.0;
        Mat<M2, M2> Znew;
        for (std::size_t i = 0; i < M2; ++i)
            for (std::size_t j = 0; j < M2; ++j)
            {
                const double v = 0.5 * (c * Z[i][j] + ic * Zinv[i][j]);
                const double d = v - Z[i][j];
                diff += d * d; scale += v * v;
                Znew[i][j] = v;
            }
        Z = Znew;
        if (std::sqrt(diff) <= tol * (std::sqrt(scale) + 1.0)) { converged = true; break; }
    }
    if (!converged) return true;

    // Stable subspace: [I; X] spans null(S + I). Solve  U X = -W  (least squares)
    //   U = [S12; S22 + I]   (2n x n),   W = [S11 + I; S21]   (2n x n).
    Mat<M2, NX> U, W;
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j)
        {
            U[i][j]       = Z[i][NX + j];
            U[NX + i][j]  = Z[NX + i][NX + j] + ((i == j) ? 1.0 : 0.0);
            W[i][j]       = Z[i][j] + ((i == j) ? 1.0 : 0.0);
            W[NX + i][j]  = Z[NX + i][j];
        }

    // Normal equations:  (U'U) X = -(U'W)
    Mat<NX, NX> UtU, UtW, UtUinv;
    detail::mulAtB<M2, NX, NX>(U, U, UtU);
    detail::mulAtB<M2, NX, NX>(U, W, UtW);
    if (detail::inv<NX>(UtU, UtUinv)) return true;
    Mat<NX, NX> X;
    detail::mul<NX, NX, NX>(UtUinv, UtW, X);
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = 0; j < NX; ++j) X[i][j] = -X[i][j];

    // Symmetrise (the CARE solution is symmetric; the LS step may drift slightly)
    for (std::size_t i = 0; i < NX; ++i)
        for (std::size_t j = i + 1; j < NX; ++j)
        {
            const double a = 0.5 * (X[i][j] + X[j][i]);
            X[i][j] = a; X[j][i] = a;
        }

    if (Xout != nullptr) *Xout = X;      // the CARE solution itself, when asked for

    // K = R^-1 B' X  =  (B R^-1)' X
    for (std::size_t a = 0; a < NU; ++a)
        for (std::size_t j = 0; j < NX; ++j)
        {
            double s = 0.0;
            for (std::size_t i = 0; i < NX; ++i) s += BRinv[i][a] * X[i][j];
            if (!std::isfinite(s)) return true;
            K[a][j] = s;
        }
    return false;
}

}} // namespace CDS::control

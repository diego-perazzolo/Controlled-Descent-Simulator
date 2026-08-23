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
// File        : ilqr.hpp
// Description : Generic control-limited iLQR / DDP solver (Tassa, Erez & Todorov
//               2014). Domain-agnostic numerics: it knows nothing about
//               quaternions, vehicles or references -- the caller supplies the
//               continuous dynamics f(x,u) and its Jacobians, an optional
//               post-step projection (e.g. quaternion renorm), a per-stage cost
//               returning its Gauss-Newton gradient/Hessian, a terminal cost,
//               and per-input actuator bounds. Fixed compile-time dimensions
//               (NX,NU) and a horizon *capacity* CAP that sizes the buffers; the
//               active horizon N is a run-time choice (1 <= N <= CAP), so the
//               prediction length is tunable without recompiling. No heap, dense
//               fixed-size linear algebra,
//               Cholesky up to NU x NU. Discrete-time Jacobians A = dF/dx,
//               B = dF/du are built at run time by RK4 sensitivity propagation,
//               reusing libs/integrate/rk4.hpp for the nominal rollout. The
//               warm-start command sequence is owned by the caller and shifted
//               in place on every solve. Header-only; depends only on <array>,
//               <cmath>, <algorithm> and libs/integrate/rk4.hpp, so it may live
//               under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#ifndef CDS_ILQR_DEBUG
#define CDS_ILQR_DEBUG 0   // 1 = per-iteration solver trace on stdout
#endif

#if CDS_ILQR_DEBUG
#include <cstdio>          // debug instrumentation only (CDS_ILQR_DEBUG=1)
#endif

#include <array>
#include <cmath>
#include <algorithm>
#include <cstddef>

#include "rk4.hpp"

namespace CDS { namespace control {

// -----------------------------------------------------------------------------
// Example (standalone) -- receding-horizon regulation of a 1-D double
// integrator [position, velocity] to the origin with a bounded force. The
// caller supplies the dynamics, its Jacobians, an (empty) projection, the
// per-stage and terminal costs, and owns the warm-start sequence:
//
//   constexpr std::size_t NX = 2, NU = 1, CAP = 64;   // buffer capacity
//   const std::size_t N = 20;                          // active horizon (<= CAP)
//   using State = std::array<double, NX>;
//   using Input = std::array<double, NU>;
//
//   auto f = [](const State& x, const Input& u) -> State {
//       return {{ x[1], u[0] }};                       // pos' = vel,  vel' = force
//   };
//   auto jac = [](const State&, const Input&, double fx[NX][NX], double fu[NX][NU]) {
//       fx[0][0]=0; fx[0][1]=1; fx[1][0]=0; fx[1][1]=0; // df/dx
//       fu[0][0]=0; fu[1][0]=1;                         // df/du
//   };
//   auto project = [](State&){};                        // no state constraint
//   auto stage = [](const State& x, const Input& u, std::size_t) {
//       CDS::control::StageCost<NX,NU> c;               // l = 1/2(x0^2 + x1^2 + 0.1 u^2)
//       c.lx = {{ x[0], x[1] }}; c.lxx[0][0]=1; c.lxx[1][1]=1;
//       c.lu = {{ 0.1*u[0] }};   c.luu[0][0]=0.1;
//       c.val = 0.5*(x[0]*x[0] + x[1]*x[1] + 0.1*u[0]*u[0]);
//       return c;
//   };
//   auto terminal = [](const State& x) {
//       CDS::control::TerminalCost<NX> c;               // 10x state weight at the end
//       c.lx = {{ 10*x[0], 10*x[1] }}; c.lxx[0][0]=10; c.lxx[1][1]=10;
//       c.val = 0.5*10*(x[0]*x[0] + x[1]*x[1]);
//       return c;
//   };
//
//   std::array<Input, CAP> warm{};                      // warm start, owned by the caller
//   State x{{ 5.0, 0.0 }};                              // start 5 m off, at rest
//   // Actuator saturation -- the defining feature of this solver. The box is a
//   // hard constraint honoured *inside* the optimisation (not a post-clip): here
//   // |force| <= 0.5, tight enough that the first commands ride the bound.
//   const Input lo{{-0.5}}, hi{{0.5}};
//   for (int step = 0; step < 120; ++step) {            // receding horizon
//       Input u0;
//       CDS::control::solve<NX,NU,CAP>(x, f, jac, project, stage, terminal,
//                                      lo, hi, /*dt=*/0.05, /*maxIters=*/10, N, warm, u0);
//       // u0 is guaranteed within [lo, hi]; apply it and advance the plant.
//       x = CDS::integrate::rk4_step<NX>(x, 0.05, [&](const State& s){ return f(s, u0); });
//   }
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
//  Per-stage and terminal cost payloads the caller returns to the solver.
//  Weights and references live entirely on the caller side (baked into the
//  cost closures); the solver only ever sees the local quadratic model.
//    lx  = d l / d x        (STATE)
//    lxx = d^2 l / d x^2    (STATE x STATE, expected PSD -- Gauss-Newton)
//    lu  = d l / d u        (INPUT)
//    luu = d^2 l / d u^2    (INPUT x INPUT, expected PSD)
// -----------------------------------------------------------------------------
template <std::size_t NX, std::size_t NU>
struct StageCost
{
    double                                         val{0.0};
    std::array<double, NX>                         lx{};
    std::array<std::array<double, NX>, NX>         lxx{};
    std::array<double, NU>                         lu{};
    std::array<std::array<double, NU>, NU>         luu{};
};

template <std::size_t NX>
struct TerminalCost
{
    double                                         val{0.0};
    std::array<double, NX>                         lx{};
    std::array<std::array<double, NX>, NX>         lxx{};
};

namespace detail {

// ---- small SPD (Cholesky) solve for a free sub-block, up to NU x NU, multi-RHS.
//      Returns true on error (non-PD pivot) per the project convention.
template <std::size_t NU, std::size_t NCOLS>
bool solveSPD(const double H[NU][NU], int nf, const double R[NU][NCOLS], int nc, double X[NU][NCOLS])
{
    double L[NU][NU] = {{0}};
    for (int i = 0; i < nf; ++i)
        for (int j = 0; j <= i; ++j)
        {
            double s = H[i][j];
            for (int k = 0; k < j; ++k) s -= L[i][k] * L[j][k];
            if (i == j) { if (s <= 1e-12) return true; L[i][j] = std::sqrt(s); }
            else        L[i][j] = s / L[j][j];
        }
    for (int c = 0; c < nc; ++c)
    {
        double y[NU];
        for (int i = 0; i < nf; ++i)      { double s = R[i][c]; for (int k = 0; k < i; ++k)  s -= L[i][k] * y[k];      y[i]    = s / L[i][i]; }
        for (int i = nf - 1; i >= 0; --i) { double s = y[i];    for (int k = i + 1; k < nf; ++k) s -= L[k][i] * X[k][c]; X[i][c] = s / L[i][i]; }
    }
    return false;
}

// ---- box-constrained QP (projected Newton) on the control increment.
//      Minimises 1/2 z' H z + g' z subject to lo <= z <= hi; writes the argmin
//      into x and the active free set into freeMask.
template <std::size_t NU>
void boxQP(const std::array<std::array<double, NU>, NU>& H,
           const std::array<double, NU>& g,
           const std::array<double, NU>& lo, const std::array<double, NU>& hi,
           std::array<double, NU>& x, bool freeMask[NU])
{
    for (std::size_t a = 0; a < NU; ++a) x[a] = std::min(std::max(0.0, lo[a]), hi[a]);
    for (int it = 0; it < 40; ++it)
    {
        std::array<double, NU> grad;
        for (std::size_t a = 0; a < NU; ++a) { double s = g[a]; for (std::size_t b = 0; b < NU; ++b) s += H[a][b] * x[b]; grad[a] = s; }

        bool clamped[NU]; int nf = 0; int fidx[NU];
        for (std::size_t a = 0; a < NU; ++a)
        {
            clamped[a] = ((x[a] <= lo[a] && grad[a] > 0) || (x[a] >= hi[a] && grad[a] < 0));
            freeMask[a] = !clamped[a];
            if (freeMask[a]) fidx[nf++] = static_cast<int>(a);
        }
        if (nf == 0) break;

        double Hf[NU][NU]; double Rf[NU][1];
        for (int i = 0; i < nf; ++i)
        {
            double gf = g[fidx[i]];
            for (std::size_t b = 0; b < NU; ++b) if (clamped[b]) gf += H[fidx[i]][b] * x[b];
            Rf[i][0] = -gf;
            for (int j = 0; j < nf; ++j) Hf[i][j] = H[fidx[i]][fidx[j]];
        }
        double Xf[NU][1];
        if (solveSPD<NU, 1>(Hf, nf, Rf, 1, Xf)) break;

        std::array<double, NU> step{}; double maxstep = 0.0;
        for (int i = 0; i < nf; ++i) { step[fidx[i]] = Xf[i][0] - x[fidx[i]]; maxstep = std::max(maxstep, std::fabs(step[fidx[i]])); }
        if (maxstep < 1e-10) break;

        auto obj = [&](const std::array<double, NU>& z)
        {
            double s = 0;
            for (std::size_t a = 0; a < NU; ++a) { double Hz = 0; for (std::size_t b = 0; b < NU; ++b) Hz += H[a][b] * z[b]; s += 0.5 * z[a] * Hz + g[a] * z[a]; }
            return s;
        };
        const double c0 = obj(x); double a = 1.0; std::array<double, NU> xn;
        for (int ls = 0; ls < 20; ++ls)
        {
            for (std::size_t k = 0; k < NU; ++k) xn[k] = std::min(std::max(x[k] + a * step[k], lo[k]), hi[k]);
            if (obj(xn) <= c0) { x = xn; break; }
            a *= 0.5; if (ls == 19) x = xn;
        }
    }
}

// ---- RK4 sensitivity: discrete Jacobians A = dF/dx, B = dF/du of one RK4 step.
//      f is the continuous dynamics at the fixed command u; jac fills the
//      continuous Jacobians fx = df/dx, fu = df/du. Projection (if any) is a
//      separate nonlinear map applied by the caller after the step -- its
//      Jacobian is intentionally not folded in here (matches the nominal
//      rollout, which projects only at the end of the step).
template <std::size_t NX, std::size_t NU, class Dyn, class Jac>
void sensitivity(Dyn&& f, Jac&& jac, const std::array<double, NX>& x, const std::array<double, NU>& u,
                 double dt, std::array<std::array<double, NX>, NX>& A, std::array<std::array<double, NU>, NX>& B)
{
    using State = std::array<double, NX>;
    const State k1 = f(x, u);
    State x2, x3, x4;
    for (std::size_t i = 0; i < NX; ++i) x2[i] = x[i] + 0.5 * dt * k1[i];
    const State k2 = f(x2, u);
    for (std::size_t i = 0; i < NX; ++i) x3[i] = x[i] + 0.5 * dt * k2[i];
    const State k3 = f(x3, u);
    for (std::size_t i = 0; i < NX; ++i) x4[i] = x[i] + dt * k3[i];

    double fx1[NX][NX], fu1[NX][NU], fx2[NX][NX], fu2[NX][NU];
    double fx3[NX][NX], fu3[NX][NU], fx4[NX][NX], fu4[NX][NU];
    jac(x,  u, fx1, fu1);
    jac(x2, u, fx2, fu2);
    jac(x3, u, fx3, fu3);
    jac(x4, u, fx4, fu4);

    auto stepA = [&](const double fx[NX][NX], double cc, const std::array<std::array<double, NX>, NX>& Aprev,
                     std::array<std::array<double, NX>, NX>& Aout)
    {
        for (std::size_t i = 0; i < NX; ++i) for (std::size_t j = 0; j < NX; ++j)
        { double s = fx[i][j]; for (std::size_t m = 0; m < NX; ++m) s += fx[i][m] * (cc * Aprev[m][j]); Aout[i][j] = s; }
    };
    auto stepB = [&](const double fx[NX][NX], const double fu[NX][NU], double cc,
                     const std::array<std::array<double, NU>, NX>& Bprev, std::array<std::array<double, NU>, NX>& Bout)
    {
        for (std::size_t i = 0; i < NX; ++i) for (std::size_t a = 0; a < NU; ++a)
        { double s = fu[i][a]; for (std::size_t m = 0; m < NX; ++m) s += fx[i][m] * (cc * Bprev[m][a]); Bout[i][a] = s; }
    };

    std::array<std::array<double, NX>, NX> A1, A2, A3, A4;
    std::array<std::array<double, NU>, NX> B1, B2, B3, B4;
    for (std::size_t i = 0; i < NX; ++i)
    {
        for (std::size_t j = 0; j < NX; ++j) A1[i][j] = fx1[i][j];
        for (std::size_t a = 0; a < NU; ++a) B1[i][a] = fu1[i][a];
    }
    stepA(fx2, 0.5 * dt, A1, A2); stepB(fx2, fu2, 0.5 * dt, B1, B2);
    stepA(fx3, 0.5 * dt, A2, A3); stepB(fx3, fu3, 0.5 * dt, B2, B3);
    stepA(fx4, dt,       A3, A4); stepB(fx4, fu4, dt,       B3, B4);
    const double s6 = dt / 6.0;
    for (std::size_t i = 0; i < NX; ++i)
    {
        for (std::size_t j = 0; j < NX; ++j) A[i][j] = (i == j ? 1.0 : 0.0) + s6 * (A1[i][j] + 2 * A2[i][j] + 2 * A3[i][j] + A4[i][j]);
        for (std::size_t a = 0; a < NU; ++a) B[i][a] = s6 * (B1[i][a] + 2 * B2[i][a] + 2 * B3[i][a] + B4[i][a]);
    }
}

} // namespace detail

// -----------------------------------------------------------------------------
//  solve() -- one receding-horizon iLQR solve.
//
//  Rolls out the warm-started command sequence, runs up to maxIters iLQR
//  iterations (backward pass with a box-QP on the control increment + Levenberg
//  regularisation on Quu, forward pass with an alpha line-search on the true
//  projected dynamics), writes the first command into u0, and shifts warmStart
//  in place so the next solve starts a few iterations from the answer.
//
//  Callables:
//    f(x,u)                 -> continuous dx/dt (any external forcing is baked in
//                              by the caller's closure).
//    jac(x,u, fx,fu)        -> fills fx=df/dx [NX][NX], fu=df/du [NX][NU].
//    project(state&)        -> in-place post-step projection (e.g. quaternion
//                              renorm); pass a no-op if none is needed.
//    stageCost(x,u,k)       -> StageCost<NX,NU> at stage k in [0, N).
//    termCost(x)            -> TerminalCost<NX> at the horizon end (index N).
//  Bounds lo,hi are absolute per-input actuator limits. CAP sizes the fixed
//  buffers; N is the active horizon (1 <= N <= CAP) — the warmStart array is
//  CAP-sized but only [0, N) is used.
//
//  Returns true on error (bool-is-error), meaning the result must NOT be
//  applied: an out-of-range horizon, or a rollout / cost / command that came out
//  non-finite. On error u0 and warmStart are left untouched, so the caller keeps
//  whatever it was holding and the next solve restarts from the last good
//  sequence -- an erroring solve must never poison the state it was given.
//
//  A false return is not a promise of optimality: it says the numbers are usable
//  (finite, in-box). Over a horizon long compared with the plant's open-loop
//  instability the warm-start rollout can leave the region where the
//  linearisation means anything, and the solver then converges on a prediction
//  that has little to do with the vehicle -- finite, in-box, and wrong. Judging
//  that is the caller's business; see the report out-parameter.
// -----------------------------------------------------------------------------
// What one solve did, for callers that want to log or sanity-check it. `cost` is
// the trajectory cost of the sequence being returned, `stateMax` the largest
// absolute state value seen in its rollout (a runaway prediction shows up here
// long before it turns into a non-finite number), `accepted` how many of the
// `iterations` line searches improved the cost.
struct SolveReport
{
    double cost{0.0};
    double stateMax{0.0};
    int    iterations{0};
    int    accepted{0};
};

template <std::size_t NX, std::size_t NU, std::size_t CAP,
          class Dyn, class Jac, class Project, class StageCostFn, class TermCostFn>
bool solve(const std::array<double, NX>& x0,
           Dyn&& f, Jac&& jac, Project&& project,
           StageCostFn&& stageCost, TermCostFn&& termCost,
           const std::array<double, NU>& lo, const std::array<double, NU>& hi,
           double dt, int maxIters, std::size_t N,
           std::array<std::array<double, NU>, CAP>& warmStart,
           std::array<double, NU>& u0,
           SolveReport* report = nullptr)
{
    using State = std::array<double, NX>;
    using Input = std::array<double, NU>;
    using Mxx   = std::array<State, NX>;
    using Mxu   = std::array<Input, NX>;   // NX rows x NU cols
    using Mux   = std::array<State, NU>;   // NU rows x NX cols

    // The buffers are fixed at capacity CAP; the active horizon N is a runtime
    // choice that must fit (1 <= N <= CAP). Out of range is an error, not a
    // silent no-op: u0 would otherwise be left as the caller declared it.
    if (N == 0 || N > CAP) return true;

    // Largest |state| over a rollout, and whether it is all finite: the two
    // things that say whether a prediction is worth optimising around.
    auto rolloutScale = [&](const std::array<State, CAP + 1>& xs_, double& worst)
    {
        worst = 0.0;
        bool finite = true;
        for (std::size_t k = 0; k <= N; ++k)
            for (std::size_t i = 0; i < NX; ++i)
            {
                const double v = xs_[k][i];
                if (!std::isfinite(v)) { finite = false; continue; }
                const double a = (v < 0.0) ? -v : v;
                if (a > worst) worst = a;
            }
        return finite;
    };

    // one discrete step: RK4 + caller projection
    auto Fd = [&](const State& x, const Input& u)
    {
        State xn = integrate::rk4_step<NX>(x, dt, [&](const State& s) { return f(s, u); });
        project(xn);
        return xn;
    };
    auto trajCost = [&](const std::array<State, CAP + 1>& xs, const std::array<Input, CAP>& us)
    {
        double J = 0.0;
        for (std::size_t k = 0; k < N; ++k) J += stageCost(xs[k], us[k], k).val;
        J += termCost(xs[N]).val;
        return J;
    };

    std::array<Input, CAP>    us = warmStart;
    std::array<State, CAP + 1> xs; xs[0] = x0;
    for (std::size_t k = 0; k < N; ++k) xs[k + 1] = Fd(xs[k], us[k]);
    double J = trajCost(xs, us);

    // A warm start whose rollout already diverged gives gradients about a
    // trajectory the vehicle will never fly: refuse rather than optimise it.
    double stateMax = 0.0;
    if (!rolloutScale(xs, stateMax) || !std::isfinite(J)) return true;

#if CDS_ILQR_DEBUG
    {
        double worst = 0.0; bool finite = true;
        for (std::size_t k = 0; k <= N; ++k)
            for (std::size_t i = 0; i < NX; ++i)
            {
                if (!std::isfinite(xs[k][i])) finite = false;
                worst = std::fmax(worst, std::fabs(xs[k][i]));
            }
        std::printf("[ilqr] N=%zu rollout: J0=%.6g  |x|max=%.6g  finite=%d\n", N, J, worst, (int) finite);
    }
#endif

    std::array<Input, CAP> kff; std::array<Mux, CAP> K; double mu = 1e-3;
    int iterCount = 0, acceptedCount = 0;

    for (int iter = 0; iter < maxIters; ++iter)
    {
        ++iterCount;
        // terminal value model
        State Vx; Mxx Vxx;
        { TerminalCost<NX> tc = termCost(xs[N]); Vx = tc.lx; Vxx = tc.lxx; }

        for (int k = static_cast<int>(N) - 1; k >= 0; --k)
        {
            Mxx A; Mxu B; detail::sensitivity<NX, NU>(f, jac, xs[k], us[k], dt, A, B);
            StageCost<NX, NU> c = stageCost(xs[k], us[k], static_cast<std::size_t>(k));

            Mux BtVxx; Mxx AtVxx;
            for (std::size_t a = 0; a < NU; ++a) for (std::size_t j = 0; j < NX; ++j) { double s = 0; for (std::size_t i = 0; i < NX; ++i) s += B[i][a] * Vxx[i][j]; BtVxx[a][j] = s; }
            for (std::size_t p = 0; p < NX; ++p) for (std::size_t j = 0; j < NX; ++j) { double s = 0; for (std::size_t i = 0; i < NX; ++i) s += A[i][p] * Vxx[i][j]; AtVxx[p][j] = s; }

            State Qx; Input Qu; Mxx Qxx; std::array<Input, NU> Quu; Mux Qux;
            for (std::size_t p = 0; p < NX; ++p) { double s = c.lx[p]; for (std::size_t i = 0; i < NX; ++i) s += A[i][p] * Vx[i]; Qx[p] = s; }
            for (std::size_t a = 0; a < NU; ++a) { double s = c.lu[a]; for (std::size_t i = 0; i < NX; ++i) s += B[i][a] * Vx[i]; Qu[a] = s; }
            for (std::size_t p = 0; p < NX; ++p) for (std::size_t q = 0; q < NX; ++q) { double s = c.lxx[p][q]; for (std::size_t m = 0; m < NX; ++m) s += AtVxx[p][m] * A[m][q]; Qxx[p][q] = s; }
            for (std::size_t a = 0; a < NU; ++a) for (std::size_t b = 0; b < NU; ++b) { double s = c.luu[a][b]; for (std::size_t m = 0; m < NX; ++m) s += BtVxx[a][m] * B[m][b]; if (a == b) s += mu; Quu[a][b] = s; }
            for (std::size_t a = 0; a < NU; ++a) for (std::size_t j = 0; j < NX; ++j) { double s = 0; for (std::size_t m = 0; m < NX; ++m) s += BtVxx[a][m] * A[m][j]; Qux[a][j] = s; }

            Input lod, hid; for (std::size_t a = 0; a < NU; ++a) { lod[a] = lo[a] - us[k][a]; hid[a] = hi[a] - us[k][a]; }
            Input ki; bool freeMask[NU]; detail::boxQP<NU>(Quu, Qu, lod, hid, ki, freeMask);

            Mux Ki; for (auto& row : Ki) row.fill(0.0);
            int nf = 0, fidx[NU]; for (std::size_t a = 0; a < NU; ++a) if (freeMask[a]) fidx[nf++] = static_cast<int>(a);
            if (nf > 0)
            {
                double Hf[NU][NU], Rf[NU][NX], Xf[NU][NX];
                for (int i = 0; i < nf; ++i) { for (int j = 0; j < nf; ++j) Hf[i][j] = Quu[fidx[i]][fidx[j]];
                    for (std::size_t j = 0; j < NX; ++j) Rf[i][j] = -Qux[fidx[i]][j]; }
                if (!detail::solveSPD<NU, NX>(Hf, nf, Rf, static_cast<int>(NX), Xf))
                    for (int i = 0; i < nf; ++i) for (std::size_t j = 0; j < NX; ++j) Ki[fidx[i]][j] = Xf[i][j];
            }
            kff[k] = ki; K[k] = Ki;

            Input Quuk; for (std::size_t a = 0; a < NU; ++a) { double s = 0; for (std::size_t b = 0; b < NU; ++b) s += Quu[a][b] * ki[b]; Quuk[a] = s; }
            for (std::size_t p = 0; p < NX; ++p) { double s = Qx[p];
                for (std::size_t a = 0; a < NU; ++a) s += Ki[a][p] * Quuk[a] + Ki[a][p] * Qu[a] + Qux[a][p] * ki[a]; Vx[p] = s; }
            Mux QuuK; for (std::size_t a = 0; a < NU; ++a) for (std::size_t j = 0; j < NX; ++j) { double s = 0; for (std::size_t b = 0; b < NU; ++b) s += Quu[a][b] * Ki[b][j]; QuuK[a][j] = s; }
            Mxx Vnew;
            for (std::size_t p = 0; p < NX; ++p) for (std::size_t q = 0; q < NX; ++q) { double s = Qxx[p][q];
                for (std::size_t a = 0; a < NU; ++a) s += Ki[a][p] * QuuK[a][q] + Ki[a][p] * Qux[a][q] + Qux[a][p] * Ki[a][q]; Vnew[p][q] = s; }
            for (std::size_t p = 0; p < NX; ++p) for (std::size_t q = 0; q < NX; ++q) Vxx[p][q] = 0.5 * (Vnew[p][q] + Vnew[q][p]);
        }

        static const double alphas[] = {1.0, 0.5, 0.25, 0.125, 0.0625, 0.03, 0.015, 0.007};
        bool accepted = false;
        for (double a : alphas)
        {
            std::array<State, CAP + 1> xn; std::array<Input, CAP> un; xn[0] = xs[0];
            for (std::size_t k = 0; k < N; ++k)
            {
                Input du;
                for (std::size_t j = 0; j < NU; ++j)
                {
                    double fb = 0; for (std::size_t p = 0; p < NX; ++p) fb += K[k][j][p] * (xn[k][p] - xs[k][p]);
                    du[j] = a * kff[k][j] + fb;
                    un[k][j] = std::min(std::max(us[k][j] + du[j], lo[j]), hi[j]);
                }
                xn[k + 1] = Fd(xn[k], un[k]);
            }
            double Jn = trajCost(xn, un);
            if (Jn < J) { xs = xn; us = un; J = Jn; accepted = true; break; }
        }
#if CDS_ILQR_DEBUG
        std::printf("[ilqr]   iter %2d: J=%.6g  accepted=%d  mu=%.3g\n", iter, J, (int) accepted, mu);
#endif
        if (accepted) { ++acceptedCount; mu = std::max(mu * 0.7, 1e-6); }
        else           { mu *= 4.0; if (mu > 1e3) break; }
    }

    // Commit only once the answer is known to be usable: everything below writes
    // into the caller's state, and a half-written command sequence is worse than
    // no update at all.
    std::array<double, NU> u0Candidate;
    for (std::size_t j = 0; j < NU; ++j)
    {
        const double v = std::min(std::max(us[0][j], lo[j]), hi[j]);
        if (!std::isfinite(v)) return true;
        u0Candidate[j] = v;
    }
    if (!rolloutScale(xs, stateMax) || !std::isfinite(J)) return true;

    u0 = u0Candidate;
    for (std::size_t k = 0; k + 1 < N; ++k) warmStart[k] = us[k + 1];
    warmStart[N - 1] = us[N - 1];

#if CDS_ILQR_DEBUG
    {
        // How much of the horizon sits on the actuator bounds: a plan that is
        // saturated everywhere has no authority left to shape the trajectory.
        int pinnedStages = 0;
        for (std::size_t k = 0; k < N; ++k)
        {
            int pinned = 0;
            for (std::size_t a = 0; a < NU; ++a)
                if (us[k][a] <= lo[a] + 1e-9 || us[k][a] >= hi[a] - 1e-9) ++pinned;
            if (pinned == static_cast<int>(NU)) ++pinnedStages;
        }
        std::printf("[ilqr] done: J=%.6g |x|max=%.4g accepted=%d/%d allSaturatedStages=%d/%zu u0=[",
                    J, stateMax, acceptedCount, iterCount, pinnedStages, N);
        for (std::size_t a = 0; a < NU; ++a) std::printf("%s%.3f", a ? ", " : "", u0[a]);
        std::printf("]\n");
    }
#endif

    if (report != nullptr)
    {
        report->cost       = J;
        report->stateMax   = stateMax;
        report->iterations = iterCount;
        report->accepted   = acceptedCount;
    }
    return false;
}

}} // namespace CDS::control

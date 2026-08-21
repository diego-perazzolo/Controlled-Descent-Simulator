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
// File        : trans_disturbance_observer.hpp
// Description : Reusable translational disturbance observer -- the offset-free
//               wiring pattern shared by every vehicle, factored out of the
//               models so each one supplies only its physics, not the plumbing.
//               State s = [r(NP), v(NP), d(NP)] with the constant-disturbance
//               model
//                   r_dot = v,
//                   v_dot = a_known + Bd d,       (a_known: known force-free accel)
//                   d_dot = 0,                    (integrating states)
//               measuring the position r. It estimates the external-force
//               disturbance d so a controller can predict with it and reach zero
//               steady-state error. Everything here is model-AGNOSTIC: the only
//               physics is the coupling Bd = d(v_dot)/d(disturbance), which the
//               caller obtains FROM the physical model (e.g. a finite difference
//               of the generated, notebook-derived Dynamics w.r.t. the external
//               force) and passes to Build(); the kinematic r_dot = v and the
//               position measurement are definitional, not vehicle-specific. A
//               dropped measurement axis is handled as predict-only (its
//               innovation is zeroed). Built on the generic steady-state observer
//               (observer.hpp); fixed compile-time NP; no heap; header-only.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cstddef>

#include "observer.hpp"   // libs/estimate -- generic steady-state observer

namespace CDS { namespace estimate {

// NP = number of translational axes (3 for a free-flying vehicle). The observer
// carries 3*NP states [r, v, d] and measures the NP positions.
template <std::size_t NP>
class TransDisturbanceObserver
{
    public:

    static constexpr std::size_t NX = 3 * NP;   // [r(NP), v(NP), d(NP)]
    static constexpr std::size_t NY = NP;        // measured position

    TransDisturbanceObserver() : m_obs{}, m_x{}, m_seeded(false) {}

    // Assemble the structure from the disturbance-input coupling Bd (NP x NP,
    // taken from the physical model) and synthesise the steady-state gain from
    // the diagonal process/measurement covariances (position, velocity and
    // disturbance process noise; position measurement noise). Returns true on
    // error (propagated from the observer synthesis).
    bool Build(const Mat<NP, NP>& Bd,
               double qPos, double qVel, double qDist, double rPos,
               int maxIters = 100, double tol = 1e-13)
    {
        for (auto& r : m_obs.A) r.fill(0.0);
        for (auto& r : m_obs.B) r.fill(0.0);
        for (auto& r : m_obs.C) r.fill(0.0);
        for (std::size_t i = 0; i < NP; ++i)
        {
            m_obs.A[i][NP + i] = 1.0;                                  // r_dot = v
            for (std::size_t j = 0; j < NP; ++j)
                m_obs.A[NP + i][2 * NP + j] = Bd[i][j];                // v_dot += Bd d
            m_obs.B[NP + i][i] = 1.0;                                  // v_dot += a_known
            m_obs.C[i][i]      = 1.0;                                  // measure r
        }

        Mat<NX, NX> Qw{}; for (auto& r : Qw) r.fill(0.0);
        for (std::size_t i = 0; i < NP; ++i)
        {
            Qw[i][i]                 = qPos;
            Qw[NP + i][NP + i]       = qVel;
            Qw[2 * NP + i][2 * NP + i] = qDist;
        }
        Mat<NY, NY> Rv{}; for (auto& r : Rv) r.fill(0.0);
        for (std::size_t i = 0; i < NP; ++i) Rv[i][i] = rPos;

        m_seeded = false;
        return m_obs.Synthesize(Qw, Rv, maxIters, tol);
    }

    // Prime the estimate to a known position/velocity (disturbance starts at 0)
    // so the filter begins converged rather than transiently guessing.
    void Seed(const std::array<double, NP>& r0, const std::array<double, NP>& v0)
    {
        m_x.fill(0.0);
        for (std::size_t i = 0; i < NP; ++i) { m_x[i] = r0[i]; m_x[NP + i] = v0[i]; }
        m_seeded = true;
    }
    bool Seeded() const { return m_seeded; }
    void Unseed()       { m_seeded = false; }

    // Advance by dt with the known force-free acceleration aKnown and the measured
    // position meas (per-axis valid mask: an invalid axis is predict-only, its
    // innovation forced to zero). Non-allocating; safe on the tick path.
    void Step(const std::array<double, NP>& aKnown,
              const std::array<double, NP>& meas,
              const std::array<bool, NP>&   valid,
              double dt)
    {
        std::array<double, NY> y{};
        for (std::size_t a = 0; a < NP; ++a)
            y[a] = valid[a] ? meas[a] : m_x[a];   // C picks r, so predicted meas is m_x[a]
        m_x = m_obs.Step(m_x, aKnown, y, dt);
    }

    // Estimates.
    std::array<double, NP> Position()    const { return Slice(0); }
    std::array<double, NP> Velocity()    const { return Slice(NP); }
    std::array<double, NP> Disturbance() const { return Slice(2 * NP); }

    private:

    std::array<double, NP> Slice(std::size_t off) const
    {
        std::array<double, NP> o{};
        for (std::size_t i = 0; i < NP; ++i) o[i] = m_x[off + i];
        return o;
    }

    LinearObserver<NX, NP, NY> m_obs;   // B maps aKnown into the velocity rows
    std::array<double, NX>     m_x;     // [r_hat, v_hat, d_hat]
    bool                       m_seeded;
};

}} // namespace CDS::estimate

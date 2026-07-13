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
// File        : Poly4.cpp
// Description : Degree-4 polynomial sub-trajectory. The coefficients are derived
//               in closed form from the 5 boundary conditions (p0, v0, pF, vF,
//               aF) and the maneuver duration T. The derivation is performed
//               symbolically in the design notebook (section 6.b) and validated
//               numerically against the reference descent before being copied
//               here verbatim.
//
//               Boundary scheme:
//                 p(0)   = p0
//                 p'(0)  = v0
//                 p(T)   = pF
//                 p'(T)  = vF
//                 p''(T) = aF
//               The initial acceleration p''(0) is left free (it is the only
//               degree of freedom not constrained by a 5-coefficient polynomial).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "Poly4.hpp"

#include <cassert>

using namespace CDS;

Poly4::Poly4(const core_trajectoryPoly4Params_t params) :
    m_params(params)
{
    m_startTime = 0;
    m_endTime   = m_params.time_s; // seconds

    // Maneuver duration must be strictly positive: the closed form has T up to
    // the 4th power in the denominator, so T = 0 is mathematically singular.
    assert(m_params.time_s > 0.0 && "Poly4: time_s must be > 0");
    if (!(m_params.time_s > 0.0))
    {
        // Defensive degraded behaviour in release builds: hold the initial
        // position. GetReference will return a constant pose with zero
        // derivatives, which is benign for the controller.
        m_a_x = { m_params.initialPos[0], 0.0, 0.0, 0.0, 0.0 };
        m_a_y = { m_params.initialPos[1], 0.0, 0.0, 0.0, 0.0 };
        m_a_z = { m_params.initialPos[2], 0.0, 0.0, 0.0, 0.0 };
        m_a_psi = { m_params.initialYaw, 0.0, 0.0, 0.0, 0.0 };
        return;
    }

    const double T = m_params.time_s;

    m_a_x = ComputeAxisCoeffs(m_params.initialPos[0], m_params.initialVel[0],
                              m_params.finalPos[0],   m_params.finalVel[0],
                              m_params.finalAcc[0],   T);
    m_a_y = ComputeAxisCoeffs(m_params.initialPos[1], m_params.initialVel[1],
                              m_params.finalPos[1],   m_params.finalVel[1],
                              m_params.finalAcc[1],   T);
    m_a_z = ComputeAxisCoeffs(m_params.initialPos[2], m_params.initialVel[2],
                              m_params.finalPos[2],   m_params.finalVel[2],
                              m_params.finalAcc[2],   T);
    m_a_psi = ComputeAxisCoeffs(m_params.initialYaw, m_params.initialYawRate,
                              m_params.finalYaw,   m_params.finalYawRate,
                              m_params.finalYawAcc,   T);

}

Poly4::~Poly4()
{
}

std::array<double, 5> Poly4::ComputeAxisCoeffs(double p0, double v0,
                                               double pF, double vF,
                                               double aF, double T)
{
    // Closed-form solution of the 5x5 linear system arising from
    //   p(0) = p0,  p'(0) = v0,  p(T) = pF,  p'(T) = vF,  p''(T) = aF
    // for p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4.
    //
    // Derivation: notebook 01_model_derivation, section 6.b.
    //
    //   a0 = p0
    //   a1 = v0
    //   a2 = (  T^2*aF - 6*T*v0 - 6*T*vF - 12*p0 + 12*pF ) / (2*T^2)
    //   a3 = ( -T^2*aF + 3*T*v0 + 5*T*vF +  8*p0 -  8*pF ) /     T^3
    //   a4 = (  T^2*aF - 2*T*v0 - 4*T*vF -  6*p0 +  6*pF ) / (2*T^4)
    //
    // Below, dp := pF - p0, so the position terms become +12*dp / -8*dp / +6*dp.
    // Common subexpressions are hoisted to keep the arithmetic close to the
    // mathematical statement and avoid recomputing T^2/T^3/T^4.
    const double T2 = T * T;
    const double T3 = T2 * T;
    const double T4 = T2 * T2;

    const double dp = pF - p0;          // delta position
    const double Tv0 = T * v0;
    const double TvF = T * vF;
    const double T2aF = T2 * aF;

    std::array<double, 5> a;
    a[0] = p0;
    a[1] = v0;
    a[2] = (  T2aF - 6.0 * Tv0 - 6.0 * TvF + 12.0 * dp ) / (2.0 * T2);
    a[3] = ( -T2aF + 3.0 * Tv0 + 5.0 * TvF -  8.0 * dp ) /        T3;
    a[4] = (  T2aF - 2.0 * Tv0 - 4.0 * TvF +  6.0 * dp ) / (2.0 * T4);
    return a;
}

bool Poly4::GetReference(const core_coord_t& time, Reference_t& ref)
{
    // The trajectory factory passes a time already shifted into the local
    // [0, time_s] window, so no offsetting is performed here.
    const double t = time;

    // Horner-form evaluation of position and its first four derivatives.
    // For p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 :
    //   p    = a0 + t*(a1 + t*(a2 + t*(a3 + t*a4)))
    //   p'   = a1 + t*(2*a2 + t*(3*a3 + t*4*a4))
    //   p''  = 2*a2 + t*(6*a3 + t*12*a4)
    //   p''' = 6*a3 + 24*a4*t
    //   p'''' = 24*a4
    for (int i = 0; i < 3; ++i)
    {
        const std::array<double, 5>& a = (i == 0) ? m_a_x : (i == 1) ? m_a_y : m_a_z;

        ref.pos[i]  = a[0] + t * (a[1] + t * (a[2] + t * (a[3] + t * a[4])));
        ref.vel[i]  = a[1] + t * (2.0 * a[2] + t * (3.0 * a[3] + t * 4.0 * a[4]));
        ref.acc[i]  = 2.0 * a[2] + t * (6.0 * a[3] + t * 12.0 * a[4]);
        ref.jerk[i] = 6.0 * a[3] + 24.0 * a[4] * t;
        ref.snap[i] = 24.0 * a[4];
    }

    // Yaw
    ref.yaw = m_a_psi[0] + t * (m_a_psi[1] + t * (m_a_psi[2] + t * (m_a_psi[3] + t * m_a_psi[4])));
    ref.yawRate = m_a_psi[1] + t * (2.0 * m_a_psi[2] + t * (3.0 * m_a_psi[3] + t * 4.0 * m_a_psi[4]));
    ref.yawAcc = 2.0 * m_a_psi[2] + t * (6.0 * m_a_psi[3] + t * 12.0 * m_a_psi[4]);
    ref.yawJerk = 6.0 * m_a_psi[3] + 24.0 * m_a_psi[4] * t;
    ref.yawSnap = 24.0 * m_a_psi[4];


    return false;
}

bool Poly4::SetParameters(const std::map<std::string, core_coord_t>& params)
{
    return true; // TODO DP
    return false;
}

bool Poly4::GetParameters(std::map<std::string, core_coord_t>& params)
{
    return true; // TODO DP
    return false;
}

bool Poly4::SetParameter(const core_coord_t& p, size_t paramIdx)
{
    return true; // TODO DP
    return false;
}

bool Poly4::GetParameter(core_coord_t& p, size_t paramIdx)
{
    return true; // TODO DP
    return false;
}

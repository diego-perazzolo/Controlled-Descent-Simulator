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
// File        : lqr_tuner.hpp
// Description : Runtime LQR gain management for feedforward+LQR models. Holds the
//               tunable cost weights (Q, R) and the synthesised feedback gain,
//               and re-solves the gain from a model's *frozen* error dynamics
//               (A_e, B_e) via the hand-written CDS::control::lqr whenever the
//               weights change -- so the weights can be retuned at run time
//               without recomputing the linearisation. Domain-agnostic: it is
//               templated on the dimensions and duck-typed on the model, which
//               must expose the constants A_e[NX][NX], B_e[NX][NU],
//               Q_default[NX][NX], R_default[NU][NU], K_default[NU][NX] and a
//               SetGain(const double(&)[NU][NX]) method. Depends only on
//               lqr.hpp; includes no core or app headers, so it lives under
//               libs/. The two FF-LQR runtime models (Rocket, QuadRotor) share
//               this instead of duplicating the gain machinery.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include "lqr.hpp"

#include <cmath>
#include <cstddef>

namespace CDS { namespace control {

template <std::size_t NX, std::size_t NU>
class LqrGainTuner
{
public:
    LqrGainTuner() : m_Q{}, m_R{}, m_K{}, m_bridgeErr(0.0) {}

    // Load the model's baked default weights (call once before the first solve).
    template <class Model>
    void loadDefaults()
    {
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t j = 0; j < NX; ++j) m_Q[i][j] = Model::Q_default[i][j];
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t b = 0; b < NU; ++b) m_R[a][b] = Model::R_default[a][b];
    }

    void SetWeights(const double Q[NX][NX], const double R[NU][NU])
    {
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t j = 0; j < NX; ++j) m_Q[i][j] = Q[i][j];
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t b = 0; b < NU; ++b) m_R[a][b] = R[a][b];
    }

    void GetWeights(double Q[NX][NX], double R[NU][NU]) const
    {
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t j = 0; j < NX; ++j) Q[i][j] = m_Q[i][j];
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t b = 0; b < NU; ++b) R[a][b] = m_R[a][b];
    }

    void GetGain(double K[NU][NX]) const
    {
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t j = 0; j < NX; ++j) K[a][j] = m_K[a][j];
    }

    // Max deviation of the last synthesised gain from the model's baked K_default.
    // Meaningful as a bridge certificate only at the default weights (~0 there).
    double bridgeError() const { return m_bridgeErr; }

    // Re-solve the gain from the model's frozen (A_e, B_e) and the current (Q, R)
    // and install it via model.SetGain. Returns true on solver error, in which
    // case the previously installed gain is left untouched (bool-is-error rule).
    template <class Model>
    bool synthesize(Model& model)
    {
        Mat<NX, NX> A{}, Q{};
        Mat<NX, NU> B{};
        Mat<NU, NU> R{};
        Mat<NU, NX> K{};

        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t j = 0; j < NX; ++j) { A[i][j] = Model::A_e[i][j]; Q[i][j] = m_Q[i][j]; }
        for (std::size_t i = 0; i < NX; ++i)
            for (std::size_t a = 0; a < NU; ++a) B[i][a] = Model::B_e[i][a];
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t b = 0; b < NU; ++b) R[a][b] = m_R[a][b];

        if (lqr<NX, NU>(A, B, Q, R, K)) return true;

        double mx = 0.0, kbuf[NU][NX];
        for (std::size_t a = 0; a < NU; ++a)
            for (std::size_t j = 0; j < NX; ++j)
            {
                m_K[a][j] = kbuf[a][j] = K[a][j];
                const double d = std::fabs(K[a][j] - Model::K_default[a][j]);
                if (d > mx) mx = d;
            }
        m_bridgeErr = mx;
        model.SetGain(kbuf);
        return false;
    }

private:
    double m_Q[NX][NX];
    double m_R[NU][NU];
    double m_K[NU][NX];
    double m_bridgeErr;
};

}} // namespace CDS::control

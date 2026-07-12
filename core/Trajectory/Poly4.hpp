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
// File        : Poly4.hpp
// Description : Degree-4 polynomial sub-trajectory with parametric boundary
//               conditions (p0, v0, pF, vF, aF, T). Coefficients are computed
//               in closed form at construction time; GetReference is a pure
//               Horner evaluation on the cached per-axis coefficient arrays.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include <array>
#include "Trajectory.hpp"

namespace CDS
{
    class Poly4 : public Trajectory
    {

        public:

        Poly4(const core_trajectoryPoly4Params_t params);
        virtual ~Poly4() override;

        /* Virtual methods */
         /* Gets reference trajectory state at a time instant. Returns true on error */
        virtual bool GetReference(const core_coord_t&  time, Reference_t& ref) override;

        /* Set dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool SetParameters(const std::map<std::string, core_coord_t>& params) override;

        /* Get dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool GetParameters(std::map<std::string, core_coord_t>& params) override;

        /* Set trajectory parameter. Returns true on error */
        virtual bool SetParameter(const core_coord_t& p, size_t paramIdx) override;

        /* Get trajectory parameter. Returns true on error */
        virtual bool GetParameter(core_coord_t& p, size_t paramIdx) override;

        private:

        /*
         * Compute the 5 polynomial coefficients [a0, a1, a2, a3, a4] for a single
         * axis from the boundary conditions (p0, v0, pF, vF, aF) and the maneuver
         * duration T. Closed-form solution; see notebook section 6.b for the
         * derivation and the numerical validation against the reference descent.
         */
        static std::array<double, 5> ComputeAxisCoeffs(double p0, double v0,
                                                       double pF, double vF,
                                                       double aF, double T);

        core_trajectoryPoly4Params_t m_params;

        // Per-axis polynomial coefficients in ascending order of powers of t,
        // i.e. p_axis(t) = m_a_axis[0] + m_a_axis[1]*t + ... + m_a_axis[4]*t^4.
        std::array<double, 5> m_a_x;
        std::array<double, 5> m_a_y;
        std::array<double, 5> m_a_z;
        std::array<double, 5> m_a_psi;
    };
};

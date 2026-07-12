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
// File        : <filename.cpp>
// Description : <brief description of this file>
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "BaseModel.hpp"

namespace CDS
{
    class Rocket : public BaseModel
    {
        public:

        Rocket();

        virtual ~Rocket();
        virtual bool SetModelParams(const std::any& params) override;
        virtual bool SetTrajectoryManager(TrajectoryManager* pTrajectoryManager) override;
        virtual bool PerformIntegration(const core_stepParams_t& params) override;
        virtual bool GetState(core_state_t& state) override;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) override; 

        using StateVec = std::array<double, 15>;   // augmented state (12 + 3 integrals)
        using InputVec = std::array<double, 4>;    // [F1, T1, T2, T3]
        using RefVec   = std::array<double, 3>;    // position reference [x_ref, y_ref, z_ref, psi_ref]
        using TrackingErr = std::array<double, 4>;    // Tracking err w.r.t. [x_ref, y_ref, z_ref, psi_ref]
        using UserForces = std::array<double, 3>;    // User input forces [Fx, Fy, Fz]

        private:
        void* m_modelPtr;
        StateVec m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr m_trackingErr;
        UserForces m_userForces;
        double m_time;

    };
}
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
// File        : QuadRotor.hpp
// Description : Quadrotor runtime model (BaseModel derivative).
//               Wraps the generated CDS::Dynamics::QUADROTOR_FF_LQR_01
//               (quaternion 6-DOF + differential-flatness feedforward + LQR)
//               with an RK4 integrator and the core simulation interface.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include "BaseModel.hpp"

namespace CDS
{
    class QuadRotor : public BaseModel
    {
        public:

        QuadRotor();

        virtual ~QuadRotor();
        virtual bool SetModelParams(const std::any& params) override;
        virtual bool SetTrajectoryManager(TrajectoryManager* pTrajectoryManager) override;
        virtual bool PerformIntegration(const core_stepParams_t& params) override;
        virtual bool GetState(core_state_t& state) override;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) override;
        virtual bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds) override;


        // Augmented runtime state (13 physical + 4 integrators):
        //   [r(3), q(4, quaternion), v(3), omega(3, body rates), IntX, IntY, IntZ, IntPsi]
        using StateVec    = std::array<double, 17>;
        using InputVec    = std::array<double, 4>;   // [T1, T2, T3, T4] rotor thrusts
        using RefVec      = std::array<double, 3>;    // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;    // tracking err w.r.t. [x, y, z, yaw]
        using UserForces  = std::array<double, 3>;    // user input forces [Fx, Fy, Fz]

        private:
        void*              m_modelPtr;
        StateVec           m_state;
        TrajectoryManager* m_trajectoryManagerPtr = nullptr;
        TrackingErr        m_trackingErr;
        UserForces         m_userForces;
        double             m_time;

    };
}
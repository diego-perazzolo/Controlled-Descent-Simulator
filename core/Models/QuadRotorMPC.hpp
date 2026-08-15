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
// File        : QuadRotorMPC.hpp
// Description : Quadrotor runtime model (BaseModel derivative) driven by a
//               nonlinear Model-Predictive Controller. Wraps the generated
//               CDS::Dynamics::QUADROTOR_MPC_01 (quaternion 6-DOF prediction
//               model) with a control-limited iLQR/DDP solver -- folded into the
//               .cpp as private machinery -- and an RK4 integrator. Each tick it
//               samples the reference over the horizon, solves the MPC problem
//               (warm-started), applies the first command as a zero-order hold,
//               and integrates the plant by the measured wall-clock step.
//               Controller knobs (weights, horizon, dt) are hard-wired for now.
//               Derived in modeling/notebooks/dynamics_quadRotor_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>

#include "BaseModel.hpp"

namespace CDS
{
    class QuadRotorMPC : public BaseModel
    {
        public:

        QuadRotorMPC();

        virtual ~QuadRotorMPC();
        virtual bool SetModelParams(const std::any& params) override;
        virtual bool SetTrajectoryManager(TrajectoryManager* pTrajectoryManager) override;
        virtual bool PerformIntegration(const core_stepParams_t& params) override;
        virtual bool GetState(core_state_t& state) override;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) override;
        virtual bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds) override;

        // Physical runtime state (no LQR integrators):
        //   [r(3), q(4, quaternion), v(3), omega(3, body rates)]
        static constexpr std::size_t STATE_DIM = 13;
        static constexpr std::size_t INPUT_DIM = 4;    // [T1, T2, T3, T4] rotor thrusts
        static constexpr std::size_t HORIZON   = 40;   // MPC prediction horizon (compile-time)

        using StateVec    = std::array<double, STATE_DIM>;
        using InputVec    = std::array<double, INPUT_DIM>;
        using RefVec      = std::array<double, 3>;      // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;      // tracking err w.r.t. [x, y, z, yaw]
        using UserForces  = std::array<double, 3>;      // user input forces [Fx, Fy, Fz]

        private:
        void*              m_modelPtr;
        StateVec           m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr        m_trackingErr;
        UserForces         m_userForces;
        double             m_time;

        // Warm-start command sequence: the previous solve, kept and shifted so the
        // next tick starts a few iterations away from the answer.
        std::array<InputVec, HORIZON> m_warmStart;
        bool               m_seeded;

        // The MPC re-solves only at the control cadence (every DT_MPC of model
        // time); between solves the last command is held as a zero-order hold.
        // This keeps the expensive solve off most ticks, so a high tick rate does
        // not monopolise the system lock and the simulation degrades gracefully.
        InputVec           m_lastU0;
        double             m_lastSolveTime;
    };
}

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
// File        : RocketMPC.hpp
// Description : Rocket runtime model (BaseModel derivative) driven by a nonlinear
//               Model-Predictive Controller. Wraps the generated
//               CDS::Dynamics::ROCKET_MPC_01 (Euler-angle 6-DOF prediction model)
//               with the generic control-limited iLQR/DDP solver
//               (libs/control/ilqr.hpp) and an RK4 integrator. Each tick it
//               samples the reference over the horizon, solves the MPC problem
//               (warm-started), applies the first command as a zero-order hold,
//               and integrates the plant by the measured wall-clock step. This is
//               the Euler-angle sibling of QuadRotorMPC and is deliberately
//               simpler: the attitude error is a direct (heading-wrapped) angle
//               difference (no error-quaternion, no state projection), and the
//               command is the wrench [F1, T1, T2, T3] directly (no motor
//               allocation), with a per-input actuator box. Controller knobs
//               (cost weights, control step, iterations) are runtime-tunable
//               through the controller-parameter interface; the horizon N is
//               compile-time (fixed buffers) and exposed read-only.
//               Derived in modeling/notebooks/model/dynamics_rocket_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>

#include "BaseModel.hpp"
#include "controller_params.hpp"

namespace CDS
{
    class RocketMPC : public BaseModel
    {
        public:

        RocketMPC();

        virtual ~RocketMPC();
        virtual bool SetModelParams(const std::any& params) override;
        virtual bool SetTrajectoryManager(TrajectoryManager* pTrajectoryManager) override;
        virtual bool PerformIntegration(const core_stepParams_t& params) override;
        virtual bool GetState(core_state_t& state) override;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) override;
        virtual bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds) override;

        // Controller-parameter interface (cost weights, control step, iterations).
        bool GetControllerManifest(char* buf, std::size_t n) override;
        bool SetControllerParam(int id, double value) override;

        // Physical runtime state (Euler angles, no LQR integrators):
        //   [r(3), euler(3: alpha,beta,psi), v(3), euler_rate(3)]
        static constexpr std::size_t STATE_DIM = 12;
        static constexpr std::size_t INPUT_DIM = 4;    // [F1, T1, T2, T3] wrench
        static constexpr std::size_t HORIZON   = 40;   // MPC prediction horizon (compile-time)

        using StateVec    = std::array<double, STATE_DIM>;
        using InputVec    = std::array<double, INPUT_DIM>;
        using RefVec      = std::array<double, 3>;      // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;      // tracking err w.r.t. [x, y, z, yaw]
        using UserForces  = std::array<double, 3>;      // user input forces [Fx, Fy, Fz]

        private:
        void BuildParamTable();        // register the exposed MPC knobs

        void*              m_modelPtr;
        StateVec           m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr        m_trackingErr;
        UserForces         m_userForces;
        double             m_time;

        // Runtime-tunable controller knobs. The state cost is one Gauss-Newton
        // weight per block (position, attitude, velocity, body rate); the control
        // cost is a per-input weight (the rocket's actuators are heterogeneous --
        // thrust ~1e2 N vs torques ~1e0 N.m -- so a single scalar would let the
        // thrust dominate). w_term scales the terminal state cost; dt_mpc is the
        // control step; max_iters caps the iLQR iterations. HORIZON is compile-time.
        // Defaulted in the constructor to the values tuned in the notebook.
        double             m_wp, m_wq, m_wv, m_ww, m_wterm;
        double             m_rF1, m_rT1, m_rT2, m_rT3;
        double             m_dtMpc;
        int                m_maxIters;
        control::ParamTable<> m_params;

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

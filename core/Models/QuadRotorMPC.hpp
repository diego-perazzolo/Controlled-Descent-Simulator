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
//               Controller knobs (cost weights, control step, iterations) are
//               runtime-tunable through the controller-parameter interface; the
//               horizon N is runtime-tunable (fixed buffers sized to MAX_HORIZON).
//               Derived in modeling/notebooks/dynamics_quadRotor_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>

#include "BaseModel.hpp"
#include "param_table.hpp"                 // libs/param    -- tunable-parameter registry
#include "trans_disturbance_observer.hpp"  // libs/estimate -- reusable offset-free observer
#include "sensor_model.hpp"                // libs/sensor   -- position measurement corruptor

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

        // Tunable-parameter interface, split by domain: model (horizon),
        // controller (cost weights, control step, iterations), observer
        // (covariances + enable) and sensor (per-axis bias / noise / enable).
        bool GetModelManifest(char* buf, std::size_t n) override;
        bool SetModelParam(int id, double value) override;
        bool GetControllerManifest(char* buf, std::size_t n) override;
        bool SetControllerParam(int id, double value) override;
        bool GetObserverManifest(char* buf, std::size_t n) override;
        bool SetObserverParam(int id, double value) override;
        bool GetSensorManifest(char* buf, std::size_t n) override;
        bool SetSensorParam(int id, double value) override;

        // Physical runtime state (no LQR integrators):
        //   [r(3), q(4, quaternion), v(3), omega(3, body rates)]
        static constexpr std::size_t STATE_DIM = 13;
        static constexpr std::size_t INPUT_DIM = 4;    // [T1, T2, T3, T4] rotor thrusts
        static constexpr std::size_t MAX_HORIZON = 256; // fixed buffer capacity (max prediction horizon)

        using StateVec    = std::array<double, STATE_DIM>;
        using InputVec    = std::array<double, INPUT_DIM>;
        using RefVec      = std::array<double, 3>;      // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;      // tracking err w.r.t. [x, y, z, yaw]
        using UserForces  = std::array<double, 3>;      // user input forces [Fx, Fy, Fz]

        // Translational disturbance-observer size: the vehicle has 3 position
        // axes, so the sensor bank and the observer both work in 3 dimensions.
        static constexpr std::size_t POS_DIM = 3;

        // Offset-free observer feature toggle (opt-in; OFF by default so the
        // model is unchanged until switched on). Not part of the BaseModel
        // interface -- surfaced to the frontend later through the ext API.
        void SetObserverEnabled(bool on) { m_obsEnabled = on; }
        bool IsObserverEnabled() const   { return m_obsEnabled; }
        // Runtime access to the position sensor bank (per-axis noise / bias /
        // enable) so a dropped or noisy sensor can be configured live.
        sensor::SensorModel<POS_DIM>& PositionSensor() { return m_posSensor; }

        private:
        void BuildParamTable();        // register the exposed knobs into the 4 tables

        void*              m_modelPtr;
        StateVec           m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr        m_trackingErr;
        UserForces         m_userForces;
        double             m_time;

        // Runtime-tunable controller knobs (Gauss-Newton cost weights per block,
        // the control step DT_MPC, the iLQR iteration cap, and the active
        // prediction horizon N <= MAX_HORIZON). Defaulted in the constructor to
        // the values tuned in the notebook.
        double             m_wp, m_wq, m_wv, m_ww, m_wu, m_wterm;
        double             m_dtMpc;
        int                m_maxIters;
        std::size_t        m_horizon;   // active horizon N (1..MAX_HORIZON)
        param::ParamTable<> m_modelParams;       // structural knobs (horizon)
        param::ParamTable<> m_controllerParams;  // cost weights + solver
        param::ParamTable<> m_observerParams;    // observer covariances + enable
        param::ParamTable<> m_sensorParams;      // per-axis position sensor knobs

        // Warm-start command sequence: the previous solve, kept and shifted so the
        // next tick starts a few iterations away from the answer.
        std::array<InputVec, MAX_HORIZON> m_warmStart;
        bool               m_seeded;

        // The MPC re-solves only at the control cadence (every DT_MPC of model
        // time); between solves the last command is held as a zero-order hold.
        // This keeps the expensive solve off most ticks, so a high tick rate does
        // not monopolise the system lock and the simulation degrades gracefully.
        InputVec           m_lastU0;
        double             m_lastSolveTime;

        // ---- Offset-free disturbance observer (opt-in, OFF by default) --------
        // Estimates the external force disturbance from the (optionally corrupted)
        // position measurement so the MPC can predict with predForce = d_hat and
        // drive the steady-state tracking error to zero. Translational, Euclidean:
        // state [r(3), v(3), d(3)] with the constant-disturbance model d_dot = 0
        // (the integrating states that supply the missing integral action);
        // known input a_known(3) = the model's force-free acceleration; measure
        // r(3). When m_obsEnabled is false the model behaves exactly as before.
        // The disturbance-input coupling Bd is read FROM the generated model
        // (a finite difference of Dynamics w.r.t. the external force), so the
        // physics derives from the notebook-exported C++, not from hand code.
        void BuildObserver();   // read Bd from the model and synthesise the gain

        estimate::TransDisturbanceObserver<POS_DIM> m_obs;
        sensor::SensorModel<POS_DIM> m_posSensor;   // per-axis noise/bias/enable
        bool   m_obsEnabled;                        // feature toggle (default false)
        double m_obsQpos, m_obsQvel, m_obsQdist;    // process-noise covariance diag
        double m_obsRpos;                           // measurement-noise covariance
    };
}

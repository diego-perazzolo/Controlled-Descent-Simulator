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
#include "lqr_tuner.hpp"
#include "param_table.hpp"                 // libs/param    -- tunable-parameter registry
#include "trans_disturbance_observer.hpp"  // libs/estimate -- state estimate that feeds the LQR
#include "sensor_model.hpp"                // libs/sensor   -- position measurement corruptor

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

        // Runtime-tunable LQR weights. Changing them re-synthesises the feedback
        // gain from the (frozen) error dynamics via CDS::control::lqr; the
        // physical parameters and the linearisation are unaffected.
        void   SetWeights(const double Q[16][16], const double R[4][4]);
        void   GetWeights(double Q[16][16], double R[4][4]) const;
        void   GetGain(double K[4][16]) const;
        double GetGainBridgeError() const;   // max|runtime gain - baked K_default| at default weights

        // Tunable-parameter interface, split by domain: controller (Q/R diagonal
        // weights), observer (covariances + enable) and sensor (per-axis bias /
        // noise / enable). This model has no structural (model) knobs.
        bool GetControllerManifest(char* buf, std::size_t n) override;
        bool SetControllerParam(int id, double value) override;
        bool GetObserverManifest(char* buf, std::size_t n) override;
        bool SetObserverParam(int id, double value) override;
        bool GetSensorManifest(char* buf, std::size_t n) override;
        bool SetSensorParam(int id, double value) override;

        // Augmented runtime state (13 physical + 4 integrators):
        //   [r(3), q(4, quaternion), v(3), omega(3, body rates), IntX, IntY, IntZ, IntPsi]
        using StateVec    = std::array<double, 17>;
        using InputVec    = std::array<double, 4>;   // [T1, T2, T3, T4] rotor thrusts
        using RefVec      = std::array<double, 3>;    // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;    // tracking err w.r.t. [x, y, z, yaw]
        using UserForces  = std::array<double, 3>;    // user input forces [Fx, Fy, Fz]

        // Translational disturbance-observer size: 3 position axes.
        static constexpr std::size_t POS_DIM = 3;

        // State-estimator feature toggle (opt-in; OFF by default so the model is
        // unchanged until switched on). When on, the observer's filtered position/
        // velocity estimate replaces the true state in the LQR *feedback* term
        // (the integral action, baked into the generated dynamics, still uses the
        // true state -- so this is a partial, sensor-robustness integration, not
        // offset-free: the LQR already integrates for that). Not part of the
        // BaseModel interface -- surfaced to the frontend later through the ext API.
        void SetObserverEnabled(bool on) { m_obsEnabled = on; }
        bool IsObserverEnabled() const   { return m_obsEnabled; }
        // Runtime access to the position sensor bank (per-axis noise / bias /
        // enable) so a dropped or noisy sensor can be configured live.
        sensor::SensorModel<POS_DIM>& PositionSensor() { return m_posSensor; }

        private:
        bool RecomputeGain();          // (re)synthesise the LQR gain and install it (true on error)
        void BuildParamTable();        // register the exposed knobs into the tables
        void BuildObserver();          // read Bd from the model and synthesise the estimator gain

        void*              m_modelPtr;
        StateVec           m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr        m_trackingErr;
        UserForces         m_userForces;
        double             m_time;

        // Runtime LQR gain manager: tunable weights + synthesised gain (libs/control).
        control::LqrGainTuner<16, 4> m_lqr;
        param::ParamTable<>          m_controllerParams;  // LQR Q/R diagonal weights
        param::ParamTable<>          m_observerParams;    // observer covariances + enable
        param::ParamTable<>          m_sensorParams;      // per-axis position sensor knobs

        // ---- State estimator + sensor bank (opt-in, OFF by default) ----------
        // The translational observer supplies a filtered position/velocity to the
        // LQR feedback; the sensor bank corrupts (noise/bias) or drops the position
        // measurement it consumes. The disturbance state is kept only to keep the
        // estimate accurate under an unmodeled force -- it is NOT fed forward (the
        // LQR integral states reject constant disturbances themselves).
        estimate::TransDisturbanceObserver<POS_DIM> m_obs;
        sensor::SensorModel<POS_DIM> m_posSensor;   // per-axis noise/bias/enable
        bool   m_obsEnabled;                        // feature toggle (default false)
        double m_obsQpos, m_obsQvel, m_obsQdist;    // process-noise covariance diag
        double m_obsRpos;                           // measurement-noise covariance
    };
}
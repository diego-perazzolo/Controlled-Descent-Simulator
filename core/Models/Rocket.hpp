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
// File        : Rocket.hpp
// Description : Rocket model wrapper around the generated FF-LQR dynamics
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "BaseModel.hpp"
#include "lqr_tuner.hpp"
#include "controller_params.hpp"
#include "trans_disturbance_observer.hpp"  // libs/estimate -- state estimate that feeds the LQR
#include "sensor_model.hpp"                // libs/sensor   -- position measurement corruptor

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
        virtual bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds) override;

        // Runtime-tunable LQR weights. Changing them re-synthesises the feedback
        // gain from the (frozen) error dynamics via CDS::control::lqr; the
        // physical parameters and the linearisation are unaffected.
        void   SetWeights(const double Q[16][16], const double R[4][4]);
        void   GetWeights(double Q[16][16], double R[4][4]) const;
        void   GetGain(double K[4][16]) const;
        double GetGainBridgeError() const;   // max|runtime gain - baked K_default| at default weights

        // Controller-parameter interface (Q/R diagonal weights).
        bool GetControllerManifest(char* buf, std::size_t n) override;
        bool SetControllerParam(int id, double value) override;

        using StateVec = std::array<double, 16>;   // augmented state (12 + 4 integrals)
        using InputVec = std::array<double, 4>;    // [F1, T1, T2, T3]
        using RefVec   = std::array<double, 3>;    // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 4>;    // Tracking err w.r.t. [x_ref, y_ref, z_ref, psi_ref]
        using UserForces = std::array<double, 3>;    // User input forces [Fx, Fy, Fz]

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
        void BuildParamTable();        // register the exposed Q/R weights
        void BuildObserver();          // read Bd from the model and synthesise the estimator gain

        void* m_modelPtr;
        StateVec m_state;
        TrajectoryManager* m_trajectoryManagerPtr;
        TrackingErr m_trackingErr;
        UserForces m_userForces;
        double m_time;

        // Runtime LQR gain manager: tunable weights + synthesised gain (libs/control).
        control::LqrGainTuner<16, 4> m_lqr;
        control::ParamTable<>        m_params;   // exposed controller parameters (Q/R diagonal)

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
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

        // =============================================================================
        // PhysicsParams — physical and actuator parameters
        // Passed by const ref to avoid globals; can be serialized to/from JSON
        // for the frontend config page.
        // =============================================================================
        struct PhysicsParams_t{
            double m   = 10.0;          // mass (kg)
            double Ix  = 10.0 / 3.0;   // moment of inertia X (kg·m²)
            double Iy  = 10.0 / 3.0;   // moment of inertia Y (kg·m²)
            double Iz  = 1.0;           // moment of inertia Z (kg·m²)
            double g   = 9.81;          // gravity (m/s²)
            double c   = 10.0;          // lateral aerodynamic drag coefficient
            double cz  = 0.02;          // axial aerodynamic drag coefficient
            double F1_max = 700.0;      // maximum thrust (N)
        };

        Rocket();

        virtual ~Rocket();
        virtual bool SetModelParams(core_rocketParams_t& params) override;
        virtual bool SetTrajectory(Trajectory* pTrajectory) override;
        virtual bool PerformIntegration(core_stepParams_t& params) override;
        virtual bool GetState(core_state_t& state) override;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) override; 

        using StateVec = std::array<double, 15>;   // augmented state (12 + 3 integrals)
        using InputVec = std::array<double, 4>;    // [F1, T1, T2, T3]
        using RefVec   = std::array<double, 3>;    // position reference [x_ref, y_ref, z_ref]
        using TrackingErr = std::array<double, 3>;    // Tracking err w.r.t. [x_ref, y_ref, z_ref]
        using UserForces = std::array<double, 3>;    // User input forces [Fx, Fy, Fz]

        private:
        StateVec m_state;
        TrackingErr m_trackingErr;
        UserForces m_userForces;
        PhysicsParams_t m_physicsParams;
        double m_time;

    };
}
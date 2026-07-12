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
// File        : QuadRotor.cpp
// Description : Quadrotor 6-DOF dynamics (quaternion attitude)
//               + RK4 integrator (with quaternion renormalization)
//               + differential-flatness feedforward + LQR controller
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "QuadRotor.hpp"
#include <cmath>
#include <array>

#include "dynamics_quadrotor_ff_lqr_01.hpp"

// =============================================================================
// State vector layout (matches CDS::Dynamics::QUADROTOR_FF_LQR_01::StateName)
//
//  [0]  x        position X (m)              (world, Z-up)
//  [1]  y        position Y (m)
//  [2]  z        position Z (m)
//  [3]  qw       attitude quaternion w       (body->world, Hamilton, scalar-first)
//  [4]  qx       attitude quaternion x
//  [5]  qy       attitude quaternion y
//  [6]  qz       attitude quaternion z
//  [7]  vx       velocity X (m/s)            (world)
//  [8]  vy       velocity Y (m/s)
//  [9]  vz       velocity Z (m/s)
//  [10] wx       body angular rate p (rad/s) (body frame — NOT Euler rates)
//  [11] wy       body angular rate q
//  [12] wz       body angular rate r
//  [13] IntX     integral of position error X (LQR augmented state)
//  [14] IntY     integral of position error Y
//  [15] IntZ     integral of position error Z
//  [16] IntPsi   integral of yaw error       (heading tracking)
//
// Input vector u[4]:  [T1, T2, T3, T4]  per-rotor thrusts (N), ArduPilot QuadX order.
// =============================================================================

// State indexes
#define IDX_X        0
#define IDX_Y        1
#define IDX_Z        2
#define IDX_QW       3
#define IDX_QX       4
#define IDX_QY       5
#define IDX_QZ       6
#define IDX_VX       7
#define IDX_VY       8
#define IDX_VZ       9
#define IDX_WX      10
#define IDX_WY      11
#define IDX_WZ      12
#define IDX_INTX    13
#define IDX_INTY    14
#define IDX_INTZ    15
#define IDX_INTPSI  16

#define QUAD_STATE_DIM 17

namespace CDS {

// -----------------------------------------------------------------------------
// _normalize_quaternion()
// RK4 does not preserve the unit-norm constraint of the attitude quaternion
// (q_dot = 1/2 Omega(w) q keeps ||q|| constant only in continuous time), so the
// norm drifts a little each step. Renormalize after every integration step —
// this is the quaternion analogue of "nothing to do" for the rocket's Euler model.
// -----------------------------------------------------------------------------
static void _normalize_quaternion(QuadRotor::StateVec& x)
{
    const double n = std::sqrt(x[IDX_QW]*x[IDX_QW] + x[IDX_QX]*x[IDX_QX]
                             + x[IDX_QY]*x[IDX_QY] + x[IDX_QZ]*x[IDX_QZ]);
    if (n > 1e-12)
    {
        const double inv = 1.0 / n;
        x[IDX_QW] *= inv; x[IDX_QX] *= inv; x[IDX_QY] *= inv; x[IDX_QZ] *= inv;
    }
}

// =============================================================================
// rk4_step()
// Advances the state by one timestep dt using classic RK4.
// Reference is held constant over the step (ZOH); control is computed once at
// the current state. The quaternion is renormalized at the end of the step.
// =============================================================================
static bool rk4_step(void* pDynamics, QuadRotor::StateVec& x,
                     Reference_t& ref,
                     const QuadRotor::UserForces& userF,
                     const double dt)
{
    if (pDynamics == nullptr)
    {
        // ERR
        return true;
    }

    Dynamics::QUADROTOR_FF_LQR_01* pDyn =
        static_cast<Dynamics::QUADROTOR_FF_LQR_01*>(pDynamics);

    // Compute control at current state (held constant over the step)
    const QuadRotor::InputVec u = pDyn->ExecuteControl(x, ref);

    // Four RK4 slope evaluations
    const QuadRotor::StateVec k1 = pDyn->Dynamics(x, u, ref, userF);

    QuadRotor::StateVec x2{};
    for (size_t i = 0; i < QUAD_STATE_DIM; ++i) x2[i] = x[i] + k1[i] * dt * 0.5;
    const QuadRotor::StateVec k2 = pDyn->Dynamics(x2, u, ref, userF);

    QuadRotor::StateVec x3{};
    for (size_t i = 0; i < QUAD_STATE_DIM; ++i) x3[i] = x[i] + k2[i] * dt * 0.5;
    const QuadRotor::StateVec k3 = pDyn->Dynamics(x3, u, ref, userF);

    QuadRotor::StateVec x4{};
    for (size_t i = 0; i < QUAD_STATE_DIM; ++i) x4[i] = x[i] + k3[i] * dt;
    const QuadRotor::StateVec k4 = pDyn->Dynamics(x4, u, ref, userF);

    // Weighted sum
    for (size_t i = 0; i < QUAD_STATE_DIM; ++i)
        x[i] = x[i] + (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) * dt / 6.0;

    // Keep the attitude quaternion on the unit sphere
    _normalize_quaternion(x);

    return false;
}

static void _init_dynamicsState(Reference_t& ref, QuadRotor::StateVec& state)
{
    using SN = CDS::Dynamics::QUADROTOR_FF_LQR_01::StateName;

    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::X, ref.pos[0]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Y, ref.pos[1]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Z, ref.pos[2]);

    // Identity attitude (level hover); a yaw-aligned start would set q from ref.yaw
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qw, 1.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qx, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qy, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qz, 0.0);

    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::VX, ref.vel[0]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::VY, ref.vel[1]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::VZ, ref.vel[2]);

    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::WX, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::WY, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::WZ, 0.0);

    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::IntX,   0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::IntY,   0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::IntZ,   0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::IntPsi, 0.0);
}

QuadRotor::QuadRotor()
{
    // Init dynamic model
    m_modelPtr = new CDS::Dynamics::QUADROTOR_FF_LQR_01();

    // Initial state = 0, then set identity quaternion
    m_state.fill(0);
    m_state[IDX_QW] = 1.0;
    m_trackingErr.fill(0);
    m_userForces.fill(0);

    m_time = 0;
}

QuadRotor::~QuadRotor()
{
    if (m_modelPtr)
    {
        delete (CDS::Dynamics::QUADROTOR_FF_LQR_01*) m_modelPtr;
        m_modelPtr = nullptr;
    }
}

bool QuadRotor::SetModelParams(const std::any& params)
{
    auto dynamics = (Dynamics::QUADROTOR_FF_LQR_01*)m_modelPtr;

    if (dynamics == nullptr || params.type() != typeid(core_quadRotorParams_t&))
    {
        // Err
        return true;
    }

    const auto& p = std::any_cast<const core_quadRotorParams_t&>(params);
    using PN = Dynamics::QUADROTOR_FF_LQR_01::ParamName;
    dynamics->SetParam(PN::Mass,      p.m);
    dynamics->SetParam(PN::Ix,        p.Ix);
    dynamics->SetParam(PN::Iy,        p.Iy);
    dynamics->SetParam(PN::Iz,        p.Iz);
    dynamics->SetParam(PN::Gravity,   p.g);
    dynamics->SetParam(PN::DragLat,   p.c);
    dynamics->SetParam(PN::DragAx,    p.cz);
    dynamics->SetParam(PN::KThrust,   p.kT);
    dynamics->SetParam(PN::KTorque,   p.kQ);
    dynamics->SetParam(PN::Arm,       p.L);
    dynamics->SetParam(PN::IRotor,    p.Irot);
    dynamics->SetParam(PN::ThrustMax, p.Fm_max);
    dynamics->SetParam(PN::ThrustMin, p.Fm_min);

    return false;
}

bool QuadRotor::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
{
    Reference_t ref;

    if (pTrajectoryManager == nullptr || pTrajectoryManager->GetReference(m_time, ref))
    {
        // Error
        return true;
    }

    m_trajectoryManagerPtr = pTrajectoryManager;
    _init_dynamicsState(ref, m_state);

    return false;
}

bool QuadRotor::PerformIntegration(const core_stepParams_t& params)
{
    // Getting reference setpoints from Trajectory
    Reference_t ref;
    if (m_trajectoryManagerPtr == nullptr || m_trajectoryManagerPtr->GetReference(m_time, ref))
    {
        // ERR
        return true;
    }

    // Compute tracking errors (position)
    m_trackingErr[0] = ref.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref.pos[2] - m_state[IDX_Z];

    // User forces
    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;

    // Runge Kutta 4 (control is computed inside, at the current state)
    if (rk4_step(m_modelPtr, m_state, ref, m_userForces, params.timestep))
    {
        // Err
        return true;
    }

    m_time += params.timestep;

    return false;
}

bool QuadRotor::GetState(core_state_t& state)
{
    // Velocities (world)
    state.x_dot = m_state[IDX_VX];
    state.y_dot = m_state[IDX_VY];
    state.z_dot = m_state[IDX_VZ];
    // Position (world)
    state.x = m_state[IDX_X];
    state.y = m_state[IDX_Y];
    state.z = m_state[IDX_Z];

    // Angular velocity: body rates omega (NOTE: these are body-frame rates, not
    // Euler-angle rates — they differ away from hover by the W(phi,theta) map).
    state.roll_dot  = m_state[IDX_WX];
    state.pitch_dot = m_state[IDX_WY];
    state.yaw_dot   = m_state[IDX_WZ];

    // Attitude: quaternion -> aerospace Z-Y-X Euler angles (visualization only)
    const double qw = m_state[IDX_QW], qx = m_state[IDX_QX];
    const double qy = m_state[IDX_QY], qz = m_state[IDX_QZ];
    double sinp = 2.0 * (qw*qy - qz*qx);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    state.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
    state.pitch = std::asin(sinp);
    state.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));

    return false;
}

bool QuadRotor::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x = m_trackingErr[0];
    tErrors.y = m_trackingErr[1];
    tErrors.z = m_trackingErr[2];
    return false;
}

} // namespace CDS

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
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"
#include "rk4.hpp"
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

static const auto logger = cds_log::registry().module("Quadrotor");
static const auto profile = cds_profile::registry().module("Quadrotor");

// ----------------------------------------------------------------------------
// Data recorder (black-box wide CSV, server-side). Channel 0 is t_sim; then the
// full 17-state (quaternion attitude + LQR integrators), the 4 rotor thrusts,
// the position/heading/velocity reference, the tracking error and the user
// forces. See the row-alignment note in PerformIntegration.
// ----------------------------------------------------------------------------
static cds_record::Recorder<double, 36, 4096> recorder("Quadrotor", {{
    "t_sim",
    "x", "y", "z", "qw", "qx", "qy", "qz",
    "vx", "vy", "vz", "wx", "wy", "wz",
    "IntX", "IntY", "IntZ", "IntPsi",
    "T1", "T2", "T3", "T4",
    "ref_x", "ref_y", "ref_z", "ref_yaw", "ref_vx", "ref_vy", "ref_vz",
    "e_x", "e_y", "e_z", "e_yaw",
    "uf_x", "uf_y", "uf_z",
}});

// -----------------------------------------------------------------------------
// _normalize_quaternion()
// RK4 does not preserve the unit-norm constraint of the attitude quaternion
// (q_dot = 1/2 Omega(w) q keeps ||q|| constant only in continuous time), so the
// norm drifts a little each step. Renormalize after every integration step
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

static void _init_dynamicsState(Reference_t& ref, QuadRotor::StateVec& state)
{
    using SN = CDS::Dynamics::QUADROTOR_FF_LQR_01::StateName;

    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::X, ref.pos[0]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Y, ref.pos[1]);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Z, ref.pos[2]);

    // Initial attitude: all zero angles except heading: ref.yaw  ->  pure yaw quaternion
    //   q = [cos(psi/2), 0, 0, sin(psi/2)]  (rotation about world Z)
    const double half_psi = 0.5 * ref.yaw;
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qw, std::cos(half_psi));
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qx, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qy, 0.0);
    Dynamics::QUADROTOR_FF_LQR_01::SetState(state, SN::Qz, std::sin(half_psi));

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

    m_trajectoryManagerPtr = nullptr;

    m_time = 0;

    // LQR weights default to the model's baked Q/R; synthesise the gain at runtime
    // from the frozen error dynamics (rather than using the baked K literal) and
    // certify it reproduces the notebook gain (bridge deviation ~0).
    m_lqr.loadDefaults<Dynamics::QUADROTOR_FF_LQR_01>();
    RecomputeGain();
    if (m_lqr.bridgeError() > 1e-6)
        CDS_LOG_ERROR(logger, "runtime LQR gain deviates from baked K_default by {}", m_lqr.bridgeError());
    BuildParamTable();

    recorder.activateAsModel(); // this model owns the model data recorder while it lives
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
        CDS_LOG_ERROR(logger, "Model not initialized or wrong params type");
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

    // Recorder run metadata: full model parameters (trajectory added in
    // SetTrajectoryManager, the last setup step).
    recorder.clearMeta();
    recorder.addMeta("model", "Quadrotor (FF-LQR-01)");
    recorder.addMeta("mass_kg", p.m);
    recorder.addMeta("Ix_kgm2", p.Ix);
    recorder.addMeta("Iy_kgm2", p.Iy);
    recorder.addMeta("Iz_kgm2", p.Iz);
    recorder.addMeta("gravity_ms2", p.g);
    recorder.addMeta("drag_lateral", p.c);
    recorder.addMeta("drag_axial", p.cz);
    recorder.addMeta("kThrust", p.kT);
    recorder.addMeta("kTorque", p.kQ);
    recorder.addMeta("arm_m", p.L);
    recorder.addMeta("rotor_inertia_kgm2", p.Irot);
    recorder.addMeta("motor_thrust_max_N", p.Fm_max);
    recorder.addMeta("motor_thrust_min_N", p.Fm_min);

    return false;
}

bool QuadRotor::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
{
    Reference_t ref;

    if (pTrajectoryManager == nullptr || pTrajectoryManager->GetReference(m_time, ref))
    {
        CDS_LOG_ERROR(logger, "Trajectory not initialized");
        return true;
    }

    m_trajectoryManagerPtr = pTrajectoryManager;
    _init_dynamicsState(ref, m_state);

    // Recorder run metadata: trajectory context (start setpoint + altitude span).
    recorder.addMeta("traj_start_x_m", ref.pos[0]);
    recorder.addMeta("traj_start_y_m", ref.pos[1]);
    recorder.addMeta("traj_start_z_m", ref.pos[2]);
    recorder.addMeta("traj_start_yaw_rad", ref.yaw);
    core_coord_t altRange = 0.0;
    if (!pTrajectoryManager->GetAltitudeRange(altRange))
        recorder.addMeta("traj_altitude_range_m", altRange);

    return false;
}

bool QuadRotor::PerformIntegration(const core_stepParams_t& params)
{
    // Getting reference setpoints from Trajectory
    Reference_t ref;
    if (m_trajectoryManagerPtr == nullptr || 
        m_trajectoryManagerPtr->GetReference(m_time, ref))
    {
        CDS_LOG_ERROR(logger, "Cannot get trajectory reference");
        return true;
    }

    // Compute tracking errors (position + heading; the LQR carries a yaw integrator)
    m_trackingErr[0] = ref.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref.pos[2] - m_state[IDX_Z];

    const double qw = m_state[IDX_QW], qx = m_state[IDX_QX];
    const double qy = m_state[IDX_QY], qz = m_state[IDX_QZ];
    const double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    double eyaw = ref.yaw - yaw;                       // wrap to [-pi, pi]
    eyaw = std::atan2(std::sin(eyaw), std::cos(eyaw));
    m_trackingErr[3] = eyaw;

    // User forces
    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;

    auto pDyn = static_cast<Dynamics::QUADROTOR_FF_LQR_01*>(m_modelPtr);
    if (pDyn == nullptr)
    {
        CDS_LOG_ERROR(logger, "Cannot perform model integration");
        return true;
    }

    // Compute the control once at the current state; it is held constant (ZOH)
    // over the RK4 step, so it is captured by the derivative closure below.
    QuadRotor::InputVec uApplied{};
    {
        CDS_PROFILE(profile, "Execute control");
        uApplied = pDyn->ExecuteControl(m_state, ref);
    }

    // Runge Kutta 4 (generic fixed-control step; reference held constant, ZOH),
    // then renormalize the attitude quaternion (RK4 does not preserve ||q||=1).
    {
        CDS_PROFILE(profile, "RK4 integration");
        m_state = integrate::rk4_step<QUAD_STATE_DIM>(m_state, params.timestep,
            [&](const QuadRotor::StateVec& s) { return pDyn->Dynamics(s, uApplied, ref, m_userForces); });
        _normalize_quaternion(m_state);
    }

    m_time += params.timestep;

    // Data recorder: one wide row per tick. Row alignment: the state is
    // post-integration at t_sim; uApplied is the command held over the step;
    // the reference and tracking error are sampled at the step start (they lead
    // the state by one dt — negligible at the simulation timestep).
#if CDS_RECORD_ENABLED
    {
        const std::array<double, 36> row{{
            m_time,
            m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z],
            m_state[IDX_QW], m_state[IDX_QX], m_state[IDX_QY], m_state[IDX_QZ],
            m_state[IDX_VX], m_state[IDX_VY], m_state[IDX_VZ],
            m_state[IDX_WX], m_state[IDX_WY], m_state[IDX_WZ],
            m_state[IDX_INTX], m_state[IDX_INTY], m_state[IDX_INTZ], m_state[IDX_INTPSI],
            uApplied[0], uApplied[1], uApplied[2], uApplied[3],
            ref.pos[0], ref.pos[1], ref.pos[2], ref.yaw, ref.vel[0], ref.vel[1], ref.vel[2],
            m_trackingErr[0], m_trackingErr[1], m_trackingErr[2], m_trackingErr[3],
            m_userForces[0], m_userForces[1], m_userForces[2],
        }};
        recorder.record(row);
    }
#endif

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
    tErrors.x   = m_trackingErr[0];
    tErrors.y   = m_trackingErr[1];
    tErrors.z   = m_trackingErr[2];
    tErrors.yaw = m_trackingErr[3];
    return false;
}

bool QuadRotor::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
{
    currentTimeSeconds = m_time;
    return false;
}

// -----------------------------------------------------------------------------
// LQR gain synthesis (runtime). RecomputeGain re-solves the CARE from the frozen
// error dynamics and the current weights and installs the gain into the
// generated model; SetWeights retunes and re-synthesises. The physical model
// parameters are untouched.
// -----------------------------------------------------------------------------
bool QuadRotor::RecomputeGain()
{
    auto pDyn = static_cast<Dynamics::QUADROTOR_FF_LQR_01*>(m_modelPtr);
    if (pDyn == nullptr) return true;
    if (m_lqr.synthesize(*pDyn))
    {
        CDS_LOG_ERROR(logger, "LQR gain synthesis failed; keeping previous gain");
        return true;
    }
    return false;
}

void QuadRotor::SetWeights(const double Q[16][16], const double R[4][4])
{
    m_lqr.SetWeights(Q, R);
    RecomputeGain();
}

void QuadRotor::GetWeights(double Q[16][16], double R[4][4]) const { m_lqr.GetWeights(Q, R); }
void QuadRotor::GetGain(double K[4][16]) const                    { m_lqr.GetGain(K); }
double QuadRotor::GetGainBridgeError() const                      { return m_lqr.bridgeError(); }

// ---- exposed controller parameters: the LQR cost diagonal (Q, R) -------------
// Semantic labels for the 16-state error vector and the 4 inputs -- the anchor
// that ties each parameter id (its manifest row) to a physical meaning.
namespace {
const char* const QUAD_ERR_LABELS[16] = {
    "x", "y", "z", "roll", "pitch", "yaw", "vx", "vy", "vz",
    "p", "q", "r", "int_x", "int_y", "int_z", "int_yaw" };
const char* const QUAD_IN_LABELS[4] = { "T1", "T2", "T3", "T4" };
} // namespace

void QuadRotor::BuildParamTable()
{
    m_params.clear();
    for (std::size_t i = 0; i < 16; ++i)
        m_params.add("Q", QUAD_ERR_LABELS[i], true,
                     [this, i] { return m_lqr.qDiag(i); },
                     [this, i](double v) { if (v < 0.0) return true;  m_lqr.setQDiag(i, v); return RecomputeGain(); });
    for (std::size_t a = 0; a < 4; ++a)
        m_params.add("R", QUAD_IN_LABELS[a], true,
                     [this, a] { return m_lqr.rDiag(a); },
                     [this, a](double v) { if (v <= 0.0) return true; m_lqr.setRDiag(a, v); return RecomputeGain(); });
}

bool QuadRotor::GetControllerManifest(char* buf, std::size_t n)
{
    m_params.buildManifest(buf, n);
    return false;
}

bool QuadRotor::SetControllerParam(int id, double value) { return m_params.set(id, value); }

} // namespace CDS

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
// File        : Rocket.cpp
// Description : 6 DOF rocket dynamics — equations of motion
//               + RK4 integrator + LQR controller
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "Rocket.hpp"
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"
#include "rk4.hpp"
#include "estimator_params.hpp"

#include <cmath>
#include <array>

#include "dynamics_rocket_ff_lqr_01.hpp" // TODO DP: create base dyn model

// =============================================================================
// State vector layout (matches: ode_sys1 / sol1)
//
//  [0]  x          position X (m)
//  [1]  y          position Y (m)
//  [2]  z          position Z (m)
//  [3]  alpha      Euler angle α — pitch around Y (rad)
//  [4]  beta       Euler angle β — roll  around X (rad)
//  [5]  psi        Euler angle ψ — yaw   around Z (rad)
//  [6]  x_dot      velocity X (m/s)
//  [7]  y_dot      velocity Y (m/s)
//  [8]  z_dot      velocity Z (m/s)
//  [9]  alpha_dot  angular rate α (rad/s)
//  [10] beta_dot   angular rate β (rad/s)
//  [11] psi_dot    angular rate ψ (rad/s)
//  [12] IntX       integral of position error X (for LQR augmented state)
//  [13] IntY       integral of position error Y
//  [14] IntZ       integral of position error Z
//  [15] IntPsi     integral of yaw error ψ
//
// Input vector u[4]:
//  [0]  F1   main thrust (N)
//  [1]  T1   torque about body Y (N·m) — drives alpha (pitch)
//  [2]  T2   torque about body X (N·m) — drives beta
//  [3]  T3   torque about body Z (N·m) — drives psi (roll about thrust axis)
// =============================================================================

// State indexes
#define IDX_X          0
#define IDX_Y          1
#define IDX_Z          2
#define IDX_ALPHA      3
#define IDX_BETA       4
#define IDX_PSI        5
#define IDX_XDOT       6
#define IDX_YDOT       7
#define IDX_ZDOT       8
#define IDX_ALPHADOT   9
#define IDX_BETADOT   10
#define IDX_PSIDOT    11
#define IDX_INTX      12
#define IDX_INTY      13
#define IDX_INTZ      14
#define IDX_INTPSI    15

namespace CDS {

static const auto logger = cds_log::registry().module("Rocket");
static const auto profile = cds_profile::registry().module("Rocket");

// ----------------------------------------------------------------------------
// Data recorder (black-box wide CSV, server-side). Channel 0 is t_sim; then the
// full 16-state, the 4 control inputs, the position/heading/velocity reference,
// the tracking error and the user/disturbance forces. See the row-alignment note
// in PerformIntegration. A single active model records at a time.
// ----------------------------------------------------------------------------
static cds_record::Recorder<double, 35, 4096> recorder("Rocket", {{
    "t_sim",
    "x", "y", "z", "alpha", "beta", "psi",
    "x_dot", "y_dot", "z_dot", "alpha_dot", "beta_dot", "psi_dot",
    "IntX", "IntY", "IntZ", "IntPsi",
    "F1", "T1", "T2", "T3",
    "ref_x", "ref_y", "ref_z", "ref_yaw", "ref_vx", "ref_vy", "ref_vz",
    "e_x", "e_y", "e_z", "e_yaw",
    "uf_x", "uf_y", "uf_z",
}});

static void _init_dynamicsState(Reference_t& ref, Rocket::StateVec& state)
{
    using SN = CDS::Dynamics::ROCKET_FF_LQR_01::StateName;

    // TODO DP: add initial values for angles aswell
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::X,        ref.pos[0]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::Y,         ref.pos[1]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::Z,        ref.pos[2]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::Alpha,      0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::Beta,       0.0);
    // Initial heading = ref.yaw so the yaw tracking error starts at zero.
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::Psi,        ref.yaw);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::XDot,       ref.vel[0]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::YDot,       ref.vel[1]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::ZDot,     ref.vel[2]);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::AlphaDot,   0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::BetaDot,    0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::PsiDot,     0.0);

    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::IntX,       0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::IntY,       0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::IntZ,       0.0);
    Dynamics::ROCKET_FF_LQR_01::SetState(state, SN::IntPsi,     0.0);

}

Rocket::Rocket()
{
    // Init dynamic model 
    m_modelPtr = new CDS::Dynamics::ROCKET_FF_LQR_01();

    // Initial state = 0;
    m_state.fill(0);
    m_trackingErr.fill(0);
    m_userForces.fill(0);

    m_trajectoryManagerPtr = nullptr;

    m_time = 0;

    // LQR weights default to the model's baked Q/R; synthesise the gain at runtime
    // from the frozen error dynamics (rather than using the baked K literal) and
    // certify it reproduces the notebook gain (bridge deviation ~0).
    m_lqr.loadDefaults<Dynamics::ROCKET_FF_LQR_01>();
    RecomputeGain();
    if (m_lqr.bridgeError() > 1e-6)
        CDS_LOG_ERROR(logger, "runtime LQR gain deviates from baked K_default by {}", m_lqr.bridgeError());
    BuildParamTable();

    // State estimator: OFF by default (opt-in). Gain synthesised in
    // SetModelParams, once the model is parameterised.
    m_obsEnabled = false;
    m_obsQpos = 1.0e-4; m_obsQvel = 1.0e-2; m_obsQdist = 25.0; m_obsRpos = 1.0e-4;

    recorder.activateAsModel(); // this model owns the model data recorder while it lives
}

// Synthesise the translational state estimator. The only physical input is the
// disturbance-input coupling Bd = d(v_dot)/d(F_ext), read FROM the generated
// model by finite-differencing Dynamics w.r.t. the external force (additive, so
// state/input-independent -- evaluated once at the upright hover). The estimate
// feeds the LQR feedback; the disturbance state keeps it accurate under a force.
void Rocket::BuildObserver()
{
    auto dyn = (Dynamics::ROCKET_FF_LQR_01*) m_modelPtr;
    if (dyn == nullptr) return;

    using PN = Dynamics::ROCKET_FF_LQR_01::ParamName;
    const double mg = dyn->GetParam(PN::Mass) * dyn->GetParam(PN::Gravity);
    StateVec x{}; x.fill(0.0);            // upright, at rest, at the origin
    const InputVec u{{mg, 0.0, 0.0, 0.0}};
    Reference_t ref{};                    // ref only affects the integral-state rows

    const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
    const StateVec d0 = dyn->Dynamics(x, u, ref, zeroF);
    control::Mat<POS_DIM, POS_DIM> Bd{}; for (auto& r : Bd) r.fill(0.0);
    for (std::size_t j = 0; j < POS_DIM; ++j)
    {
        std::array<double,3> Fe{{0.0, 0.0, 0.0}}; Fe[j] = 1.0;   // unit external force on axis j
        const StateVec dj = dyn->Dynamics(x, u, ref, Fe);
        Bd[0][j] = dj[IDX_XDOT] - d0[IDX_XDOT];
        Bd[1][j] = dj[IDX_YDOT] - d0[IDX_YDOT];
        Bd[2][j] = dj[IDX_ZDOT] - d0[IDX_ZDOT];
    }

    if (m_obs.Build(Bd, m_obsQpos, m_obsQvel, m_obsQdist, m_obsRpos))
        CDS_LOG_ERROR(logger, "State-estimator synthesis failed");
}

Rocket::~Rocket()
{
    if(m_modelPtr)
    {
        delete (CDS::Dynamics::ROCKET_FF_LQR_01*) m_modelPtr;
        m_modelPtr = nullptr;
    }
}

bool Rocket::SetModelParams(const std::any& params)
{
    auto dynamics = (Dynamics::ROCKET_FF_LQR_01*)m_modelPtr;

    if(dynamics == nullptr || params.type() != typeid(core_rocketParams_t&))
    {
        CDS_LOG_ERROR(logger, "Cannot set model params");
        return true;
    }

    const auto& p = std::any_cast<const core_rocketParams_t&>(params);
    using PN = Dynamics::ROCKET_FF_LQR_01::ParamName;
    dynamics->SetParam(PN::Mass, p.m);
    dynamics->SetParam(PN::Ix, p.Ix);
    dynamics->SetParam(PN::Iy, p.Iy);
    dynamics->SetParam(PN::Iz, p.Iz);
    dynamics->SetParam(PN::Gravity, p.g);
    dynamics->SetParam(PN::DragLateral, p.c);
    dynamics->SetParam(PN::DragAxial, p.cz);
    dynamics->SetParam(PN::ThrustMax, p.F1_max);
    dynamics->SetParam(PN::ThrustMin, p.F1_min);
    dynamics->SetParam(PN::TorqueXMax, p.T1_max);
    dynamics->SetParam(PN::TorqueXMin, p.T1_min);
    dynamics->SetParam(PN::TorqueYMax, p.T2_max);
    dynamics->SetParam(PN::TorqueYMin, p.T2_min);
    dynamics->SetParam(PN::TorqueZMax, p.T3_max);
    dynamics->SetParam(PN::TorqueZMin, p.T3_min);

    // Synthesise the state estimator now the model is parameterised.
    BuildObserver();

    // Recorder run metadata: full model parameters (the trajectory is added in
    // SetTrajectoryManager, the last setup step, so both end up in the header).
    recorder.clearMeta();
    recorder.addMeta("model", "Rocket (FF-LQR-01)");
    recorder.addMeta("mass_kg", p.m);
    recorder.addMeta("Ix_kgm2", p.Ix);
    recorder.addMeta("Iy_kgm2", p.Iy);
    recorder.addMeta("Iz_kgm2", p.Iz);
    recorder.addMeta("gravity_ms2", p.g);
    recorder.addMeta("drag_lateral", p.c);
    recorder.addMeta("drag_axial", p.cz);
    recorder.addMeta("F1_max_N", p.F1_max);
    recorder.addMeta("F1_min_N", p.F1_min);
    recorder.addMeta("T1_max_Nm", p.T1_max);
    recorder.addMeta("T1_min_Nm", p.T1_min);
    recorder.addMeta("T2_max_Nm", p.T2_max);
    recorder.addMeta("T2_min_Nm", p.T2_min);
    recorder.addMeta("T3_max_Nm", p.T3_max);
    recorder.addMeta("T3_min_Nm", p.T3_min);

    return false;
}
bool Rocket::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
{
    Reference_t ref;

    if(pTrajectoryManager == nullptr || pTrajectoryManager->GetReference(m_time, ref))
    {
        CDS_LOG_ERROR(logger, "Trajectory error");
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


bool Rocket::PerformIntegration(const core_stepParams_t& params)
{
    // Getting reference setpoints from Trajectory
    Reference_t ref;
    if(m_trajectoryManagerPtr == nullptr || m_trajectoryManagerPtr->GetReference(m_time, ref))
    {
        CDS_LOG_ERROR(logger, "Trajectory error");
        return true;
    }

    // Compute tracking errors
    m_trackingErr[0] = ref.pos[0] - m_state[IDX_X]; // Position X
    m_trackingErr[1] = ref.pos[1] - m_state[IDX_Y]; // Position Y
    m_trackingErr[2] = ref.pos[2] - m_state[IDX_Z]; // Position Z
    {                                               // Heading (wrapped to [-pi, pi])
        const double eyaw = ref.yaw - m_state[IDX_PSI];
        m_trackingErr[3] = std::atan2(std::sin(eyaw), std::cos(eyaw));
    }

    // User forces
    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;

#if 0 // Debug logging of the first 30 integration steps
    auto u = ((Dynamics::ROCKET_FF_LQR_01*)m_modelPtr)->ExecuteControl(m_state, ref);

    if (m_time < 30 * params.timestep) {
        using SN = CDS::Dynamics::ROCKET_FF_LQR_01::StateName;
        std::printf("=== t=%.6f  dt=%.6f ===\n", m_time, params.timestep);
        std::printf("  STATE BEFORE:  pos=(%.4f, %.4f, %.4f)  ang(rad)=(%.6f, %.6f, %.6f)  "
                    "angvel(rad/s)=(%.6f, %.6f, %.6f)\n",
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::X),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::Y),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::Z),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::Alpha),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::Beta),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::Psi),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::AlphaDot),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::BetaDot),
            Dynamics::ROCKET_FF_LQR_01::GetState(m_state, SN::PsiDot));
        std::printf("  REF: pos=(%.4f, %.4f, %.4f)  acc=(%.4f, %.4f, %.4f)\n",
            ref.pos[0], ref.pos[1], ref.pos[2],
            ref.acc[0], ref.acc[1], ref.acc[2]);
        std::printf("  CONTROL: F1=%.3f  T1=%.6f  T2=%.6f  T3=%.6f\n",
            u[0], u[1], u[2], u[3]);
    }
#endif

    auto pDyn = static_cast<Dynamics::ROCKET_FF_LQR_01*>(m_modelPtr);
    if (pDyn == nullptr)
    {
        CDS_LOG_ERROR(logger, "Cannot integrate model");
        return true;
    }

    // Compute the control once at the current state; it is held constant (ZOH)
    // over the RK4 step, so it is captured by the derivative closure below. When
    // the estimator is on, the LQR *feedback* reads the observer's filtered
    // position/velocity in place of the true state (the integral states, evolved
    // inside the generated dynamics, still use the true state -- offset-free stays
    // with them). With the estimator off this is the former true-state feedback.
    Rocket::StateVec stateForControl = m_state;
    if (m_obsEnabled)
    {
        if (m_obs.Seeded())
        {
            const auto rHat = m_obs.Position();
            const auto vHat = m_obs.Velocity();
            stateForControl[IDX_X]=rHat[0];    stateForControl[IDX_Y]=rHat[1];    stateForControl[IDX_Z]=rHat[2];
            stateForControl[IDX_XDOT]=vHat[0]; stateForControl[IDX_YDOT]=vHat[1]; stateForControl[IDX_ZDOT]=vHat[2];
        }
    }
    else
    {
        // No estimator: the LQR feedback reads the raw (sensor-corrupted) position
        // directly, so sensor noise/bias bite without filtering. Identity sensor
        // -> the true position, unchanged. (Integral states still use the true
        // state inside the generated dynamics.)
        const auto yPos = measuredThrough(m_posSensor,
            std::array<double,POS_DIM>{{ m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z] }});
        stateForControl[IDX_X]=yPos[0]; stateForControl[IDX_Y]=yPos[1]; stateForControl[IDX_Z]=yPos[2];
    }
    Rocket::InputVec uApplied{};
    {
        CDS_PROFILE(profile, "Execute control");
        uApplied = pDyn->ExecuteControl(stateForControl, ref);
    }

    // Runge Kutta 4 (generic fixed-control step; reference held constant, ZOH)
    {
        CDS_PROFILE(profile, "RK4 integration");
        m_state = integrate::rk4_step<16>(m_state, params.timestep,
            [&](const Rocket::StateVec& s) { return pDyn->Dynamics(s, uApplied, ref, m_userForces); });
    }

    // ---- state estimator update (opt-in) ------------------------------------
    // Every tick at the measured dt off the fresh state: known input a_known is
    // the model's force-free acceleration; position measured through the sensor
    // bank; a dropped axis is handled as predict-only inside the helper.
    if (m_obsEnabled)
    {
        if (!m_obs.Seeded())
            m_obs.Seed({{ m_state[IDX_X],    m_state[IDX_Y],    m_state[IDX_Z]    }},
                       {{ m_state[IDX_XDOT], m_state[IDX_YDOT], m_state[IDX_ZDOT] }});

        const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
        const Rocket::StateVec xdot = pDyn->Dynamics(m_state, uApplied, ref, zeroF);
        const std::array<double,POS_DIM> aKnown{{ xdot[IDX_XDOT], xdot[IDX_YDOT], xdot[IDX_ZDOT] }};

        const std::array<double,POS_DIM> truthPos{{ m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z] }};
        std::array<double,POS_DIM> yPos{};
        std::array<bool,POS_DIM>   valid{};
        m_posSensor.Apply(truthPos, yPos, valid);

        m_obs.Step(aKnown, yPos, valid, params.timestep);
    }

    m_time += params.timestep;

    // Data recorder: one wide row per tick. Row alignment: the state is
    // post-integration at t_sim; uApplied is the command held over the step;
    // the reference and tracking error are sampled at the step start (they lead
    // the state by one dt — negligible at the simulation timestep).
#if CDS_RECORD_ENABLED
    {
        const std::array<double, 35> row{{
            m_time,
            m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z],
            m_state[IDX_ALPHA], m_state[IDX_BETA], m_state[IDX_PSI],
            m_state[IDX_XDOT], m_state[IDX_YDOT], m_state[IDX_ZDOT],
            m_state[IDX_ALPHADOT], m_state[IDX_BETADOT], m_state[IDX_PSIDOT],
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

bool Rocket::GetState(core_state_t& state)
{
    // Copies rocket's state in the core's struct
    state.x_dot = m_state[IDX_XDOT];
    state.y_dot = m_state[IDX_YDOT];
    state.z_dot = m_state[IDX_ZDOT];
    state.x = m_state[IDX_X];
    state.y = m_state[IDX_Y];
    state.z = m_state[IDX_Z];
    state.roll_dot = m_state[IDX_ALPHADOT];
    state.pitch_dot = m_state[IDX_BETADOT];
    state.yaw_dot = m_state[IDX_PSIDOT];
    state.roll = m_state[IDX_ALPHA];
    state.pitch = m_state[IDX_BETA];
    state.yaw = m_state[IDX_PSI];

    // IDX_INTX    
    // IDX_INTY    
    // IDX_INTZ    

    return false;
}

bool Rocket::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x   = m_trackingErr[0];
    tErrors.y   = m_trackingErr[1];
    tErrors.z   = m_trackingErr[2];
    tErrors.yaw = m_trackingErr[3];
    return false;
}

bool Rocket::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
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
bool Rocket::RecomputeGain()
{
    auto pDyn = static_cast<Dynamics::ROCKET_FF_LQR_01*>(m_modelPtr);
    if (pDyn == nullptr) return true;
    if (m_lqr.synthesize(*pDyn))
    {
        CDS_LOG_ERROR(logger, "LQR gain synthesis failed; keeping previous gain");
        return true;
    }
    return false;
}

void Rocket::SetWeights(const double Q[16][16], const double R[4][4])
{
    m_lqr.SetWeights(Q, R);
    RecomputeGain();
}

void Rocket::GetWeights(double Q[16][16], double R[4][4]) const { m_lqr.GetWeights(Q, R); }
void Rocket::GetGain(double K[4][16]) const                    { m_lqr.GetGain(K); }
double Rocket::GetGainBridgeError() const                      { return m_lqr.bridgeError(); }

// ---- exposed controller parameters: the LQR cost diagonal (Q, R) -------------
// Semantic labels for the 16-state error vector and the 4 inputs -- the anchor
// that ties each parameter id (its manifest row) to a physical meaning.
namespace {
const char* const ROCKET_ERR_LABELS[16] = {
    "x", "y", "z", "alpha", "beta", "psi", "vx", "vy", "vz",
    "alpha_dot", "beta_dot", "psi_dot", "int_x", "int_y", "int_z", "int_psi" };
const char* const ROCKET_IN_LABELS[4] = { "F1", "T1", "T2", "T3" };
} // namespace

void Rocket::BuildParamTable()
{
    m_params.clear();
    for (std::size_t i = 0; i < 16; ++i)
        m_params.add("Q", ROCKET_ERR_LABELS[i], true,
                     [this, i] { return m_lqr.qDiag(i); },
                     [this, i](double v) { if (v < 0.0) return true;  m_lqr.setQDiag(i, v); return RecomputeGain(); });
    for (std::size_t a = 0; a < 4; ++a)
        m_params.add("R", ROCKET_IN_LABELS[a], true,
                     [this, a] { return m_lqr.rDiag(a); },
                     [this, a](double v) { if (v <= 0.0) return true; m_lqr.setRDiag(a, v); return RecomputeGain(); });

    // Observer covariances + per-axis position sensor knobs (ride the same
    // manifest; a covariance change re-synthesises the gain).
    appendEstimatorParams(m_params, m_obsEnabled, m_posSensor,
                          m_obsQpos, m_obsQvel, m_obsQdist, m_obsRpos,
                          [this]{ BuildObserver(); });
}

bool Rocket::GetControllerManifest(char* buf, std::size_t n)
{
    m_params.buildManifest(buf, n);
    return false;
}

bool Rocket::SetControllerParam(int id, double value) { return m_params.set(id, value); }

} // namespace CDS

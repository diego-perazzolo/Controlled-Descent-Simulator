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
// File        : RocketMPC.cpp
// Description : Rocket 6-DOF (Euler-angle) runtime model with a nonlinear MPC.
//               The control-limited iLQR/DDP solver is the generic, reusable
//               CDS::control::solve (libs/control/ilqr.hpp); this file only holds
//               the rocket-specific pieces the solver needs -- the Gauss-Newton
//               tracking cost (direct, heading-wrapped Euler-angle errors and a
//               per-input control weight) -- as private machinery. Being Euler,
//               it needs no state projection (no quaternion renorm) and the
//               command is the wrench [F1, T1, T2, T3] directly, box-constrained
//               per input. Each tick it samples the reference over the horizon,
//               builds the cost closures, solves warm-started, applies the first
//               command as a zero-order hold, and integrates the plant by the
//               measured step. Controller knobs are runtime-tunable.
//               Mirrors modeling/notebooks/model/dynamics_rocket_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "RocketMPC.hpp"

#include <array>
#include <cmath>
#include <algorithm>

#include "dynamics_rocket_mpc_01.hpp"
#include "rk4.hpp"
#include "ilqr.hpp"
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"
#include "observer_params.hpp"   // libs/estimate -- observer knobs registration
#include "sensor_params.hpp"     // libs/sensor   -- sensor knobs registration + measuredThrough


// State indexes (match CDS::Dynamics::ROCKET_MPC_01::StateName)
#define IDX_X      0
#define IDX_Y      1
#define IDX_Z      2
#define IDX_ALPHA  3
#define IDX_BETA   4
#define IDX_PSI    5
#define IDX_VX     6
#define IDX_VY     7
#define IDX_VZ     8
#define IDX_DALPHA 9
#define IDX_DBETA  10
#define IDX_DPSI   11

namespace CDS {

static const auto logger = cds_log::registry().module("Rocket MPC");
static const auto profile = cds_profile::registry().module("Rocket MPC");

// ----------------------------------------------------------------------------
// Data recorder (black-box wide CSV, server-side). Channel 0 is t_sim; then the
// 12-state (Euler attitude, no integrators -- the MPC is not augmented), the
// 4 wrench inputs, the position/heading/velocity reference, the tracking error
// and the user forces. See the row-alignment note in PerformIntegration.
// ----------------------------------------------------------------------------
static cds_record::Recorder<double, 31, 4096> recorder("Rocket MPC", {{
    "t_sim",
    "x", "y", "z", "alpha", "beta", "psi",
    "vx", "vy", "vz", "alpha_dot", "beta_dot", "psi_dot",
    "F1", "T1", "T2", "T3",
    "ref_x", "ref_y", "ref_z", "ref_yaw", "ref_vx", "ref_vy", "ref_vz",
    "e_x", "e_y", "e_z", "e_yaw",
    "uf_x", "uf_y", "uf_z",
}});

// =============================================================================
//  Rocket-specific MPC machinery -- private to this translation unit. The generic
//  iLQR solver lives in libs/control/ilqr.hpp; here we only supply the tracking
//  cost (Gauss-Newton residuals, with a heading-wrapped Euler attitude error and
//  per-input control weights) and the hard-wired controller tuning. Being Euler,
//  there is no state projection to supply (the quadrotor MPC renormalises its
//  quaternion here; the rocket needs nothing).
// =============================================================================
namespace {

using Model = Dynamics::ROCKET_MPC_01;

constexpr std::size_t NX = 12;
constexpr std::size_t NU = 4;
constexpr std::size_t NR = 12;   // state-residual dimension

using V12    = std::array<double, NX>;
using V4     = std::array<double, NU>;
using M12x12 = std::array<V12, NR>;

// State-error weights (per block) and control weights (per input).
struct Weights { double wp, wq, wv, ww, wterm; double rF1, rT1, rT2, rT3; };
struct StageRef { std::array<double,3> p; std::array<double,3> ang; std::array<double,3> v; std::array<double,3> w; };

// As for the quadrotor, but the rocket flies descent trajectories hundreds of
// metres tall, so the box that still counts as "a prediction of this vehicle" is
// correspondingly larger. This is only the FLOOR of the runaway test: the box
// grows with the reference the horizon is tracking, because "large" only means
// anything relative to where the trajectory is asking the vehicle to be -- a
// full-scale descent legitimately sits at 10^5 m and 10^3 m/s, and a fixed box
// would flag every single solve. See runawayLimit() and SolveMpc().
constexpr double STATE_SANITY_LIMIT = 2000.0;

// How far past the reference a rollout may reach before it stops being a
// prediction of this flight. Generous on purpose: the test is there to catch a
// prediction that has left the trajectory behind entirely, not to grade tracking.
constexpr double RUNAWAY_REF_MARGIN = 5.0;

inline double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// ---- cost: residuals + Gauss-Newton Jacobian (SS4 of the notebook) ----
//   Euler attitude error is a plain angle difference; the heading (psi) error is
//   wrapped to [-pi, pi]. Wrapping changes the residual value but not its slope
//   (1 a.e.), so the Jacobian stays a constant diagonal.
void stateResidualAndJacobian(const V12& x, const StageRef& ref, const Weights& w, V12& r, M12x12& J)
{
    for (auto& row : J) row.fill(0.0);
    r[0]=w.wp*(x[0]-ref.p[0]); r[1]=w.wp*(x[1]-ref.p[1]); r[2]=w.wp*(x[2]-ref.p[2]);
    J[0][0]=J[1][1]=J[2][2]=w.wp;
    r[3]=w.wq*(x[3]-ref.ang[0]); r[4]=w.wq*(x[4]-ref.ang[1]); r[5]=w.wq*wrapPi(x[5]-ref.ang[2]);
    J[3][3]=J[4][4]=J[5][5]=w.wq;
    r[6]=w.wv*(x[6]-ref.v[0]); r[7]=w.wv*(x[7]-ref.v[1]); r[8]=w.wv*(x[8]-ref.v[2]);
    J[6][6]=J[7][7]=J[8][8]=w.wv;
    r[9]=w.ww*(x[9]-ref.w[0]); r[10]=w.ww*(x[10]-ref.w[1]); r[11]=w.ww*(x[11]-ref.w[2]);
    J[9][9]=J[10][10]=J[11][11]=w.ww;
}

// ---- per-stage cost payload for the generic solver (state GN + per-input control ridge) ----
control::StageCost<NX,NU> rocketStageCost(const V12& x, const V4& u, const StageRef& ref, const V4& uref, const Weights& w)
{
    control::StageCost<NX,NU> c;
    V12 r; M12x12 J; stateResidualAndJacobian(x, ref, w, r, J);
    for (std::size_t j=0;j<NX;++j){
        double g=0.0; for (std::size_t i=0;i<NR;++i) g+=J[i][j]*r[i]; c.lx[j]=g;
        for (std::size_t k=0;k<NX;++k){ double h=0.0; for (std::size_t i=0;i<NR;++i) h+=J[i][j]*J[i][k]; c.lxx[j][k]=h; }
    }
    const double rw[NU]={w.rF1, w.rT1, w.rT2, w.rT3};   // per-input control weights (heterogeneous actuators)
    double su=0.0;
    for (std::size_t a=0;a<NU;++a){ const double ru=rw[a]*(u[a]-uref[a]); c.lu[a]=rw[a]*ru; c.luu[a][a]=rw[a]*rw[a]; su+=ru*ru; }
    double sr=0.0; for (double ri : r) sr+=ri*ri;
    c.val=0.5*(sr+su);
    return c;
}

// ---- terminal cost payload: W_TERM * state-only Gauss-Newton cost ----
control::TerminalCost<NX> rocketTermCost(const V12& x, const StageRef& ref, const Weights& w)
{
    control::TerminalCost<NX> c;
    V12 r; M12x12 J; stateResidualAndJacobian(x, ref, w, r, J);
    for (std::size_t j=0;j<NX;++j){
        double g=0.0; for (std::size_t i=0;i<NR;++i) g+=J[i][j]*r[i]; c.lx[j]=w.wterm*g;
        for (std::size_t k=0;k<NX;++k){ double h=0.0; for (std::size_t i=0;i<NR;++i) h+=J[i][j]*J[i][k]; c.lxx[j][k]=w.wterm*h; }
    }
    double sr=0.0; for (double ri : r) sr+=ri*ri;
    c.val=w.wterm*0.5*sr;
    return c;
}

// ---- One RK4 step at the measured dt with the applied command (no projection) ----
V12 integrateStep(const Model& model, const V12& x, const V4& u, const std::array<double,3>& uF, double dt)
{
    return integrate::rk4_step<NX>(x, dt, [&](const V12& s){ return model.Dynamics(s, u, uF); });
}

// ---- reference preview over the prediction horizon (one StageRef per stage) ----
// Past the end of the trajectory the last sampled stage is held. Returns true
// (error) when not even the first stage can be sampled: there is nothing to hold
// then, and the caller must skip the solve rather than read an unset stage.
bool sampleHorizon(TrajectoryManager& tm, double t0, double dtMpc, std::size_t horizon,
                   std::array<StageRef, RocketMPC::MAX_HORIZON + 1>& refs)
{
    for (std::size_t k = 0; k <= horizon; ++k)
    {
        Reference_t r;
        if (tm.GetReference(t0 + k * dtMpc, r))
        {
            if (k == 0) { return true; }     // no first stage to hold: nothing to solve from
            refs[k] = refs[k - 1];           // beyond the trajectory: hold the last
            continue;
        }
        refs[k].p   = {{r.pos[0], r.pos[1], r.pos[2]}};
        refs[k].ang = {{0.0, 0.0, r.yaw}};   // upright attitude at the commanded heading
        refs[k].v   = {{r.vel[0], r.vel[1], r.vel[2]}};
        refs[k].w   = {{0.0, 0.0, 0.0}};
    }
    return false;
}

// The runaway box for this solve: the sanity floor, or a multiple of the largest
// position / velocity the horizon's reference asks for, whichever is larger.
// Only pos and vel enter the scale -- the attitude and rate references are 0 (or
// a bounded angle), so they carry no magnitude information.
double runawayLimit(const std::array<StageRef, RocketMPC::MAX_HORIZON + 1>& refs,
                    std::size_t horizon)
{
    double scale = 0.0;
    for (std::size_t k = 0; k <= horizon; ++k)
    {
        for (std::size_t i = 0; i < 3; ++i)
        {
            scale = std::max(scale, std::abs(refs[k].p[i]));
            scale = std::max(scale, std::abs(refs[k].v[i]));
        }
    }
    return std::max(STATE_SANITY_LIMIT, RUNAWAY_REF_MARGIN * scale);
}

void initState(const Reference_t& ref, V12& s)
{
    s.fill(0.0);
    s[IDX_X]=ref.pos[0]; s[IDX_Y]=ref.pos[1]; s[IDX_Z]=ref.pos[2];
    s[IDX_PSI]=ref.yaw;                       // heading = ref.yaw so the yaw error starts at zero
    s[IDX_VX]=ref.vel[0]; s[IDX_VY]=ref.vel[1]; s[IDX_VZ]=ref.vel[2];
}

} // anonymous namespace

// =============================================================================
RocketMPC::RocketMPC()
{
    m_modelPtr = new CDS::Dynamics::ROCKET_MPC_01();
    m_state.fill(0);
    m_trackingErr.fill(0);
    m_userForces.fill(0);
    for (auto& u : m_warmStart) u.fill(0);
    m_lastU0.fill(0);
    m_trajectoryManagerPtr = nullptr;
    m_time = 0;
    m_seeded = false;
    m_lastSolveTime = 0;
    m_lastSolveWarnTime = -1.0e30;   // nothing reported yet

    // Controller knobs default to the values tuned in the notebook.
    m_wp = 6.0; m_wq = 8.0; m_wv = 1.0; m_ww = 0.50; m_wterm = 20.0;
    m_rF1 = 0.02; m_rT1 = 0.20; m_rT2 = 0.20; m_rT3 = 0.20;   // per-input control weights
    m_dtMpc = 0.02;      // MPC prediction / control step [s]
    m_maxIters = 12;     // iLQR iterations per solve (warm-started)
    m_horizon = 40;      // active prediction horizon N (runtime-tunable, 1..MAX_HORIZON)
    BuildParamTable();

    // Offset-free disturbance observer: OFF by default (opt-in). Gain synthesised
    // in SetModelParams, once the model is parameterised. The rocket is heavy
    // (small disturbance-input coupling Bd = 1/m), so the disturbance is weakly
    // observed; a larger disturbance-state process variance raises the observer
    // bandwidth to keep convergence brisk (still well inside the numerically safe
    // qDist/rPos ratio).
    m_obsEnabled = false;
    m_obsQpos = 1.0e-4; m_obsQvel = 1.0e-2; m_obsQdist = 25.0; m_obsRpos = 1.0e-4;

    recorder.activateAsModel(); // this model owns the model data recorder while it lives
}

// Synthesise the translational disturbance observer. The only physical input is
// the disturbance-input coupling Bd = d(v_dot)/d(F_ext), read FROM the generated
// model by finite-differencing Dynamics w.r.t. the external force (the force
// enters additively, so Bd is state/input-independent -- evaluated once at the
// upright hover). Everything generic lives in the reusable helper.
void RocketMPC::BuildObserver()
{
    auto dyn = (Dynamics::ROCKET_MPC_01*) m_modelPtr;
    if (dyn == nullptr) return;

    using PN = Dynamics::ROCKET_MPC_01::ParamName;
    const double mg = dyn->GetParam(PN::Mass) * dyn->GetParam(PN::Gravity);
    V12 x{}; x.fill(0.0);                 // upright, at rest, at the origin
    const V4 u{{mg, 0.0, 0.0, 0.0}};      // hover wrench

    const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
    const V12 d0 = dyn->Dynamics(x, u, zeroF);
    control::Mat<POS_DIM, POS_DIM> Bd{}; for (auto& r : Bd) r.fill(0.0);
    for (std::size_t j = 0; j < POS_DIM; ++j)
    {
        std::array<double,3> Fe{{0.0, 0.0, 0.0}}; Fe[j] = 1.0;   // unit external force on axis j
        const V12 dj = dyn->Dynamics(x, u, Fe);
        Bd[0][j] = dj[IDX_VX] - d0[IDX_VX];
        Bd[1][j] = dj[IDX_VY] - d0[IDX_VY];
        Bd[2][j] = dj[IDX_VZ] - d0[IDX_VZ];
    }

    if (m_obs.Build(Bd, m_obsQpos, m_obsQvel, m_obsQdist, m_obsRpos))
        CDS_LOG_ERROR(logger, "Disturbance-observer synthesis failed");
}

RocketMPC::~RocketMPC()
{
    if (m_modelPtr) { delete (CDS::Dynamics::ROCKET_MPC_01*) m_modelPtr; m_modelPtr = nullptr; }
}

bool RocketMPC::SetModelParams(const std::any& params)
{
    auto dynamics = (Dynamics::ROCKET_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || params.type() != typeid(core_rocketParams_t&))
    {
        CDS_LOG_ERROR(logger, "Model not initialized or wrong params type");
        return true;
    }
    const auto& p = std::any_cast<const core_rocketParams_t&>(params);
    using PN = Dynamics::ROCKET_MPC_01::ParamName;
    dynamics->SetParam(PN::Mass,        p.m);
    dynamics->SetParam(PN::Ix,          p.Ix);
    dynamics->SetParam(PN::Iy,          p.Iy);
    dynamics->SetParam(PN::Iz,          p.Iz);
    dynamics->SetParam(PN::Gravity,     p.g);
    dynamics->SetParam(PN::DragLateral, p.c);
    dynamics->SetParam(PN::DragAxial,   p.cz);
    dynamics->SetParam(PN::ThrustMax,   p.F1_max);
    dynamics->SetParam(PN::ThrustMin,   p.F1_min);
    dynamics->SetParam(PN::TorqueXMax,  p.T1_max);
    dynamics->SetParam(PN::TorqueXMin,  p.T1_min);
    dynamics->SetParam(PN::TorqueYMax,  p.T2_max);
    dynamics->SetParam(PN::TorqueYMin,  p.T2_min);
    dynamics->SetParam(PN::TorqueZMax,  p.T3_max);
    dynamics->SetParam(PN::TorqueZMin,  p.T3_min);

    // Synthesise the disturbance observer now the model is parameterised.
    BuildObserver();

    // Recorder run metadata: full model parameters (trajectory added in
    // SetTrajectoryManager, the last setup step).
    recorder.clearMeta();
    recorder.addMeta("model", "Rocket MPC (MPC-01)");
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

bool RocketMPC::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
{
    Reference_t ref;
    if (pTrajectoryManager == nullptr || pTrajectoryManager->GetReference(m_time, ref))
    {
        CDS_LOG_ERROR(logger, "Trajectory not initialized");
        return true;
    }
    m_trajectoryManagerPtr = pTrajectoryManager;
    initState(ref, m_state);

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

bool RocketMPC::PerformIntegration(const core_stepParams_t& params)
{
    auto dynamics = (Dynamics::ROCKET_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || m_trajectoryManagerPtr == nullptr) { return true; }

    /* Current reference: drives the tracking errors, and anchors the horizon
       preview when the controller re-solves */
    Reference_t ref0;
    if (m_trajectoryManagerPtr->GetReference(m_time, ref0)) { return true; }
    UpdateTrackingErrors(ref0);

    /* The solve runs at the control cadence only; in between, m_lastU0 is held */
    if (!m_seeded || (m_time - m_lastSolveTime) >= m_dtMpc) { SolveMpc(); }

    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;
    {
        CDS_PROFILE(profile, "Rk4 integration");
        m_state = integrateStep(*dynamics, m_state, m_lastU0, m_userForces, params.timestep);
    }

    if (m_obsEnabled) { UpdateObserver(params.timestep); }

    m_time += params.timestep;
    RecordTick(ref0);

    return false;
}

void RocketMPC::UpdateTrackingErrors(const Reference_t& ref0)
{
    m_trackingErr[0] = ref0.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref0.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref0.pos[2] - m_state[IDX_Z];
    m_trackingErr[3] = wrapPi(ref0.yaw - m_state[IDX_PSI]);
}

void RocketMPC::SolveMpc(void)
{
    auto dynamics = (Dynamics::ROCKET_MPC_01*) m_modelPtr;

    CDS_PROFILE(profile, "Total MPC execution");
    const Weights W{m_wp, m_wq, m_wv, m_ww, m_wterm, m_rF1, m_rT1, m_rT2, m_rT3};

    /* hover command and per-input actuator box from the model params */
    using PN = Dynamics::ROCKET_MPC_01::ParamName;
    const double mg = dynamics->GetParam(PN::Mass) * dynamics->GetParam(PN::Gravity);
    const V4 uref{{mg, 0.0, 0.0, 0.0}};
    const V4 lo{{dynamics->GetParam(PN::ThrustMin), dynamics->GetParam(PN::TorqueXMin),
                 dynamics->GetParam(PN::TorqueYMin), dynamics->GetParam(PN::TorqueZMin)}};
    const V4 hi{{dynamics->GetParam(PN::ThrustMax), dynamics->GetParam(PN::TorqueXMax),
                 dynamics->GetParam(PN::TorqueYMax), dynamics->GetParam(PN::TorqueZMax)}};
    if (!m_seeded) { for (auto& u : m_warmStart) u = uref; }

    std::array<StageRef, RocketMPC::MAX_HORIZON + 1> refs;
    if (sampleHorizon(*m_trajectoryManagerPtr, m_time, m_dtMpc, m_horizon, refs))
    {
        /* No reference at the horizon start: keep holding the last command */
        return;
    }

    std::array<double, POS_DIM> predForce{{0.0, 0.0, 0.0}};
    const V12 x0 = ControllerInitialState(predForce);

    /* rocket-specific closures handed to the generic iLQR (Euler state: no
       projection needed) */
    auto fdyn  = [&](const V12& x, const V4& u){ return dynamics->Dynamics(x, u, predForce); };
    auto jac   = [&](const V12& x, const V4& u, double fx[NX][NX], double fu[NX][NU]){ dynamics->Jacobians(x, u, fx, fu); };
    auto proj  = [](V12&){ };
    auto scost = [&](const V12& x, const V4& u, std::size_t k){ return rocketStageCost(x, u, refs[k], uref, W); };
    auto tcost = [&](const V12& x){ return rocketTermCost(x, refs[m_horizon], W); };

    V4 u0{};
    control::SolveReport report;
    bool solveFailed = false;
    {
        CDS_PROFILE(profile, "MPC solve");
        solveFailed = control::solve<NX, NU, RocketMPC::MAX_HORIZON>(
            x0, fdyn, jac, proj, scost, tcost, lo, hi, m_dtMpc, m_maxIters, m_horizon, m_warmStart, u0,
            &report);
    }

    /* A refused solve leaves u0 and the warm start untouched, so the previous
       command stays in force (ZOH) until the next cadence: a stale command beats
       one the solver itself would not vouch for. */
    if (solveFailed)
    {
        if (m_time - m_lastSolveWarnTime >= 1.0)
        {
            m_lastSolveWarnTime = m_time;
            CDS_LOG_ERROR(logger, "MPC solve refused (horizon {}) at t = {} s: holding the previous command",
                          static_cast<int>(m_horizon), std::round(m_time * 100.0) / 100.0);
        }
        m_lastSolveTime = m_time;
        return;
    }

    /* Finite and in-box is not the same as trustworthy: over a horizon long
       compared with the open-loop instability the predicted rollout can run away
       while every number stays finite, and the command then optimises a
       trajectory the vehicle will never fly. Report it -- the command is applied
       anyway, since refusing every solve would freeze the controller. */
    if (report.stateMax > runawayLimit(refs, m_horizon) && (m_time - m_lastSolveWarnTime >= 1.0))
    {
        m_lastSolveWarnTime = m_time;
        CDS_LOG_WARN(logger, "MPC prediction is running away (horizon {}) at t = {} s: |state|max = {}, cost = {}. "
                             "The horizon is long compared with the open-loop instability: shorten it",
                     static_cast<int>(m_horizon), std::round(m_time * 100.0) / 100.0,
                     static_cast<int>(report.stateMax), static_cast<int>(report.cost));
    }

    m_lastU0        = u0;
    m_lastSolveTime = m_time;
    m_seeded        = true;
}

RocketMPC::StateVec RocketMPC::ControllerInitialState(std::array<double, RocketMPC::POS_DIM>& predForce)
{
    StateVec x0 = m_state;
    predForce = {{0.0, 0.0, 0.0}};

    if (m_obsEnabled)
    {
        if (m_obs.Seeded())
        {
            const auto rHat = m_obs.Position();
            const auto vHat = m_obs.Velocity();
            const auto dHat = m_obs.Disturbance();
            x0[IDX_X]=rHat[0];  x0[IDX_Y]=rHat[1];  x0[IDX_Z]=rHat[2];
            x0[IDX_VX]=vHat[0]; x0[IDX_VY]=vHat[1]; x0[IDX_VZ]=vHat[2];
            predForce = {{dHat[0], dHat[1], dHat[2]}};
        }
    }
    else
    {
        /* No estimator: the raw (sensor-corrupted) position goes straight to the
           controller, so noise and bias bite unfiltered. Identity sensor -> truth */
        const auto yPos = measuredThrough(m_posSensor,
            std::array<double,POS_DIM>{{ m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z] }});
        x0[IDX_X]=yPos[0]; x0[IDX_Y]=yPos[1]; x0[IDX_Z]=yPos[2];
    }

    return x0;
}

void RocketMPC::UpdateObserver(core_coord_t dt)
{
    auto dynamics = (Dynamics::ROCKET_MPC_01*) m_modelPtr;

    /* One scope over the whole per-tick observer work -- sensor read, known
       input, prediction and correction -- which is what "how much does the
       estimator cost" means. Scoping only the seed would time a one-shot copy
       that happens once per run. */
    CDS_PROFILE(profile, "Execute observer");

    if (!m_obs.Seeded())
        m_obs.Seed({{ m_state[IDX_X],  m_state[IDX_Y],  m_state[IDX_Z]  }},
                   {{ m_state[IDX_VX], m_state[IDX_VY], m_state[IDX_VZ] }});

    /* known force-free acceleration: exactly the MPC's prediction assumption */
    const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
    const V12 xdot = dynamics->Dynamics(m_state, m_lastU0, zeroF);
    const std::array<double,POS_DIM> aKnown{{ xdot[IDX_VX], xdot[IDX_VY], xdot[IDX_VZ] }};

    const std::array<double,POS_DIM> truthPos{{ m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z] }};
    std::array<double,POS_DIM> yPos{};
    std::array<bool,POS_DIM>   valid{};
    m_posSensor.Apply(truthPos, yPos, valid);

    m_obs.Step(aKnown, yPos, valid, dt);
}

void RocketMPC::RecordTick(const Reference_t& ref0)
{
#if CDS_RECORD_ENABLED
    const std::array<double, 31> row{{
        m_time,
        m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z],
        m_state[IDX_ALPHA], m_state[IDX_BETA], m_state[IDX_PSI],
        m_state[IDX_VX], m_state[IDX_VY], m_state[IDX_VZ],
        m_state[IDX_DALPHA], m_state[IDX_DBETA], m_state[IDX_DPSI],
        m_lastU0[0], m_lastU0[1], m_lastU0[2], m_lastU0[3],
        ref0.pos[0], ref0.pos[1], ref0.pos[2], ref0.yaw, ref0.vel[0], ref0.vel[1], ref0.vel[2],
        m_trackingErr[0], m_trackingErr[1], m_trackingErr[2], m_trackingErr[3],
        m_userForces[0], m_userForces[1], m_userForces[2],
    }};
    recorder.record(row);
#else
    (void) ref0;
#endif
}

bool RocketMPC::GetState(core_state_t& state)
{
    state.x_dot = m_state[IDX_VX]; state.y_dot = m_state[IDX_VY]; state.z_dot = m_state[IDX_VZ];
    state.x = m_state[IDX_X]; state.y = m_state[IDX_Y]; state.z = m_state[IDX_Z];
    state.roll_dot = m_state[IDX_DALPHA]; state.pitch_dot = m_state[IDX_DBETA]; state.yaw_dot = m_state[IDX_DPSI];
    state.roll = m_state[IDX_ALPHA]; state.pitch = m_state[IDX_BETA]; state.yaw = m_state[IDX_PSI];
    return false;
}

bool RocketMPC::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x = m_trackingErr[0]; tErrors.y = m_trackingErr[1];
    tErrors.z = m_trackingErr[2]; tErrors.yaw = m_trackingErr[3];
    return false;
}

bool RocketMPC::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
{
    currentTimeSeconds = m_time;
    return false;
}

// ---- exposed controller parameters: the MPC cost weights and solver knobs ----
// The state weights are the per-block Gauss-Newton cost weights; the control
// weights are per-input (F1/T1/T2/T3) because the rocket's actuators are
// heterogeneous. dt_mpc is the control step (also the horizon spacing); max_iters
// caps the iLQR iterations per solve. The horizon N is runtime-tunable (buffers
// fixed at MAX_HORIZON). No gain to re-synthesise: the MPC re-solves
// from scratch every control step.
// Per-axis group names for the position sensor bank (x/y/z).
static const char* const SENSOR_GROUPS[3] = { "Sensor x", "Sensor y", "Sensor z" };

void RocketMPC::BuildParamTable()
{
    // Model bucket: the runtime-tunable prediction horizon.
    m_modelParams.clear();
    // Active prediction horizon N (runtime-tunable up to the fixed buffer
    // capacity). A change re-seeds the warm start on the next solve.
    m_modelParams.add("solver", "horizon N", true,
        [this]{ return static_cast<double>(m_horizon); },
        [this](double v){ const int n = static_cast<int>(v + 0.5);
                          if (n < 1 || n > static_cast<int>(RocketMPC::MAX_HORIZON)) return true;
                          m_horizon = static_cast<std::size_t>(n); m_seeded = false; return false; });

    // Controller bucket: cost weights + solver knobs.
    m_controllerParams.clear();
    // State-error cost weights (the MPC's analogue of the LQR Q, one scalar per
    // state block rather than a full diagonal).
    m_controllerParams.add("Q (state cost)", "position", true, [this]{ return m_wp; },
                 [this](double v){ if (v < 0.0) return true; m_wp = v; return false; });
    m_controllerParams.add("Q (state cost)", "attitude", true, [this]{ return m_wq; },
                 [this](double v){ if (v < 0.0) return true; m_wq = v; return false; });
    m_controllerParams.add("Q (state cost)", "velocity", true, [this]{ return m_wv; },
                 [this](double v){ if (v < 0.0) return true; m_wv = v; return false; });
    m_controllerParams.add("Q (state cost)", "body rate", true, [this]{ return m_ww; },
                 [this](double v){ if (v < 0.0) return true; m_ww = v; return false; });
    // Control-effort cost weights, one per (heterogeneous) input.
    m_controllerParams.add("R (control cost)", "F1", true, [this]{ return m_rF1; },
                 [this](double v){ if (v <= 0.0) return true; m_rF1 = v; return false; });
    m_controllerParams.add("R (control cost)", "T1", true, [this]{ return m_rT1; },
                 [this](double v){ if (v <= 0.0) return true; m_rT1 = v; return false; });
    m_controllerParams.add("R (control cost)", "T2", true, [this]{ return m_rT2; },
                 [this](double v){ if (v <= 0.0) return true; m_rT2 = v; return false; });
    m_controllerParams.add("R (control cost)", "T3", true, [this]{ return m_rT3; },
                 [this](double v){ if (v <= 0.0) return true; m_rT3 = v; return false; });
    // Solver knobs: terminal-cost weight, control step, iteration cap.
    m_controllerParams.add("solver", "terminal weight", true, [this]{ return m_wterm; },
                 [this](double v){ if (v < 0.0) return true; m_wterm = v; return false; });
    m_controllerParams.add("solver", "dt_mpc [s]", true, [this]{ return m_dtMpc; },
                 [this](double v){ if (v <= 0.0) return true; m_dtMpc = v; return false; });
    m_controllerParams.add("solver", "max iters", true, [this]{ return static_cast<double>(m_maxIters); },
                 [this](double v){ const int n = static_cast<int>(v + 0.5); if (n < 1) return true; m_maxIters = n; return false; });

    // Observer bucket: covariances + enable (a covariance change re-synthesises
    // the gain off the tick path).
    m_observerParams.clear();
    estimate::appendObserverParams(m_observerParams, m_obsEnabled,
                                   m_obsQpos, m_obsQvel, m_obsQdist, m_obsRpos,
                                   [this]{ BuildObserver(); });

    // Sensor bucket: per-axis position-sensor knobs.
    m_sensorParams.clear();
    sensor::appendSensorParams(m_sensorParams, m_posSensor, SENSOR_GROUPS);
}

bool RocketMPC::GetModelManifest(char* buf, std::size_t n)      { m_modelParams.buildManifest(buf, n); return false; }
bool RocketMPC::SetModelParam(int id, double value)            { return m_modelParams.set(id, value); }
bool RocketMPC::GetControllerManifest(char* buf, std::size_t n){ m_controllerParams.buildManifest(buf, n); return false; }
bool RocketMPC::SetControllerParam(int id, double value)      { return m_controllerParams.set(id, value); }
bool RocketMPC::GetObserverManifest(char* buf, std::size_t n)  { m_observerParams.buildManifest(buf, n); return false; }
bool RocketMPC::SetObserverParam(int id, double value)        { return m_observerParams.set(id, value); }
bool RocketMPC::GetSensorManifest(char* buf, std::size_t n)    { m_sensorParams.buildManifest(buf, n); return false; }
bool RocketMPC::SetSensorParam(int id, double value)          { return m_sensorParams.set(id, value); }

} // namespace CDS

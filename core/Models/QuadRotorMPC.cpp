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
// File        : QuadRotorMPC.cpp
// Description : Quadrotor 6-DOF (quaternion) runtime model with a nonlinear MPC.
//               The control-limited iLQR/DDP solver is the generic, reusable
//               CDS::control::solve (libs/control/ilqr.hpp); this file only holds
//               the quad-specific pieces the solver needs -- the Gauss-Newton
//               tracking cost (with the error-quaternion Jacobian) and the
//               quaternion renormalisation -- as private machinery. Each tick it
//               samples the reference over the horizon, builds the cost closures,
//               solves warm-started, applies the first command as a ZOH, and
//               integrates the plant by the measured step. Controller knobs
//               (weights, horizon, dt) are hard-wired for now.
//               Mirrors modeling/notebooks/dynamics_quadRotor_MPC01.ipynb.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "QuadRotorMPC.hpp"

#include <array>
#include <cmath>
#include <algorithm>

#include "dynamics_quadrotor_mpc_01.hpp"
#include "rk4.hpp"
#include "ilqr.hpp"
#include "log.hpp"
#include "profile.hpp"
#include "Recorder.hpp"
#include "observer_params.hpp"   // libs/estimate -- observer knobs registration
#include "sensor_params.hpp"     // libs/sensor   -- sensor knobs registration + measuredThrough


// State indexes (match CDS::Dynamics::QUADROTOR_MPC_01::StateName)
#define IDX_X   0
#define IDX_Y   1
#define IDX_Z   2
#define IDX_QW  3
#define IDX_QX  4
#define IDX_QY  5
#define IDX_QZ  6
#define IDX_VX  7
#define IDX_VY  8
#define IDX_VZ  9
#define IDX_WX  10
#define IDX_WY  11
#define IDX_WZ  12

namespace CDS {

static const auto logger = cds_log::registry().module("Quadrotor MPC");
static const auto profile = cds_profile::registry().module("Quadrotor MPC");

// ----------------------------------------------------------------------------
// Data recorder (black-box wide CSV, server-side). Channel 0 is t_sim; then the
// 13-state (quaternion attitude, no integrators — the MPC is not augmented), the
// 4 rotor thrusts held this step, the position/heading/velocity reference, the
// tracking error and the user forces. See the row-alignment note below.
// ----------------------------------------------------------------------------
static cds_record::Recorder<double, 32, 4096> recorder("Quadrotor MPC", {{
    "t_sim",
    "x", "y", "z", "qw", "qx", "qy", "qz",
    "vx", "vy", "vz", "wx", "wy", "wz",
    "T1", "T2", "T3", "T4",
    "ref_x", "ref_y", "ref_z", "ref_yaw", "ref_vx", "ref_vy", "ref_vz",
    "e_x", "e_y", "e_z", "e_yaw",
    "uf_x", "uf_y", "uf_z",
}});

// =============================================================================
//  Quad-specific MPC machinery -- private to this translation unit. The generic
//  iLQR solver lives in libs/control/ilqr.hpp; here we only supply the tracking
//  cost (Gauss-Newton residuals, incl. the error-quaternion Jacobian), the
//  quaternion renormalisation, and the hard-wired controller tuning.
// =============================================================================
namespace {

using Model = Dynamics::QUADROTOR_MPC_01;

constexpr std::size_t NX = 13;
constexpr std::size_t NU = 4;
constexpr std::size_t NR = 12;   // state-residual dimension

using V13    = std::array<double, NX>;
using V4     = std::array<double, NU>;
using V12    = std::array<double, NR>;
using M12x13 = std::array<V13, NR>;

struct Weights { double wp, wq, wv, ww, wu, wterm; };
struct StageRef { std::array<double,3> p; std::array<double,4> q; std::array<double,3> v; std::array<double,3> w; };

// ---- quaternion helpers ----
inline void quatMul(const double a[4], const double b[4], double o[4])
{
    o[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    o[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    o[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    o[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}
inline void quatConj(const double a[4], double o[4]) { o[0]=a[0]; o[1]=-a[1]; o[2]=-a[2]; o[3]=-a[3]; }
inline void normalizeQuat(V13& x)
{
    const double n = std::sqrt(x[3]*x[3]+x[4]*x[4]+x[5]*x[5]+x[6]*x[6]);
    if (n > 1e-12){ const double s=1.0/n; x[3]*=s; x[4]*=s; x[5]*=s; x[6]*=s; }
}

// ---- cost: residuals + Gauss-Newton Jacobian (SS4 of the notebook) ----
void stateResidualAndJacobian(const V13& x, const StageRef& ref, const Weights& w, V12& r, M12x13& J)
{
    for (auto& row : J) row.fill(0.0);
    r[0]=w.wp*(x[0]-ref.p[0]); r[1]=w.wp*(x[1]-ref.p[1]); r[2]=w.wp*(x[2]-ref.p[2]);
    J[0][0]=J[1][1]=J[2][2]=w.wp;
    r[6]=w.wv*(x[7]-ref.v[0]); r[7]=w.wv*(x[8]-ref.v[1]); r[8]=w.wv*(x[9]-ref.v[2]);
    J[6][7]=J[7][8]=J[8][9]=w.wv;
    r[9]=w.ww*(x[10]-ref.w[0]); r[10]=w.ww*(x[11]-ref.w[1]); r[11]=w.ww*(x[12]-ref.w[2]);
    J[9][10]=J[10][11]=J[11][12]=w.ww;

    const double qref[4]={ref.q[0],ref.q[1],ref.q[2],ref.q[3]};
    const double qcur[4]={x[3],x[4],x[5],x[6]};
    double qrc[4]; quatConj(qref, qrc);
    double qe[4];  quatMul(qrc, qcur, qe);
    const double sgn = (qe[0]>=0.0)?1.0:-1.0;
    r[3]=w.wq*2.0*sgn*qe[1]; r[4]=w.wq*2.0*sgn*qe[2]; r[5]=w.wq*2.0*sgn*qe[3];
    const double a0=qrc[0], a1=qrc[1], a2=qrc[2], a3=qrc[3], c=2.0*sgn*w.wq;
    J[3][3]= c*a1;  J[3][4]= c*a0;  J[3][5]=-c*a3;  J[3][6]= c*a2;
    J[4][3]= c*a2;  J[4][4]= c*a3;  J[4][5]= c*a0;  J[4][6]=-c*a1;
    J[5][3]= c*a3;  J[5][4]=-c*a2;  J[5][5]= c*a1;  J[5][6]= c*a0;
}

// ---- per-stage cost payload for the generic solver (state GN + control ridge) ----
control::StageCost<NX,NU> quadStageCost(const V13& x, const V4& u, const StageRef& ref, const V4& uref, const Weights& w)
{
    control::StageCost<NX,NU> c;
    V12 r; M12x13 J; stateResidualAndJacobian(x, ref, w, r, J);
    for (std::size_t j=0;j<NX;++j){
        double g=0.0; for (std::size_t i=0;i<NR;++i) g+=J[i][j]*r[i]; c.lx[j]=g;
        for (std::size_t k=0;k<NX;++k){ double h=0.0; for (std::size_t i=0;i<NR;++i) h+=J[i][j]*J[i][k]; c.lxx[j][k]=h; }
    }
    double su=0.0;
    for (std::size_t a=0;a<NU;++a){ const double ru=w.wu*(u[a]-uref[a]); c.lu[a]=w.wu*ru; c.luu[a][a]=w.wu*w.wu; su+=ru*ru; }
    double sr=0.0; for (double ri : r) sr+=ri*ri;
    c.val=0.5*(sr+su);
    return c;
}

// ---- terminal cost payload: W_TERM * state-only Gauss-Newton cost ----
control::TerminalCost<NX> quadTermCost(const V13& x, const StageRef& ref, const Weights& w)
{
    control::TerminalCost<NX> c;
    V12 r; M12x13 J; stateResidualAndJacobian(x, ref, w, r, J);
    for (std::size_t j=0;j<NX;++j){
        double g=0.0; for (std::size_t i=0;i<NR;++i) g+=J[i][j]*r[i]; c.lx[j]=w.wterm*g;
        for (std::size_t k=0;k<NX;++k){ double h=0.0; for (std::size_t i=0;i<NR;++i) h+=J[i][j]*J[i][k]; c.lxx[j][k]=w.wterm*h; }
    }
    double sr=0.0; for (double ri : r) sr+=ri*ri;
    c.val=w.wterm*0.5*sr;
    return c;
}

// ---- One RK4 step at the measured dt with the applied command ----
V13 integrateAndNormalize(const Model& model, const V13& x, const V4& u, const std::array<double,3>& uF, double dt)
{
    V13 xn = integrate::rk4_step<NX>(x, dt, [&](const V13& s){ return model.Dynamics(s, u, uF); });
    normalizeQuat(xn);
    return xn;
}

// ---- reference preview over the prediction horizon (one StageRef per stage) ----
// Past the end of the trajectory the last sampled stage is held. Returns true
// (error) when not even the first stage can be sampled: there is nothing to hold
// then, and the caller must skip the solve rather than read an unset stage.
bool sampleHorizon(TrajectoryManager& tm, double t0, double dtMpc, std::size_t horizon,
                   std::array<StageRef, QuadRotorMPC::MAX_HORIZON + 1>& refs)
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
        const double h = 0.5 * r.yaw;
        refs[k].p = {{r.pos[0], r.pos[1], r.pos[2]}};
        refs[k].q = {{std::cos(h), 0.0, 0.0, std::sin(h)}};   // level attitude at the commanded heading
        refs[k].v = {{r.vel[0], r.vel[1], r.vel[2]}};
        refs[k].w = {{0.0, 0.0, 0.0}};
    }
    return false;
}

void initState(const Reference_t& ref, V13& s)
{
    s.fill(0.0);
    s[IDX_X]=ref.pos[0]; s[IDX_Y]=ref.pos[1]; s[IDX_Z]=ref.pos[2];
    const double h=0.5*ref.yaw; s[IDX_QW]=std::cos(h); s[IDX_QZ]=std::sin(h);
    s[IDX_VX]=ref.vel[0]; s[IDX_VY]=ref.vel[1]; s[IDX_VZ]=ref.vel[2];
}

} // anonymous namespace

// =============================================================================
QuadRotorMPC::QuadRotorMPC()
{
    m_modelPtr = new CDS::Dynamics::QUADROTOR_MPC_01();
    m_state.fill(0);
    m_state[IDX_QW] = 1.0;
    m_trackingErr.fill(0);
    m_userForces.fill(0);
    for (auto& u : m_warmStart) u.fill(0);
    m_lastU0.fill(0);
    m_trajectoryManagerPtr = nullptr;
    m_time = 0;
    m_seeded = false;
    m_lastSolveTime = 0;

    // Controller knobs default to the values tuned in the notebook.
    m_wp = 6.0; m_wq = 2.0; m_wv = 1.0; m_ww = 0.30; m_wu = 0.10; m_wterm = 8.0;
    m_dtMpc = 0.02;      // MPC prediction / control step [s]
    m_maxIters = 12;     // iLQR iterations per solve (warm-started)
    m_horizon = 40;      // active prediction horizon N (runtime-tunable, 1..MAX_HORIZON)
    BuildParamTable();

    // Offset-free disturbance observer: OFF by default (opt-in). Diagonal noise
    // covariances trade convergence speed against noise rejection; the large
    // disturbance-state variance lets d_hat integrate the position residual. The
    // gain is synthesised in SetModelParams, once the model is parameterised.
    m_obsEnabled = false;
    m_obsQpos = 1.0e-4; m_obsQvel = 1.0e-2; m_obsQdist = 1.0; m_obsRpos = 1.0e-4;

    recorder.activateAsModel(); // this model owns the model data recorder while it lives
}

// Synthesise the translational disturbance observer. The only physical input is
// the disturbance-input coupling Bd = d(v_dot)/d(F_ext), which is read FROM the
// generated model by finite-differencing Dynamics w.r.t. the external force (the
// force enters additively, so Bd is state/input-independent -- evaluated once at
// the hover). Everything else (the r_dot = v kinematics, the position
// measurement, the constant-disturbance model) lives in the reusable helper.
void QuadRotorMPC::BuildObserver()
{
    auto dyn = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
    if (dyn == nullptr) return;

    using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
    const double Th = dyn->GetParam(PN::Mass) * dyn->GetParam(PN::Gravity) / 4.0;
    V13 x{}; x.fill(0.0); x[IDX_QW] = 1.0;   // level, at rest, at the origin
    const V4 u{{Th, Th, Th, Th}};

    const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
    const V13 d0 = dyn->Dynamics(x, u, zeroF);
    control::Mat<POS_DIM, POS_DIM> Bd{}; for (auto& r : Bd) r.fill(0.0);
    for (std::size_t j = 0; j < POS_DIM; ++j)
    {
        std::array<double,3> Fe{{0.0, 0.0, 0.0}}; Fe[j] = 1.0;   // unit external force on axis j
        const V13 dj = dyn->Dynamics(x, u, Fe);
        Bd[0][j] = dj[IDX_VX] - d0[IDX_VX];
        Bd[1][j] = dj[IDX_VY] - d0[IDX_VY];
        Bd[2][j] = dj[IDX_VZ] - d0[IDX_VZ];
    }

    if (m_obs.Build(Bd, m_obsQpos, m_obsQvel, m_obsQdist, m_obsRpos))
        CDS_LOG_ERROR(logger, "Disturbance-observer synthesis failed");
}

QuadRotorMPC::~QuadRotorMPC()
{
    if (m_modelPtr) { delete (CDS::Dynamics::QUADROTOR_MPC_01*) m_modelPtr; m_modelPtr = nullptr; }
}

bool QuadRotorMPC::SetModelParams(const std::any& params)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || params.type() != typeid(core_quadRotorParams_t&))
    {
        CDS_LOG_ERROR(logger, "Model not initialized or wrong params type");
        return true;
    }
    const auto& p = std::any_cast<const core_quadRotorParams_t&>(params);
    using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
    dynamics->SetParam(PN::Mass,      p.m);
    dynamics->SetParam(PN::Ix,        p.Ix);
    dynamics->SetParam(PN::Iy,        p.Iy);
    dynamics->SetParam(PN::Iz,        p.Iz);
    dynamics->SetParam(PN::Gravity,   p.g);
    dynamics->SetParam(PN::DragX,     p.c);    // lateral drag on x
    dynamics->SetParam(PN::DragY,     p.c);    // lateral drag on y
    dynamics->SetParam(PN::DragZ,     p.cz);   // axial drag on z
    dynamics->SetParam(PN::KThrust,   p.kT);
    dynamics->SetParam(PN::KTorque,   p.kQ);
    dynamics->SetParam(PN::Arm,       p.L);
    dynamics->SetParam(PN::ThrustMax, p.Fm_max);
    dynamics->SetParam(PN::ThrustMin, p.Fm_min);

    // Synthesise the disturbance observer now the model is parameterised.
    BuildObserver();

    // Recorder run metadata: full model parameters (trajectory added in
    // SetTrajectoryManager, the last setup step).
    recorder.clearMeta();
    recorder.addMeta("model", "Quadrotor MPC (MPC-01)");
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
    recorder.addMeta("motor_thrust_max_N", p.Fm_max);
    recorder.addMeta("motor_thrust_min_N", p.Fm_min);

    return false;
}

bool QuadRotorMPC::SetTrajectoryManager(TrajectoryManager* pTrajectoryManager)
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

bool QuadRotorMPC::PerformIntegration(const core_stepParams_t& params)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
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
        m_state = integrateAndNormalize(*dynamics, m_state, m_lastU0, m_userForces, params.timestep);
    }

    if (m_obsEnabled) { UpdateObserver(params.timestep); }

    m_time += params.timestep;
    RecordTick(ref0);

    return false;
}

void QuadRotorMPC::UpdateTrackingErrors(const Reference_t& ref0)
{
    m_trackingErr[0] = ref0.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref0.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref0.pos[2] - m_state[IDX_Z];
    const double qw=m_state[IDX_QW], qx=m_state[IDX_QX], qy=m_state[IDX_QY], qz=m_state[IDX_QZ];
    const double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    double eyaw = ref0.yaw - yaw; eyaw = std::atan2(std::sin(eyaw), std::cos(eyaw));
    m_trackingErr[3] = eyaw;
}

void QuadRotorMPC::SolveMpc(void)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;

    CDS_PROFILE(profile, "Total MPC execution");
    const Weights W{m_wp, m_wq, m_wv, m_ww, m_wu, m_wterm};   // current cost weights

    /* hover command and actuator box from the model params */
    using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
    const double mg = dynamics->GetParam(PN::Mass) * dynamics->GetParam(PN::Gravity);
    const double Th = mg / 4.0;
    const V4 uref{{Th, Th, Th, Th}};
    V4 lo, hi;
    lo.fill(dynamics->GetParam(PN::ThrustMin));
    hi.fill(dynamics->GetParam(PN::ThrustMax));
    if (!m_seeded) { for (auto& u : m_warmStart) u = uref; }

    std::array<StageRef, QuadRotorMPC::MAX_HORIZON + 1> refs;
    if (sampleHorizon(*m_trajectoryManagerPtr, m_time, m_dtMpc, m_horizon, refs))
    {
        /* No reference at the horizon start: keep holding the last command */
        return;
    }

    std::array<double, POS_DIM> predForce{{0.0, 0.0, 0.0}};
    const V13 x0 = ControllerInitialState(predForce);

    /* quad-specific closures handed to the generic iLQR */
    auto fdyn  = [&](const V13& x, const V4& u){ return dynamics->Dynamics(x, u, predForce); };
    auto jac   = [&](const V13& x, const V4& u, double fx[NX][NX], double fu[NX][NU]){ dynamics->Jacobians(x, u, fx, fu); };
    auto proj  = [](V13& s){ normalizeQuat(s); };
    auto scost = [&](const V13& x, const V4& u, std::size_t k){ return quadStageCost(x, u, refs[k], uref, W); };
    auto tcost = [&](const V13& x){ return quadTermCost(x, refs[m_horizon], W); };

    V4 u0;
    {
        CDS_PROFILE(profile, "MPC solve");
        control::solve<NX, NU, QuadRotorMPC::MAX_HORIZON>(
            x0, fdyn, jac, proj, scost, tcost, lo, hi, m_dtMpc, m_maxIters, m_horizon, m_warmStart, u0);
    }

    m_lastU0        = u0;
    m_lastSolveTime = m_time;
    m_seeded        = true;
}

QuadRotorMPC::StateVec QuadRotorMPC::ControllerInitialState(std::array<double, QuadRotorMPC::POS_DIM>& predForce)
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

void QuadRotorMPC::UpdateObserver(core_coord_t dt)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;

    if (!m_obs.Seeded())
        m_obs.Seed({{ m_state[IDX_X],  m_state[IDX_Y],  m_state[IDX_Z]  }},
                   {{ m_state[IDX_VX], m_state[IDX_VY], m_state[IDX_VZ] }});

    /* known force-free acceleration: exactly the MPC's prediction assumption */
    const std::array<double,3> zeroF{{0.0, 0.0, 0.0}};
    const V13 xdot = dynamics->Dynamics(m_state, m_lastU0, zeroF);
    const std::array<double,POS_DIM> aKnown{{ xdot[IDX_VX], xdot[IDX_VY], xdot[IDX_VZ] }};

    const std::array<double,POS_DIM> truthPos{{ m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z] }};
    std::array<double,POS_DIM> yPos{};
    std::array<bool,POS_DIM>   valid{};
    m_posSensor.Apply(truthPos, yPos, valid);

    m_obs.Step(aKnown, yPos, valid, dt);
}

void QuadRotorMPC::RecordTick(const Reference_t& ref0)
{
#if CDS_RECORD_ENABLED
    const std::array<double, 32> row{{
        m_time,
        m_state[IDX_X], m_state[IDX_Y], m_state[IDX_Z],
        m_state[IDX_QW], m_state[IDX_QX], m_state[IDX_QY], m_state[IDX_QZ],
        m_state[IDX_VX], m_state[IDX_VY], m_state[IDX_VZ],
        m_state[IDX_WX], m_state[IDX_WY], m_state[IDX_WZ],
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

bool QuadRotorMPC::GetState(core_state_t& state)
{
    state.x_dot = m_state[IDX_VX]; state.y_dot = m_state[IDX_VY]; state.z_dot = m_state[IDX_VZ];
    state.x = m_state[IDX_X]; state.y = m_state[IDX_Y]; state.z = m_state[IDX_Z];
    state.roll_dot = m_state[IDX_WX]; state.pitch_dot = m_state[IDX_WY]; state.yaw_dot = m_state[IDX_WZ];

    const double qw=m_state[IDX_QW], qx=m_state[IDX_QX], qy=m_state[IDX_QY], qz=m_state[IDX_QZ];
    double sinp = 2.0*(qw*qy - qz*qx);
    if (sinp >  1.0) sinp =  1.0;
    if (sinp < -1.0) sinp = -1.0;
    state.roll  = std::atan2(2.0*(qw*qx + qy*qz), 1.0 - 2.0*(qx*qx + qy*qy));
    state.pitch = std::asin(sinp);
    state.yaw   = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    return false;
}

bool QuadRotorMPC::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x = m_trackingErr[0]; tErrors.y = m_trackingErr[1];
    tErrors.z = m_trackingErr[2]; tErrors.yaw = m_trackingErr[3];
    return false;
}

bool QuadRotorMPC::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
{
    currentTimeSeconds = m_time;
    return false;
}

// ---- exposed controller parameters: the MPC cost weights and solver knobs ----
// The weights are the per-block Gauss-Newton cost weights; dt_mpc is the control
// step (also the horizon spacing); max_iters caps the iLQR iterations per solve.
// The horizon N is runtime-tunable (buffers fixed at MAX_HORIZON). No gain to
// re-synthesise: the MPC re-solves from scratch every control step.
// Per-axis group names for the position sensor bank (x/y/z).
static const char* const SENSOR_GROUPS[3] = { "Sensor x", "Sensor y", "Sensor z" };

void QuadRotorMPC::BuildParamTable()
{
    // Model bucket: the runtime-tunable prediction horizon.
    m_modelParams.clear();
    // Active prediction horizon N (runtime-tunable up to the fixed buffer
    // capacity). A change re-seeds the warm start on the next solve.
    m_modelParams.add("solver", "horizon N", true,
        [this]{ return static_cast<double>(m_horizon); },
        [this](double v){ const int n = static_cast<int>(v + 0.5);
                          if (n < 1 || n > static_cast<int>(QuadRotorMPC::MAX_HORIZON)) return true;
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
    // Control-effort cost weight (the analogue of the LQR R).
    m_controllerParams.add("R (control cost)", "thrust", true, [this]{ return m_wu; },
                 [this](double v){ if (v <= 0.0) return true; m_wu = v; return false; });
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

bool QuadRotorMPC::GetModelManifest(char* buf, std::size_t n)      { m_modelParams.buildManifest(buf, n); return false; }
bool QuadRotorMPC::SetModelParam(int id, double value)            { return m_modelParams.set(id, value); }
bool QuadRotorMPC::GetControllerManifest(char* buf, std::size_t n){ m_controllerParams.buildManifest(buf, n); return false; }
bool QuadRotorMPC::SetControllerParam(int id, double value)      { return m_controllerParams.set(id, value); }
bool QuadRotorMPC::GetObserverManifest(char* buf, std::size_t n)  { m_observerParams.buildManifest(buf, n); return false; }
bool QuadRotorMPC::SetObserverParam(int id, double value)        { return m_observerParams.set(id, value); }
bool QuadRotorMPC::GetSensorManifest(char* buf, std::size_t n)    { m_sensorParams.buildManifest(buf, n); return false; }
bool QuadRotorMPC::SetSensorParam(int id, double value)          { return m_sensorParams.set(id, value); }

} // namespace CDS

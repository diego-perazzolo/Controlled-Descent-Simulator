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

// ---- hard-wired controller tuning (revisit for frontend exposure later) ----
constexpr double DT_MPC    = 0.02;   // MPC prediction step [s]
constexpr int    MAX_ITERS = 12;     // iLQR iterations per tick (warm-started)

using V13    = std::array<double, NX>;
using V4     = std::array<double, NU>;
using V12    = std::array<double, NR>;
using M12x13 = std::array<V13, NR>;

struct Weights { double wp, wq, wv, ww, wu, wterm; };
struct StageRef { std::array<double,3> p; std::array<double,4> q; std::array<double,3> v; std::array<double,3> w; };

const Weights W{6.0, 2.0, 1.0, 0.30, 0.10, 8.0};   // notebook defaults

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
    return false;
}

bool QuadRotorMPC::PerformIntegration(const core_stepParams_t& params)
{
    auto dynamics = (Dynamics::QUADROTOR_MPC_01*) m_modelPtr;
    if (dynamics == nullptr || m_trajectoryManagerPtr == nullptr) { return true; }

    // ---- current reference: drives the tracking errors, and is the base of the
    //      horizon preview when we re-solve ----
    Reference_t ref0;
    if (m_trajectoryManagerPtr->GetReference(m_time, ref0)) { return true; }

    // ---- tracking errors (position + heading) w.r.t. the current reference ----
    m_trackingErr[0] = ref0.pos[0] - m_state[IDX_X];
    m_trackingErr[1] = ref0.pos[1] - m_state[IDX_Y];
    m_trackingErr[2] = ref0.pos[2] - m_state[IDX_Z];
    const double qw=m_state[IDX_QW], qx=m_state[IDX_QX], qy=m_state[IDX_QY], qz=m_state[IDX_QZ];
    const double yaw = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    double eyaw = ref0.yaw - yaw; eyaw = std::atan2(std::sin(eyaw), std::cos(eyaw));
    m_trackingErr[3] = eyaw;

    // ---- re-solve the MPC only at the control cadence (every DT_MPC of model
    //      time); hold the last command as a zero-order hold in between. The
    //      horizon already assumes a DT_MPC ZOH, so this is both faithful and
    //      cheap: it keeps the expensive solve off most ticks, so a high tick
    //      rate cannot monopolise the system lock (the sim degrades gracefully
    //      instead of freezing). ----
    if (!m_seeded || (m_time - m_lastSolveTime) >= DT_MPC)
    {
        CDS_PROFILE(profile, "Total MPC execution");
        // hover command and actuator box from the model params
        using PN = Dynamics::QUADROTOR_MPC_01::ParamName;
        const double mg = dynamics->GetParam(PN::Mass) * dynamics->GetParam(PN::Gravity);
        const double Th = mg / 4.0;
        const V4 uref{{Th, Th, Th, Th}};
        V4 lo, hi;
        lo.fill(dynamics->GetParam(PN::ThrustMin));
        hi.fill(dynamics->GetParam(PN::ThrustMax));
        if (!m_seeded) { for (auto& u : m_warmStart) u = uref; }

        // sample the reference over the horizon (preview)
        std::array<StageRef, QuadRotorMPC::HORIZON + 1> refs;
        for (std::size_t k = 0; k <= QuadRotorMPC::HORIZON; ++k)
        {
            Reference_t r;
            if (m_trajectoryManagerPtr->GetReference(m_time + k * DT_MPC, r))
            {
                refs[k] = refs[k - 1];           // beyond the trajectory: hold the last (defensive)
                continue;
            }
            const double h = 0.5 * r.yaw;
            refs[k].p = {{r.pos[0], r.pos[1], r.pos[2]}};
            refs[k].q = {{std::cos(h), 0.0, 0.0, std::sin(h)}};   // level attitude at the commanded heading
            refs[k].v = {{r.vel[0], r.vel[1], r.vel[2]}};
            refs[k].w = {{0.0, 0.0, 0.0}};
        }

        // build the quad closures and hand off to the generic iLQR. The MPC
        // predicts with no external force (userF = 0); the plant is advanced
        // below with the true user force. (Offset-free force estimation lands here.)
        const std::array<double,3> predForce{{0.0, 0.0, 0.0}};
        auto fdyn  = [&](const V13& x, const V4& u){ return dynamics->Dynamics(x, u, predForce); };
        auto jac   = [&](const V13& x, const V4& u, double fx[NX][NX], double fu[NX][NU]){ dynamics->Jacobians(x, u, fx, fu); };
        auto proj  = [](V13& s){ normalizeQuat(s); };
        auto scost = [&](const V13& x, const V4& u, std::size_t k){ return quadStageCost(x, u, refs[k], uref, W); };
        auto tcost = [&](const V13& x){ return quadTermCost(x, refs[QuadRotorMPC::HORIZON], W); };

        V4 u0;
        {
            CDS_PROFILE(profile, "MPC solve");
            control::solve<NX, NU, QuadRotorMPC::HORIZON>(
                m_state, fdyn, jac, proj, scost, tcost, lo, hi, DT_MPC, MAX_ITERS, m_warmStart, u0);
        }

        m_lastU0        = u0;
        m_lastSolveTime = m_time;
        m_seeded        = true;
    }

    // ---- apply the held command (ZOH), integrate the plant by the measured step ----
    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;
    {
        CDS_PROFILE(profile, "Rk4 integration");
        m_state = integrateAndNormalize(*dynamics, m_state, m_lastU0, m_userForces, params.timestep);
    }

    m_time += params.timestep;
    return false;
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

} // namespace CDS

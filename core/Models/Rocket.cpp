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
#include <cmath>
#include <array>

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
//  [12] y1         integral of position error X (for LQR augmented state)
//  [13] y2         integral of position error Y
//  [14] y3         integral of position error Z
//
// Input vector u[4]:
//  [0]  F1   main thrust (N)       — saturated [0, F1_MAX]
//  [1]  T1   torque around X (N·m)
//  [2]  T2   torque around Y (N·m)
//  [3]  T3   torque around Z (N·m)
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

namespace CDS {

// =============================================================================
// LQR gain matrix K_e (4 x 15) — hardcoded here
// Linearized at equilibrium: alpha=0, beta=0, psi=0, F1=m*g, all rates=0
// Row i = gains for input u[i]
// =============================================================================
static constexpr std::array<std::array<double, 15>, 4> K_e = {{
    // u[0] = F1
    {  0.0,  0.0,  20.0498822436876,
       0.0,  0.0,  0.0,
       0.0,  0.0,  20.0298888992870,
       0.0,  0.0,  0.0,
       0.0,  0.0, -10.0 },
    // u[1] = T1
    {  0.00453339041286829,  0.0,  0.0,
       10.0361988950329,  0.0,  0.0,
       0.00452339043558321,  0.0,  0.0,
       8.17973059663664,  0.0,  0.0,
      -0.000010000000000,  0.0,  0.0 },
    // u[2] = T2
    {  0.0, -0.00453339041286829,  0.0,
       0.0,  10.0361988950329,  0.0,
       0.0, -0.00452339043558321,  0.0,
       0.0,  8.17973059663664,  0.0,
       0.0,  0.000010000000000,  0.0 },
    // u[3] = T3
    {  0.0,  0.0,  0.0,
       0.0,  0.0,  1.0,
       0.0,  0.0,  0.0,
       0.0,  0.0,  1.73205080756888,
       0.0,  0.0,  0.0 }
}};

// =============================================================================
// dynamics()
// Computes dxdt = f(x, u, params)
// x   : current augmented state vector (15 elements)
// u   : input vector (4 elements), F1 already saturated by caller
// ref : position reference [x_ref, y_ref, z_ref]
// p   : physical parameters
// =============================================================================
static Rocket::StateVec dynamics(const Rocket::StateVec& x,
                         const Rocket::InputVec& u,
                         const Rocket::RefVec&   ref,
                         const Rocket::PhysicsParams_t& p,
                        const Rocket::UserForces& userF)
{
    Rocket::StateVec dxdt{};

    // Precompute trig — each used multiple times
    const double sa = std::sin(x[IDX_ALPHA]),  ca = std::cos(x[IDX_ALPHA]);
    const double sb = std::sin(x[IDX_BETA]),   cb = std::cos(x[IDX_BETA]);
    const double sp = std::sin(x[IDX_PSI]),    cp = std::cos(x[IDX_PSI]);

    // Feed forward (TODO)
    const double guard = 0.001; // Avoid dividing by 0
    const double f1ff = (p.m * p.g) / (ca * cb + guard); // compensate gravity


    const double xd = x[IDX_XDOT];
    const double yd = x[IDX_YDOT];
    const double zd = x[IDX_ZDOT];
    const double F1 = u[0];
    const double T1 = u[1];
    const double T2 = u[2];
    const double T3 = u[3];

    // ------------------------------------------------------------------
    // Kinematic equations: dp/dt = v  (f[0..5])
    // ------------------------------------------------------------------
    dxdt[IDX_X]     = xd;                 // f[0]
    dxdt[IDX_Y]     = yd;                 // f[1]
    dxdt[IDX_Z]     = zd;                 // f[2]
    dxdt[IDX_ALPHA] = x[IDX_ALPHADOT];   // f[3]
    dxdt[IDX_BETA]  = x[IDX_BETADOT];    // f[4]
    dxdt[IDX_PSI]   = x[IDX_PSIDOT];     // f[5]

    // ------------------------------------------------------------------
    // Translational dynamics: dv/dt = F/m
    // Drag is expressed in body frame via Rm, projected back to inertial
    // ------------------------------------------------------------------

    // f[6]: x_ddot
    dxdt[IDX_XDOT] = -1.0 / p.m * (
          sp * sa * sb * xd * p.c
        + sp * sb * ca * zd * p.c
        + sp * cb      * yd * p.c
        - cp * sa      * zd * p.c
        + cp * ca      * xd * p.c
        - sa * cb      * F1
    );

    // f[7]: y_ddot
    dxdt[IDX_YDOT] = -1.0 / p.m * (
          cp * sa * sb * xd * p.c
        + cp * sb * ca * zd * p.c
        + sp * sa      * zd * p.c
        - sp * ca      * xd * p.c
        + cp * cb      * yd * p.c
        + sb           * F1
    );

    // f[8]: z_ddot
    dxdt[IDX_ZDOT] = -1.0 / p.m * (
          sa * cb * xd * p.cz
        + ca * cb * zd * p.cz
        - sb      * yd * p.cz
        - ca * cb * F1
        + p.m * p.g
    );

    // ------------------------------------------------------------------
    // Perturbing system with user input forces, defined in inertial frame
    // ------------------------------------------------------------------
    dxdt[IDX_XDOT] += userF[0] / p.m;
    dxdt[IDX_YDOT] += userF[1] / p.m;
    dxdt[IDX_ZDOT] += userF[2] / p.m;

    // ------------------------------------------------------------------
    // Rotational dynamics: domega/dt = T/I  (f[9..11])
    // Simple because inertia is diagonal and torques are direct inputs
    // ------------------------------------------------------------------
    dxdt[IDX_ALPHADOT] = T1 / p.Ix;   // f[9]
    dxdt[IDX_BETADOT]  = T2 / p.Iy;   // f[10]
    dxdt[IDX_PSIDOT]   = T3 / p.Iz;   // f[11]

    // ------------------------------------------------------------------
    // Augmented states: integral of position error  (y1, y2, y3)
    // ------------------------------------------------------------------
    dxdt[IDX_INTX] = ref[0] - x[IDX_X];   // ẏ1 = x_ref - x
    dxdt[IDX_INTY] = ref[1] - x[IDX_Y];   // ẏ2 = y_ref - y
    dxdt[IDX_INTZ] = ref[2] - x[IDX_Z];   // ẏ3 = z_ref - z

    return dxdt;
}

// =============================================================================
// lqr_control()
// Computes u = -K_e * x_aug, then saturates F1.
// Returns the saturated input vector.
// =============================================================================
static Rocket::InputVec lqr_control(const Rocket::StateVec& x, const Rocket::PhysicsParams_t& p)
{
    Rocket::InputVec u{};

    for (size_t i = 0; i < 4; ++i) {
        double val = 0.0;
        for (size_t j = 0; j < 15; ++j)
            val += K_e[i][j] * x[j];
        u[i] = -val;
    }

    // Saturate F1 in [0, F1_max]
    if      (u[0] > p.F1_max) u[0] = p.F1_max;
    else if (u[0] < 0.0)      u[0] = 0.0;

    return u;
}

// =============================================================================
// rk4_step()
// Advances the state by one timestep dt using classic RK4.
// Reference position is held constant over the step (ZOH).
// =============================================================================
static Rocket::StateVec rk4_step(const Rocket::StateVec&     x,
                         const Rocket::RefVec&        ref,
                         const Rocket::PhysicsParams_t& p,
                         const Rocket::UserForces& userF,
                         double               dt)
{
    // Compute control at current state (held constant over the step)
    const Rocket::InputVec u = lqr_control(x, p);

    // Four RK4 slope evaluations
    const Rocket::StateVec k1 = dynamics(x, u, ref, p, userF);

    Rocket::StateVec x2{};
    for (size_t i = 0; i < 15; ++i) x2[i] = x[i] + k1[i] * dt * 0.5;
    const Rocket::StateVec k2 = dynamics(x2, u, ref, p, userF);

    Rocket::StateVec x3{};
    for (size_t i = 0; i < 15; ++i) x3[i] = x[i] + k2[i] * dt * 0.5;
    const Rocket::StateVec k3 = dynamics(x3, u, ref, p, userF);

    Rocket::StateVec x4{};
    for (size_t i = 0; i < 15; ++i) x4[i] = x[i] + k3[i] * dt;
    const Rocket::StateVec k4 = dynamics(x4, u, ref, p, userF);

    // Weighted sum
    Rocket::StateVec x_next{};
    for (size_t i = 0; i < 15; ++i)
        x_next[i] = x[i] + (k1[i] + 2.0*k2[i] + 2.0*k3[i] + k4[i]) * dt / 6.0;

    return x_next;
}

Rocket::Rocket()
{
    // Initial state = 0;
    m_state.fill(0);
    m_trackingErr.fill(0);
    m_userForces.fill(0);

    m_state[IDX_X] = -250;
    m_state[IDX_Y] = 40;
    m_state[IDX_Z] = 200;
    m_state[IDX_ALPHA] = -0.05;
    m_state[IDX_BETA] = 3.1415/2;
    m_state[IDX_PSI] = 3;
    m_state[IDX_XDOT] = -2;
    m_state[IDX_YDOT] = 2;
    m_state[IDX_ZDOT] = -100;
    m_state[IDX_ALPHADOT] = 0;
    m_state[IDX_BETADOT] = 0;
    m_state[IDX_PSIDOT] = 0;

    m_time = 0;

}

Rocket::~Rocket()
{
}

bool Rocket::SetModelParams(core_rocketParams_t& params)
{

    m_physicsParams.m   = params.m;
    m_physicsParams.Ix  = params.Ix;
    m_physicsParams.Iy  = params.Iy;
    m_physicsParams.Iz  = params.Iz;
    m_physicsParams.g   = params.g;
    m_physicsParams.c   = params.c;
    m_physicsParams.cz  = params.cz;
    m_physicsParams.F1_max = params.F1_max;
    
    return false;
}
bool Rocket::SetTrajectory(Trajectory* pTrajectory)
{

    return false;
}

bool Rocket::PerformIntegration(core_stepParams_t& params)
{
    // Getting reference setpoints from Trajectory

    /* Variable z setpont, for having stable dynamics*/
    double refX = 0;
    double refY = 0;
    double refZ;
    if (m_state[IDX_ZDOT] < -10.0 && m_state[IDX_Z] > 50.0)
        refZ = m_state[IDX_Z] + 50.0;
    else if (m_time > 8.0 && m_time < 12.0)
        refZ = 4.0;
    else if (m_time >= 12.0)
        refZ = 0.0;
    else
        refZ = m_state[IDX_Z];   // mantieni quota corrente

    Rocket::RefVec ref = {refX, refY, refZ}; // X, Y, Z
    
    // Compute tracking errors
    m_trackingErr[0] = refX - m_state[IDX_X]; // Position X
    m_trackingErr[1] = refY - m_state[IDX_Y]; // Position Y
    m_trackingErr[2] = refZ - m_state[IDX_Z]; // Position Z

    // User forces
    m_userForces[0] = params.user_fX;
    m_userForces[1] = params.user_fY;
    m_userForces[2] = params.user_fZ;

    // Runge Kutta 4
    m_state = rk4_step(m_state, ref, m_physicsParams, m_userForces, params.timestep);
    m_time += params.timestep;

    // Log
#if 0
    InputVec u = lqr_control(m_state, m_physicsParams);
    printf("t=%.1f  x=%.2f  y=%.2f  F1=%.2f  T1=%.4f  T2=%.4f  T3=%.4f  intX=%.3f  intY=%.3f\n",
    m_time,
    m_state[IDX_X], m_state[IDX_Y],
    u[0], u[1], u[2], u[3],
    m_state[IDX_INTX], m_state[IDX_INTY]);
    //printf("user forces [N] -> x: %.1f, y: %.1f, z: %.1f\n", m_userForces[0], m_userForces[1], m_userForces[2]);
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
    state.yaw = m_state[IDX_PSIDOT];

    // IDX_INTX    
    // IDX_INTY    
    // IDX_INTZ    

    return false;
}

bool Rocket::GetTrackingErrors(core_trackingErrors_t& tErrors)
{
    tErrors.x = m_trackingErr[0];
    tErrors.y = m_trackingErr[1];
    tErrors.z = m_trackingErr[2];
    return false;
}

} // namespace CDS
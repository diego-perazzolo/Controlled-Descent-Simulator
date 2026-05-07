
#include <cstdio>
#include "dynamics_ff_lqr_01.hpp"

int main() {
    using namespace CDS;
    using namespace CDS::Dynamics;

    FF_LQR_01 model;
    model.SetParam(FF_LQR_01::ParamName::Mass,        10.0);
    model.SetParam(FF_LQR_01::ParamName::Ix,          10.0/3.0);
    model.SetParam(FF_LQR_01::ParamName::Iy,          10.0/3.0);
    model.SetParam(FF_LQR_01::ParamName::Iz,          1.0);
    model.SetParam(FF_LQR_01::ParamName::Gravity,     9.81);
    model.SetParam(FF_LQR_01::ParamName::DragLateral, 1.0);
    model.SetParam(FF_LQR_01::ParamName::DragAxial,   0.02);
    model.SetParam(FF_LQR_01::ParamName::ThrustMax,   500.0);
    model.SetParam(FF_LQR_01::ParamName::ThrustMin,   0.0);
    model.SetParam(FF_LQR_01::ParamName::TorqueXMax,   10.0);
    model.SetParam(FF_LQR_01::ParamName::TorqueXMin,   -10.0);
    model.SetParam(FF_LQR_01::ParamName::TorqueYMax,   10.0);
    model.SetParam(FF_LQR_01::ParamName::TorqueYMin,   -10.0);

    FF_LQR_01::StateVec s = {1.0, -2.0, 50.0,  0.05, -0.03, 0.0,
                             0.5, -0.1, -10.0,  0.01, -0.02, 0.0,
                             0.1, 0.2, -0.05};
    FF_LQR_01::InputVec u = {100.0, 0.5, -0.3, 0.0};
    Vec3 userF = {2.5, -1.5, 0.7};

    // Test Dynamics with a Reference_t (only ref.pos is used inside, but we
    // pass the full struct as the new signature requires).
    Reference_t ref_for_dynamics;
    ref_for_dynamics.pos = {0.0, 0.0, 0.0};
    auto dxdt = model.Dynamics(s, u, ref_for_dynamics, userF);
    for (double v : dxdt) std::printf("%.15e\n", v);

    // Test ExecuteControl with a richer Reference_t.
    Reference_t r;
    r.pos  = {10.0, -5.0, 100.0};
    r.vel  = {2.0, -1.0, -8.0};
    r.acc  = {0.5, -0.2, -3.0};
    r.jerk = {0.1, -0.05, 0.2};
    r.snap = {0.01, -0.02, 0.03};
    auto u_ctrl = model.ExecuteControl(s, r);
    for (double v : u_ctrl) std::printf("%.15e\n", v);
    return 0;
}

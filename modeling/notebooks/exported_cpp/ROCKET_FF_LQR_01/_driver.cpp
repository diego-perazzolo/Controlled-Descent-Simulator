
#include <cstdio>
#include "dynamics_rocket_ff_lqr_01.hpp"
int main() {
    using namespace CDS; using namespace CDS::Dynamics;
    ROCKET_FF_LQR_01 model;
    using PN = ROCKET_FF_LQR_01::ParamName;
    model.SetParam(PN::Mass,10.0); model.SetParam(PN::Ix,10.0/3.0); model.SetParam(PN::Iy,10.0/3.0);
    model.SetParam(PN::Iz,1.0); model.SetParam(PN::Gravity,9.81); model.SetParam(PN::DragLateral,1.0);
    model.SetParam(PN::DragAxial,0.02); model.SetParam(PN::ThrustMax,500.0); model.SetParam(PN::ThrustMin,0.0);
    model.SetParam(PN::TorqueXMax,10.0); model.SetParam(PN::TorqueXMin,-10.0);
    model.SetParam(PN::TorqueYMax,10.0); model.SetParam(PN::TorqueYMin,-10.0);
    ROCKET_FF_LQR_01::StateVec s = { 1.0, -2.0, 50.0, 0.05, -0.03, 0.0, 0.5, -0.1, -10.0, 0.01, -0.02, 0.0, 0.1, 0.2, -0.05, 0.15 };
    ROCKET_FF_LQR_01::InputVec u = { 100.0, 0.5, -0.3, 0.0 };
    Vec3 userF = { 2.5, -1.5, 0.7 };
    Reference_t rd{}; rd.pos = {0.0,0.0,0.0}; rd.yaw = 0.3;
    auto dxdt = model.Dynamics(s, u, rd, userF);
    for (double v : dxdt) std::printf("%.15e\n", v);
    Reference_t r{};
    r.pos={ 10.0, -5.0, 100.0 }; r.vel={ 2.0, -1.0, -8.0 }; r.acc={ 0.5, -0.2, -3.0 };
    r.jerk={ 0.1, -0.05, 0.2 }; r.snap={ 0.01, -0.02, 0.03 };
    r.yaw=0.4; r.yawRate=0.05; r.yawAcc=0.02;
    auto uc = model.ExecuteControl(s, r);
    for (double v : uc) std::printf("%.15e\n", v);
    return 0;
}

#include <emscripten/bind.h>
#include "ext_comm.hpp"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(simulator) {

    // --- Value types (struct) ---
    value_object<ext_fullState>("ext_fullState")
        .field("x",         &ext_fullState::x)
        .field("y",         &ext_fullState::y)
        .field("z",         &ext_fullState::z)
        .field("x_dot",     &ext_fullState::x_dot)
        .field("y_dot",     &ext_fullState::y_dot)
        .field("z_dot",     &ext_fullState::z_dot)
        .field("roll",      &ext_fullState::roll)
        .field("pitch",     &ext_fullState::pitch)
        .field("yaw",       &ext_fullState::yaw)
        .field("roll_dot",  &ext_fullState::roll_dot)
        .field("pitch_dot", &ext_fullState::pitch_dot)
        .field("yaw_dot",   &ext_fullState::yaw_dot);

    value_object<ext_setpointError>("ext_setpointError")
        .field("xErr",    &ext_setpointError::xErr)
        .field("yErr",    &ext_setpointError::yErr)
        .field("zErr",    &ext_setpointError::zErr);

    value_object<ext_rocketParams>("ext_rocketParams")
        .field("mass_Kg",        &ext_rocketParams::mass_Kg)
        .field("inertiaX_Kgm2", &ext_rocketParams::inertiaX_Kgm2)
        .field("inertiaY_Kgm2", &ext_rocketParams::inertiaY_Kgm2)
        .field("inertiaZ_Kgm2", &ext_rocketParams::inertiaZ_Kgm2)
        .field("c",              &ext_rocketParams::c)
        .field("cz",             &ext_rocketParams::cz);

    value_object<ext_traj>("ext_traj")
        .field("a0", &ext_traj::a0)
        .field("a1", &ext_traj::a1)
        .field("a2", &ext_traj::a2)
        .field("a3", &ext_traj::a3);

    value_object<ext_userForce>("ext_userForce")
        .field("fX", &ext_userForce::fX)
        .field("fY", &ext_userForce::fY)
        .field("fZ", &ext_userForce::fZ);

    value_object<ext_actuatorLimits>("ext_actuatorLimits")
        .field("fZ_lim", &ext_actuatorLimits::fZ_lim);

    value_object<ext_initParams>("ext_initParams")
        .field("rocketPar",      &ext_initParams::rocketPar)
        .field("actuatorLimits", &ext_initParams::actuatorLimits)
        .field("trajParams",     &ext_initParams::trajParams);

    value_object<ext_stepParams>("ext_stepParams")
        .field("timeStep_s", &ext_stepParams::timeStep_s)
        .field("userForce",  &ext_stepParams::userForce);

    value_object<ext_stepRet>("ext_stepRet")
        .field("isError", &ext_stepRet::isError)
        .field("state",   &ext_stepRet::state)
        .field("err",     &ext_stepRet::err);

    value_object<ext_trajectoryPoint>("ext_trajectoryPoint")
        .field("x", &ext_trajectoryPoint::x)
        .field("y",   &ext_trajectoryPoint::y)
        .field("z",     &ext_trajectoryPoint::z);

    // --- Funzioni esposte a JS ---
    function("ext_init", &ext_init);
    function("ext_step", &ext_step);
    function("ext_getTrajectoryPoint", &ext_getTrajectoryPoint);
}

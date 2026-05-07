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

    value_object<ext_userForce>("ext_userForce")
        .field("fX", &ext_userForce::fX)
        .field("fY", &ext_userForce::fY)
        .field("fZ", &ext_userForce::fZ);

    value_object<ext_actuatorLimits>("ext_actuatorLimits")
        .field("fZ_max", &ext_actuatorLimits::fZ_max)
        .field("fZ_min", &ext_actuatorLimits::fZ_min)
        .field("Tx_max", &ext_actuatorLimits::Tx_max)
        .field("Tx_min", &ext_actuatorLimits::Tx_min)
        .field("Ty_max", &ext_actuatorLimits::Ty_max)
        .field("Ty_min", &ext_actuatorLimits::Ty_min);

    value_object<ext_initParams>("ext_initParams")
        .field("rocketPar",      &ext_initParams::rocketPar)
        .field("actuatorLimits", &ext_initParams::actuatorLimits);

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

    value_object<ext_vec3_t>("ext_vec3_t")
        .field("x", &ext_vec3_t::x)
        .field("y",   &ext_vec3_t::y)
        .field("z",     &ext_vec3_t::z);

    value_object<ext_trajectoryPoly4Params_t>("ext_trajectoryPoly4Params_t")
        .field("initialPos", &ext_trajectoryPoly4Params_t::initialPos)
        .field("initialVel",   &ext_trajectoryPoly4Params_t::initialVel)
        .field("finalPos",     &ext_trajectoryPoly4Params_t::finalPos)
        .field("finalVel", &ext_trajectoryPoly4Params_t::finalVel)
        .field("finalAcc",   &ext_trajectoryPoly4Params_t::finalAcc)
        .field("time_s",     &ext_trajectoryPoly4Params_t::time_s);
    
    value_object<ext_trajectoryPointParams_t>("ext_trajectoryPointParams_t")
        .field("finalPos",     &ext_trajectoryPointParams_t::finalPos)
        .field("time_s",     &ext_trajectoryPointParams_t::time_s);

    // --- Funzioni esposte a JS ---
    function("ext_init", &ext_init);
    function("ext_step", &ext_step);
    function("ext_trajectory_get_point", &ext_trajectory_get_point);
    function("ext_trajectory_append_poly4", &ext_trajectory_append_poly4);
    function("ext_trajectory_append_point", &ext_trajectory_append_point);
    function("ext_trajectory_remove_last_item", &ext_trajectory_remove_last_item);
}

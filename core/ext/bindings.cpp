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
        .field("zErr",    &ext_setpointError::zErr)
        .field("yawErr",    &ext_setpointError::yawErr);

    value_object<ext_rocketParams>("ext_rocketParams")
        .field("mass_Kg",        &ext_rocketParams::mass_Kg)
        .field("inertiaX_Kgm2", &ext_rocketParams::inertiaX_Kgm2)
        .field("inertiaY_Kgm2", &ext_rocketParams::inertiaY_Kgm2)
        .field("inertiaZ_Kgm2", &ext_rocketParams::inertiaZ_Kgm2)
        .field("c",              &ext_rocketParams::c)
        .field("cz",             &ext_rocketParams::cz);

    value_object<ext_quadRotorParams>("ext_quadRotorParams")
        .field("mass_Kg",        &ext_quadRotorParams::mass_Kg)
        .field("inertiaX_Kgm2", &ext_quadRotorParams::inertiaX_Kgm2)
        .field("inertiaY_Kgm2", &ext_quadRotorParams::inertiaY_Kgm2)
        .field("inertiaZ_Kgm2", &ext_quadRotorParams::inertiaZ_Kgm2)
        .field("c",              &ext_quadRotorParams::c)
        .field("cz",             &ext_quadRotorParams::cz)
        .field("motorThrustCoefficient",  &ext_quadRotorParams::motorThrustCoefficient)
        .field("motorTorqueCoefficient",  &ext_quadRotorParams::motorTorqueCoefficient)
        .field("distanceBtwMotorAndCoM",  &ext_quadRotorParams::distanceBtwMotorAndCoM)
        .field("motorMomentOfInertia",    &ext_quadRotorParams::motorMomentOfInertia);

    value_object<ext_userForce>("ext_userForce")
        .field("fX", &ext_userForce::fX)
        .field("fY", &ext_userForce::fY)
        .field("fZ", &ext_userForce::fZ);

    value_object<ext_rocketActuatorLimits>("ext_rocketActuatorLimits")
        .field("fZ_max", &ext_rocketActuatorLimits::fZ_max)
        .field("fZ_min", &ext_rocketActuatorLimits::fZ_min)
        .field("Tx_max", &ext_rocketActuatorLimits::Tx_max)
        .field("Tx_min", &ext_rocketActuatorLimits::Tx_min)
        .field("Ty_max", &ext_rocketActuatorLimits::Ty_max)
        .field("Ty_min", &ext_rocketActuatorLimits::Ty_min);

    value_object<ext_initRocketParams>("ext_initRocketParams")
        .field("rocketPar",      &ext_initRocketParams::params)
        .field("rocketActuatorLimits", &ext_initRocketParams::actuatorLimits);

    value_object<ext_quadRotorActuatorLimits>("ext_quadRotorActuatorLimits")
        .field("motor_max_thrust", &ext_quadRotorActuatorLimits::motor_max_thrust)
        .field("motor_min_thrust", &ext_quadRotorActuatorLimits::motor_min_thrust);

    value_object<ext_initQuadRotorParams>("ext_initQuadRotorParams")
        .field("quadRotorPar",      &ext_initQuadRotorParams::params)
        .field("quadRotorActuatorLimits", &ext_initQuadRotorParams::actuatorLimits);

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
        .field("initialYaw", &ext_trajectoryPoly4Params_t::initialYaw)
        .field("initialVel",   &ext_trajectoryPoly4Params_t::initialVel)
        .field("initialYawRate",   &ext_trajectoryPoly4Params_t::initialYawRate)
        .field("finalPos",     &ext_trajectoryPoly4Params_t::finalPos)
        .field("finalYaw",     &ext_trajectoryPoly4Params_t::finalYaw)
        .field("finalVel", &ext_trajectoryPoly4Params_t::finalVel)
        .field("finalYawRate", &ext_trajectoryPoly4Params_t::finalYawRate)
        .field("finalAcc",   &ext_trajectoryPoly4Params_t::finalAcc)
        .field("finalYawAcc",   &ext_trajectoryPoly4Params_t::finalYawAcc)
        .field("time_s",     &ext_trajectoryPoly4Params_t::time_s);
    
    value_object<ext_trajectoryPointParams_t>("ext_trajectoryPointParams_t")
        .field("finalPos",     &ext_trajectoryPointParams_t::finalPos)
        .field("finalYaw",     &ext_trajectoryPointParams_t::finalYaw)
        .field("time_s",     &ext_trajectoryPointParams_t::time_s);

    // --- Funzioni esposte a JS ---
    function("ext_rocketInit", &ext_initRocket_FFLQR01);
    function("ext_quadRotorInit", &ext_initQuadRotor_FFLQR01);
    function("ext_step", &ext_step);
    function("ext_trajectory_get_point", &ext_trajectory_get_point);
    function("ext_trajectory_append_poly4", &ext_trajectory_append_poly4);
    function("ext_trajectory_append_point", &ext_trajectory_append_point);
    function("ext_trajectory_remove_last_item", &ext_trajectory_remove_last_item);
}

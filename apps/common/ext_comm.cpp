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
// File        : ext_comm.cpp
// Description : Direct implementation of the external communication API:
//               converts ext structs and calls straight into the core.
//               Linked by apps/wasm-only (in-browser core) and by
//               apps/ws-served/server (native core behind cds_server).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ext_comm.hpp"
#include "core.hpp"

/* Immediately return if ret == true */
#define ASSERT_FALSE(ret) if(ret) return ret 

/* Static functions */
static core_rocketParams_t _convertExtToCore_rocketParams(ext_rocketParams rPar, ext_rocketActuatorLimits aPar)
{
    core_rocketParams_t coreParam  = {};
    coreParam.m  = rPar.mass_Kg;
    coreParam.Ix = rPar.inertiaX_Kgm2;
    coreParam.Iy = rPar.inertiaY_Kgm2;
    coreParam.Iz = rPar.inertiaZ_Kgm2;
    coreParam.g = 9.81;
    coreParam.c = rPar.c;
    coreParam.cz = rPar.cz;
    coreParam.F1_max = aPar.fZ_max;
    coreParam.F1_min = aPar.fZ_min;
    coreParam.T1_max = aPar.Tx_max;
    coreParam.T1_min = aPar.Tx_min;
    coreParam.T2_max = aPar.Ty_max;
    coreParam.T2_min = aPar.Ty_min;
    coreParam.T3_max = aPar.Tz_max;
    coreParam.T3_min = aPar.Tz_min;

    return coreParam;
}

static core_quadRotorParams_t _convertExtToCore_quadRotorParams(ext_quadRotorParams rPar, ext_quadRotorActuatorLimits aPar)
{
    core_quadRotorParams_t coreParam  = {};
    coreParam.m  = rPar.mass_Kg;
    coreParam.Ix = rPar.inertiaX_Kgm2;
    coreParam.Iy = rPar.inertiaY_Kgm2;
    coreParam.Iz = rPar.inertiaZ_Kgm2;
    coreParam.g = 9.81;
    coreParam.c = rPar.c;
    coreParam.cz = rPar.cz;
    coreParam.kT = rPar.motorThrustCoefficient;
    coreParam.kQ = rPar.motorTorqueCoefficient;
    coreParam.L = rPar.distanceBtwMotorAndCoM;
    coreParam.Irot = rPar.motorMomentOfInertia;
    coreParam.Fm_max = aPar.motor_max_thrust;
    coreParam.Fm_min = aPar.motor_min_thrust;

    return coreParam;
}

static core_stepParams_t _convertExtToCore_stepParams(ext_stepParams sPar)
{
    core_stepParams_t coreParam = {};
    coreParam.timestep = sPar.timeStep_s;
    coreParam.user_fX = sPar.userForce.fX;
    coreParam.user_fY = sPar.userForce.fY;
    coreParam.user_fZ = sPar.userForce.fZ;

    return coreParam;
}

static ext_stepRet _convertCoreToExt_stepRetParams(core_state_t state, core_trackingErrors_t tErr)
{
    ext_stepRet extParam = {};

    extParam.state.x_dot = state.x_dot; 
    extParam.state.y_dot = state.y_dot; 
    extParam.state.z_dot = state.z_dot; 
    extParam.state.x = state.x; 
    extParam.state.y = state.y; 
    extParam.state.z = state.z; 
    extParam.state.roll_dot = state.roll_dot; 
    extParam.state.pitch_dot = state.pitch_dot; 
    extParam.state.yaw_dot = state.yaw_dot; 
    extParam.state.roll = state.roll; 
    extParam.state.pitch = state.pitch; 
    extParam.state.yaw = state.yaw; 

    extParam.err.xErr = tErr.x;
    extParam.err.yErr = tErr.y;
    extParam.err.zErr = tErr.z;
    extParam.err.yawErr = tErr.yaw;

    return extParam;
}

static core_trajectoryPoly4Params_t _convertExtToCore_trajectoryPoly4Params(ext_trajectoryPoly4Params_t ext)
{
    core_trajectoryPoly4Params_t coreParam = {
        .initialPos = {ext.initialPos.x, ext.initialPos.y, ext.initialPos.z},
        .initialYaw = ext.initialYaw,
        .initialVel = {ext.initialVel.x, ext.initialVel.y, ext.initialVel.z},
        .initialYawRate = ext.initialYawRate,
        .finalPos = {ext.finalPos.x, ext.finalPos.y, ext.finalPos.z},
        .finalYaw = ext.finalYaw,
        .finalVel = {ext.finalVel.x, ext.finalVel.y, ext.finalVel.z},
        .finalYawRate = ext.finalYawRate,
        .finalAcc = {ext.finalAcc.x, ext.finalAcc.y, ext.finalAcc.z},
        .finalYawAcc = ext.finalYawAcc,
        .time_s = ext.time_s
    };

    return coreParam;
}

static core_trajectoryPointParams_t _convertExtToCore_trajectoryPointParams(ext_trajectoryPointParams_t ext)
{
    core_trajectoryPointParams_t coreParam = {
        .finalPos = {ext.finalPos.x, ext.finalPos.y, ext.finalPos.z},
        .finalYaw = ext.finalYaw,
        .time_s = ext.time_s
    };

    return coreParam;
}

static ext_trajectoryPoint _convertExtToCore_trajectoryPoint(Vec3& point)
{
    ext_trajectoryPoint extParam = {.x = (ext_coord_t) point[0], .y = (ext_coord_t) point[1], .z = (ext_coord_t) point[2]};

    return extParam;
}

/* ext functions */

bool ext_initRocket_FFLQR01(ext_initRocketParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_rocketParams_t rPar = _convertExtToCore_rocketParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    ASSERT_FALSE(ret);

    // Rocket initialization
    ret = core_rocketFfLqr01_init(rPar);
    ASSERT_FALSE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    ASSERT_FALSE(ret);

    return ret;
}

bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_quadRotorParams_t rPar = _convertExtToCore_quadRotorParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    ASSERT_FALSE(ret);

    // QuadRotor initialization
    ret = core_quadRotorFfLqr01_init(rPar);
    ASSERT_FALSE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    ASSERT_FALSE(ret);

    return ret;
}

ext_stepRet ext_step(ext_stepParams stepParams)
{
    /* Executes one integration step with the simulation, returns system state, tracking errors */
    ext_stepRet ret = {};
    core_state_t coreState;
    core_trackingErrors_t coreTrackingErr;

    // Struct conversion
    core_stepParams_t corePar = _convertExtToCore_stepParams(stepParams);

    // Integration step
    ret.isError = core_performSimulationStep(corePar);

    // Get system data
    ret.isError |= core_getState(&coreState);
    ret.isError |= core_getTrackingError(&coreTrackingErr);

    // Return if error
    if(ret.isError)
    {
        return ret;
    }

    // Struct conversion
    ret = _convertCoreToExt_stepRetParams(coreState, coreTrackingErr);

    return ret;
}

bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params)
{
    bool ret = false;

    core_trajectoryPoly4Params_t core_params = _convertExtToCore_trajectoryPoly4Params(params);

    if(core_trajectoryAppendPoly4(core_params))
    {
        // Err

        ret = true;
    }
    
    return ret;
}

bool ext_trajectory_append_point(ext_trajectoryPointParams_t params)
{
     bool ret = false;

    core_trajectoryPointParams_t core_params = _convertExtToCore_trajectoryPointParams(params);

    if(core_trajectoryAppendPoint(core_params))
    {
        // Err
        
        ret = true;
    }
    
    return ret;
}

bool ext_trajectory_remove_last_item(void)
{
    bool ret = false;

    if(core_trajectoryRemoveLastItem())
    {
        // Err
        
        ret = true;
    }
    
    return ret;
}

ext_trajectoryPoint ext_trajectory_get_point(ext_coord_t t)
{
    Vec3 p = {};
    if(core_getTrajectoryPoint(t, p))
    {
        // Err
        return {};
    }

    return _convertExtToCore_trajectoryPoint(p);
}
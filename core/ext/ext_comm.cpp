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
// File        : <filename.cpp>
// Description : <brief description of this file>
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ext_comm.hpp"
#include "../core.hpp"

/* Immediately return if ret == true */
#define ASSERT_FALSE(ret) if(ret) return ret 

/* Static functions */
static core_rocketParams_t _convertExtToCore_rocketParams(ext_rocketParams rPar, ext_actuatorLimits aPar)
{
    core_rocketParams_t coreParam  = {};
    coreParam.m  = rPar.mass_Kg;
    coreParam.Ix = rPar.inertiaX_Kgm2;
    coreParam.Iy = rPar.inertiaY_Kgm2;
    coreParam.Iz = rPar.inertiaZ_Kgm2;
    coreParam.g = 9.81;
    coreParam.c = rPar.c;
    coreParam.cz = rPar.cz;
    coreParam.F1_max = aPar.fZ_lim;

    return coreParam;
}

static core_trajectoryParams_t _convertExtToCore_trajectoryParams(ext_traj tPar)
{
    core_trajectoryParams_t coreParam = {};

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

    return extParam;
}

/* ext functions */

bool ext_init(ext_initParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_rocketParams_t rPar = _convertExtToCore_rocketParams(params.rocketPar, params.actuatorLimits);
    core_trajectoryParams_t tPar = _convertExtToCore_trajectoryParams(params.trajParams);

    // Core initialization
    bool ret = core_init();
    ASSERT_FALSE(ret);

    // Rocket initialization
    ret = core_rocketInit(rPar);
    ASSERT_FALSE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit(tPar);
    ASSERT_FALSE(ret);

    return ret;
}

ext_stepRet ext_step(ext_stepParams stepParams)
{
    /* Executes one integratiion step with the simulation, returns system state, tracking errors */
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
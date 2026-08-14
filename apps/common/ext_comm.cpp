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
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ext_comm.hpp"
#include "core.hpp"

/* Immediately return if ret == true */
#define RETURN_IF_TRUE(ret) if(ret) return ret 

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

static core_systemParams_t _convertExtToCore_systemParams(ext_systemParams& ext)
{
    core_systemParams_t coreParam = {
        .timestep_seconds = ext.timestep_seconds,
        .user_fX = ext.user_forces.fX,
        .user_fY = ext.user_forces.fY,
        .user_fZ = ext.user_forces.fZ,
    };

    return coreParam; 
}

static ext_snapshotData _convertCoreToExt_snapshotParams(core_snapshotData_t& core)
{
    ext_snapshotData ext = {};

    ext.time_seconds = core.time_seconds;
    ext.state.x_dot = core.state.x_dot; 
    ext.state.y_dot = core.state.y_dot; 
    ext.state.z_dot = core.state.z_dot; 
    ext.state.x = core.state.x; 
    ext.state.y = core.state.y; 
    ext.state.z = core.state.z; 
    ext.state.roll_dot = core.state.roll_dot; 
    ext.state.pitch_dot = core.state.pitch_dot; 
    ext.state.yaw_dot = core.state.yaw_dot; 
    ext.state.roll = core.state.roll; 
    ext.state.pitch = core.state.pitch; 
    ext.state.yaw = core.state.yaw; 

    ext.err.xErr = core.errors.x;
    ext.err.yErr = core.errors.y;
    ext.err.zErr = core.errors.z;
    ext.err.yawErr = core.errors.yaw;

    return ext;
}

/* ext functions */

bool ext_initRocket_FFLQR01(ext_initRocketParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_rocketParams_t rPar = _convertExtToCore_rocketParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // Rocket initialization
    ret = core_rocketFfLqr01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_initQuadRotor_FFLQR01(ext_initQuadRotorParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion
    core_quadRotorParams_t rPar = _convertExtToCore_quadRotorParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // QuadRotor initialization
    ret = core_quadRotorFfLqr01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_initQuadRotor_MPC01(ext_initQuadRotorParams params)
{

    /* Initialize core, return true if error */

    // Struct conversion (same physical params as the FF+LQR quadrotor)
    core_quadRotorParams_t rPar = _convertExtToCore_quadRotorParams(params.params, params.actuatorLimits);

    // Core initialization
    bool ret = core_init();
    RETURN_IF_TRUE(ret);

    // QuadRotor MPC initialization
    ret = core_quadRotorMPC01_init(rPar);
    RETURN_IF_TRUE(ret);

    // Trajectory initialization
    ret = core_trajectoryInit();
    RETURN_IF_TRUE(ret);

    return ret;
}

bool ext_trajectory_append_poly4(ext_trajectoryPoly4Params_t params)
{
    bool ret = false;

    core_trajectoryPoly4Params_t core_params = _convertExtToCore_trajectoryPoly4Params(params);

    ret = core_trajectoryAppendPoly4(core_params);
    
    return ret;
}

bool ext_trajectory_append_point(ext_trajectoryPointParams_t params)
{
     bool ret = false;

    core_trajectoryPointParams_t core_params = _convertExtToCore_trajectoryPointParams(params);

    ret = core_trajectoryAppendPoint(core_params);
    
    return ret;
}

bool ext_trajectory_remove_last_item(void)
{
    bool ret = false;

    ret = core_trajectoryRemoveLastItem();

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

bool ext_setSystemParams(ext_systemParams params)
{
    // Struct conversion
    core_systemParams_t corePar = _convertExtToCore_systemParams(params);

    return core_setSystemParams(corePar);
}

ext_snapshotData ext_getSnapshot(void)
{
    core_snapshotData_t corePar = {};
    bool ret = core_getSnapshot(corePar);

    ext_snapshotData extPar = _convertCoreToExt_snapshotParams(corePar);
    extPar.isError = ret;

    return extPar;
}


ext_plantSnapshotData ext_getPlantSnapshot(void)
{
    core_plantSnapshotData_t corePar = {};
    bool ret = core_getPlantSnapshot(corePar);

    ext_plantSnapshotData extPar = {};
    extPar.time_seconds = corePar.time_seconds;
    /* float carries the sequence exactly up to 2^24 samples */
    extPar.sequence = (ext_coord_t)corePar.sequence;

    extPar.state.x_dot = corePar.state.x_dot;
    extPar.state.y_dot = corePar.state.y_dot;
    extPar.state.z_dot = corePar.state.z_dot;
    extPar.state.x = corePar.state.x;
    extPar.state.y = corePar.state.y;
    extPar.state.z = corePar.state.z;
    extPar.state.roll_dot = corePar.state.roll_dot;
    extPar.state.pitch_dot = corePar.state.pitch_dot;
    extPar.state.yaw_dot = corePar.state.yaw_dot;
    extPar.state.roll = corePar.state.roll;
    extPar.state.pitch = corePar.state.pitch;
    extPar.state.yaw = corePar.state.yaw;

    extPar.isAttached = corePar.isAttached;
    extPar.isReadyToStart = corePar.isReadyToStart;
    extPar.isError = ret;

    return extPar;
}

bool ext_run(void)
{
    return core_run();
}

bool ext_stop(void)
{
    return core_stop();
}

bool ext_beginStaging(ext_coord_t safetyAltitude)
{
    return core_beginStaging(safetyAltitude);
}

bool ext_stopStaging(void)
{
    return core_stopStaging();
}

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
// File        : core.cpp
// Description : Core simulation API — model lifecycle, trajectory management
//               and integration stepping
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include <mutex>

#include "core.hpp"
#include "Models/Rocket.hpp"
#include "Models/QuadRotor.hpp"
#include "Trajectory/TrajectoryManager.hpp"

using namespace CDS;

struct
{
    BaseModel *pModel;
    TrajectoryManager *pTrajectoryManager;
    std::mutex mtx;

    // System parameters
    bool run;
    core_coord_t tickPeriod_seconds;
    Vec3 userForces;
} _ctx = {};

/* private types */

/* static functions */

/* global functions */

/* Updates the system of just one tick, which is dt_seconds long. Returns true on error
    ATTENTION: this function is intended to be called at pace,
        from a proper Real-Time thread. It is NOT suited to be
        called from a frontend via ext_... layer. This function
        handles physics integration, plant communication, near-safety
        functionalities; it should be treated accordingly */
bool g_core_tick(core_coord_t dt_seconds)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // No model initialized, error
        return true;
    }

    if(!_ctx.run)
    {
        // System is not running
        return false;
    }

    // Compute safety considerations

    // Drive the plant, using past control data

    // Collect data from the plant

    // Perform filtering

    // Compute control

    // Perform model integration
    _ctx.pModel->PerformIntegration({.timestep = dt_seconds,
                                     .user_fX = _ctx.userForces[0],
                                     .user_fY = _ctx.userForces[1],
                                     .user_fZ = _ctx.userForces[2]});

    return false;
}

/* Get system's tick period expressed in seconds. Returns true on error */
bool g_core_getTickPeriod(core_coord_t &tickPeriod_second)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // No model initialized, error
        return true;
    }

    tickPeriod_second = _ctx.tickPeriod_seconds;
    return false;
}

/* public functions */

bool core_init()
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    _ctx.run = false;
    return false;
}

bool core_rocketFfLqr01_init(const core_rocketParams_t rPar)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pModel)
    {
        delete _ctx.pModel;
        _ctx.pModel = nullptr;
    }

    // A fresh model must never inherit a running tick loop
    _ctx.run = false;

    _ctx.pModel = new Rocket();

    // Initializing rocket's parameters
    return _ctx.pModel->SetModelParams(rPar);
}

bool core_quadRotorFfLqr01_init(const core_quadRotorParams_t rPar)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pModel)
    {
        delete _ctx.pModel;
        _ctx.pModel = nullptr;
    }

    // A fresh model must never inherit a running tick loop
    _ctx.run = false;

    _ctx.pModel = new QuadRotor();

    // Initializing quadrotor's parameters
    return _ctx.pModel->SetModelParams(rPar);
}

bool core_trajectoryInit()
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pTrajectoryManager)
    {
        delete _ctx.pTrajectoryManager;
        _ctx.pTrajectoryManager = nullptr;
    }

    _ctx.pTrajectoryManager = new TrajectoryManager();

    return false;
}

bool core_trajectoryAppendPoly4(const core_trajectoryPoly4Params_t tPar)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pTrajectoryManager == nullptr || _ctx.pModel == nullptr)
    {
        // ERR
        return true;
    }

    _ctx.pTrajectoryManager->AppendPoly4(tPar);
    _ctx.pModel->SetTrajectoryManager(_ctx.pTrajectoryManager);

    return false;
}

bool core_trajectoryAppendPoint(const core_trajectoryPointParams_t tPar)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pTrajectoryManager == nullptr || _ctx.pModel == nullptr)
    {
        // ERR
        return true;
    }

    _ctx.pTrajectoryManager->AppendPoint(tPar);
    _ctx.pModel->SetTrajectoryManager(_ctx.pTrajectoryManager);

    return false;
}

bool core_trajectoryRemoveLastItem(void)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (_ctx.pTrajectoryManager == nullptr)
    {
        // ERR
        return true;
    }

    return _ctx.pTrajectoryManager->RemoveLastItem();
}

bool core_getTrajectoryPoint(core_coord_t time, Vec3 &point)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    Reference_t ref;
    if (_ctx.pTrajectoryManager == nullptr || _ctx.pTrajectoryManager->GetReference(time, ref))
    {
        // Error
        return true;
    }

    point[0] = ref.pos[0];
    point[1] = ref.pos[1];
    point[2] = ref.pos[2];

    return false;
}

bool core_setSystemParams(const core_systemParams_t &par)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // Err
        return true;
    }

    _ctx.tickPeriod_seconds = par.timestep_seconds;
    _ctx.userForces[0] = par.user_fX;
    _ctx.userForces[1] = par.user_fY;
    _ctx.userForces[2] = par.user_fZ;

    return false;
}

bool core_getSnapshot(core_snapshotData_t &par)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // Err
        return true;
    }

    if(_ctx.pModel->GetState(par.state) || 
        _ctx.pModel->GetTrackingErrors(par.errors) ||
        _ctx.pModel->GetCurrentTimeSeconds(par.time_seconds))
    {
        // Err
        return true;
    }

    return false;
}

bool core_run(void)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // Err
        return true;
    }

    _ctx.run = true;

    return false;
}

bool core_stop(void)
{
    std::lock_guard<std::mutex> lock(_ctx.mtx);

    if (!_ctx.pModel)
    {
        // Err
        return true;
    }

    _ctx.run = false;
    return false;
}

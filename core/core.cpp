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

#include "log.hpp"
#include "profile.hpp"
#include "core.hpp"
#include "SystemManager.hpp"
#include "Models/Rocket.hpp"
#include "Models/QuadRotor.hpp"
#include "Models/QuadRotorMPC.hpp"
#include "Trajectory/TrajectoryManager.hpp"

using namespace CDS;

static const auto logger = cds_log::registry().module("Core");
static const auto profile = cds_profile::registry().module("Core");

struct
{
    SystemManager SM;
} _ctx = {};

/* private types */

/* static functions */

/* global functions */

/* Updates the system of just one tick, which is dt_seconds long. Returns true on error
    ATTENTION: this function is intended to be called at pace,
        from a proper Real-Time thread. It is NOT suited to be
        called from a frontend via ext_... layer. This function
        handles physics integration, plant interactions, near-safety
        functionalities; it should be treated accordingly */
bool g_core_tick(core_coord_t dt_seconds)
{
    return _ctx.SM.ExecuteTick(dt_seconds);
}

/* Get system's tick period expressed in seconds. Returns true on error */
bool g_core_getTickPeriod(core_coord_t &tickPeriod_second)
{
    SystemManager::systemManagerParams_t params;
    bool ret = _ctx.SM.GetParameters(params);
    tickPeriod_second = params.timestep_seconds;

    return ret;
}

/* Attach new plant to the System Manager */
bool g_core_attachPlant(std::unique_ptr<BasePlant> plant)
{
    return _ctx.SM.AttachPlant(std::move(plant));
}

/* Detach current plant from system manager*/
bool g_core_detachPlant()
{
    return _ctx.SM.DetachPlant();
}


/* public functions */

bool core_init()
{
    return false;
}

bool core_rocketFfLqr01_init(const core_rocketParams_t rPar)
{
    // Configure the model. The SystemManager always 
    // receives a fully-initialized model
    auto model = std::make_unique<Rocket>();
    if (model->SetModelParams(rPar))
    {
        CDS_LOG_ERROR(logger, "Cannot initialize Rocket FF LQR model");
        return true;
    }

    return _ctx.SM.InitModel(std::move(model));
}

bool core_quadRotorFfLqr01_init(const core_quadRotorParams_t rPar)
{
    // Configure the model. The SystemManager always 
    // receives a fully-initialized model
    auto model = std::make_unique<QuadRotor>();
    if (model->SetModelParams(rPar))
    {
        CDS_LOG_ERROR(logger, "Cannot initialize Quadrotor FF LQR model");
        return true;
    }

    return _ctx.SM.InitModel(std::move(model));
}

bool core_quadRotorMPC01_init(const core_quadRotorParams_t rPar)
{
    // Configure the model. The SystemManager always
    // receives a fully-initialized model
    auto model = std::make_unique<QuadRotorMPC>();
    if (model->SetModelParams(rPar))
    {
        CDS_LOG_ERROR(logger, "Cannot initialize Quadrotor MPC model");
        return true;
    }

    return _ctx.SM.InitModel(std::move(model));
}

bool core_trajectoryInit()
{
    return _ctx.SM.InitTrajectory();
}

bool core_trajectoryAppendPoly4(const core_trajectoryPoly4Params_t tPar)
{
    return _ctx.SM.MutateTrajectoryManager([tPar](TrajectoryManager &tM)
                                  { return tM.AppendPoly4(tPar);});
}

bool core_trajectoryAppendPoint(const core_trajectoryPointParams_t tPar)
{
    return _ctx.SM.MutateTrajectoryManager([tPar](TrajectoryManager &tM)
                                  { return tM.AppendPoint(tPar);});
}

bool core_trajectoryRemoveLastItem(void)
{
    return _ctx.SM.MutateTrajectoryManager([](TrajectoryManager &tM)
                                  { return tM.RemoveLastItem();});
}

bool core_getTrajectoryPoint(core_coord_t time, Vec3 &point)
{
    Reference_t ref;
    if (_ctx.SM.ExecuteOnTrajectoryManager([time, &ref](const TrajectoryManager &tM)
                                  { return tM.GetReference(time, ref);}))
    {
        CDS_LOG_ERROR(logger, "Cannot obtain trajectory reference");
        return true;
    }

    point[0] = ref.pos[0];
    point[1] = ref.pos[1];
    point[2] = ref.pos[2];

    return false;
}

bool core_setSystemParams(const core_systemParams_t &par)
{
    SystemManager::userForces_t uF = {par.user_fX, par.user_fY, par.user_fZ};

    bool ret = _ctx.SM.SetParameters({.timestep_seconds = par.timestep_seconds});
    ret |= _ctx.SM.SetUserForces(uF);

    return ret;
}

bool core_getSnapshot(core_snapshotData_t &par)
{
    CDS_PROFILE(profile, "Get system snapshot");
    return _ctx.SM.ExecuteOnModel([&par](BaseModel &model)
                                  {
                                    bool ret = model.GetState(par.state);
                                    ret |= model.GetTrackingErrors(par.errors);
                                    ret |= model.GetCurrentTimeSeconds(par.time_seconds);

                                    return ret;
                                });
}

bool core_getPlantSnapshot(core_plantSnapshotData_t &par)
{
    CDS_PROFILE(profile, "Get plant snapshot");
    par.isAttached = false;

    return _ctx.SM.ExecuteOnPlant([&par](BasePlant &plant)
        {
        /* the lambda only runs with a plant attached */
        par.isAttached = true;
        par.isReadyToStart = plant.IsReadyToStart();

        /* one Pull gets time, sequence and state as
            a single coherent sample */
        BasePlant::plantMeasurements_t measurements = {};
        if (plant.PullMeasurements(measurements))
        {
            CDS_LOG_WARN(logger, "No plant sample available yet");
            return true;
        }

        par.time_seconds = measurements.plantTime_seconds;
        par.sequence = measurements.sequence;
        par.state = measurements.state;

        return false;
    });
}

bool core_run(void)
{
    return _ctx.SM.Run();
}

bool core_stop(void)
{
    return _ctx.SM.Stop();
}

bool core_beginStaging(core_coord_t safetyAltitude)
{
    return _ctx.SM.BeginStaging(safetyAltitude);
}

bool core_stopStaging(void)
{
    return _ctx.SM.StopStaging();
}
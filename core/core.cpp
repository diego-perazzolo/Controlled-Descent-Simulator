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
#include "core.hpp"
#include "Models/Rocket.hpp"
#include "Trajectory/TrajectoryManager.hpp"

using namespace CDS;

struct 
{
    BaseModel* pModel;
    TrajectoryManager* pTrajectoryManager;
} _ctx = {};


/* private types */

/* static functions */

/* public functions */

bool core_init()
{

    return false;
}

bool core_rocketInit(const core_rocketParams_t rPar)
{
    if(_ctx.pModel)
    {
        delete _ctx.pModel;
        _ctx.pModel = nullptr;
    }
    

    _ctx.pModel = new Rocket();
    
    // Initializing rocket's parameters
    return _ctx.pModel->SetModelParams(rPar);
}

bool core_trajectoryInit()
{
    if(_ctx.pTrajectoryManager)
    {
        delete _ctx.pTrajectoryManager;
        _ctx.pTrajectoryManager = nullptr;
    }

    _ctx.pTrajectoryManager = new TrajectoryManager();

    // Default trajectory
 #if 0
    const core_trajectoryPoly4Params_t poly4Params = {
        .initialPos = {-50, 50, 80},
        .initialVel = {0, 5, -50},
        .finalPos = {0, 0, 0},
        .finalVel = {0, 0, 0},
        .finalAcc = {0, 0, 0},
        .time_s = 20
    };

    const core_trajectoryPoly4Params_t poly4Params2 = {
        .initialPos = {0, 0, 0},
        .initialVel = {10, 0, 0},
        .finalPos = {-50, 50, 80},
        .finalVel = {0, 0, 0},
        .finalAcc = {0, 0, 0},
        .time_s = 10
    };

    bool ret = true;

    ret &= core_trajectoryAppendPoly4(poly4Params);
    ret &= core_trajectoryAppendPoly4(poly4Params2);
   
    for(int i = 0; i < 1000; i++)
    {
        core_trajectoryPointParams_t pointParams1;
        const core_coord_t totalTime_s = 10;
        const core_coord_t timeStep = totalTime_s / 1000;
        pointParams1.finalPos[0] = 1 * i * timeStep;
        pointParams1.finalPos[1] = 2 * i * timeStep;
        pointParams1.finalPos[2] = 3 * i * timeStep;
        pointParams1.time_s = timeStep;

        ret &= core_trajectoryAppendPoint(pointParams1);
    }
#endif

    return false;
}

bool core_trajectoryAppendPoly4(const core_trajectoryPoly4Params_t tPar)
{
    if(_ctx.pTrajectoryManager == nullptr)
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
        if(_ctx.pTrajectoryManager == nullptr)
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
    if(_ctx.pTrajectoryManager == nullptr)
    {
        // ERR
        return true;
    }

    return _ctx.pTrajectoryManager->RemoveLastItem();
}

bool core_performSimulationStep(const core_stepParams_t sPar)
{
    return _ctx.pModel->PerformIntegration(sPar);
}

bool core_getState(core_state_t *pState)
{
    if(pState == nullptr)
    {
        // nullptr
        return true;
    }
    
   return _ctx.pModel->GetState(*pState);
}

bool core_getTrackingError(core_trackingErrors_t *pTrackingErr)
{
    return _ctx.pModel->GetTrackingErrors(*pTrackingErr);
}

bool core_getTrajectoryPoint(core_coord_t time, Vec3& point)
{
    Reference_t ref;
    if(_ctx.pTrajectoryManager == nullptr || _ctx.pTrajectoryManager->GetReference(time, ref))
    {
        // Error
        return true;
    }

    point[0] = ref.pos[0];
    point[1] = ref.pos[1];
    point[2] = ref.pos[2];

    return false;
}
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
#include "Trajectory/Poly4.hpp"

using namespace CDS;

struct 
{
    BaseModel* pModel;
    Trajectory* pTrajectory;
} _ctx = {};


/* private types */

/* static functions */

/* public functions */

bool core_init()
{
    // Context initialization
    if(_ctx.pModel)
    {
        delete _ctx.pModel;
    }

    if(_ctx.pTrajectory)
    {
        delete _ctx.pTrajectory;
    }

    _ctx.pModel = new Rocket();
    _ctx.pTrajectory = new Poly4();
    _ctx.pModel->SetTrajectory(_ctx.pTrajectory);


    return false;
}

bool core_rocketInit(core_rocketParams_t rPar)
{
    // Initializing rocket's parameters
    return _ctx.pModel->SetModelParams(rPar);
}

bool core_trajectoryInit(core_trajectoryParams_t tPar)
{

    return false;
}

bool core_performSimulationStep(core_stepParams_t sPar)
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
    if(_ctx.pTrajectory == nullptr || _ctx.pTrajectory->GetReference(time, ref))
    {
        // Error
        return true;
    }

    point[0] = ref.pos[0];
    point[1] = ref.pos[1];
    point[2] = ref.pos[2];

    return false;
}
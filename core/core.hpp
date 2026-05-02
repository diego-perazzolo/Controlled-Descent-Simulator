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
#pragma once
#include "core_defs.hpp"

/* Public C-like interface */

// Initializes the simulation core, returns true on error
bool core_init(void);

// Set rocket parameters, returns true on error
bool core_rocketInit(const core_rocketParams_t rPar);

// Set trajectory initial parameters, returns true on error
bool core_trajectoryInit(void);

// Append to the trajectory list an item of type: 4th order Polynomial trajectory, returns true on error
bool core_trajectoryAppendPoly4(const core_trajectoryPoly4Params_t tPar);

// Append to the trajectory list an item of type: Point, returns true on error
bool core_trajectoryAppendPoint(const core_trajectoryPointParams_t tPar);

// Remove last trajectory item, returns true on error
bool core_trajectoryRemoveLastItem(void);

// Performs one simulation (integration) step
bool core_performSimulationStep(const core_stepParams_t sPar);

// Get system's state, returns true on error
bool core_getState(core_state_t *pState);

// Get tracking error, returns true on error
bool core_getTrackingError(core_trackingErrors_t *pTrackingErr);

// Get Trajectory point at a certain time instant, returns true on error
bool core_getTrajectoryPoint(core_coord_t time, Vec3& point);
    
    

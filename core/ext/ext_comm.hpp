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
#include "ext_defs.hpp"

/* structs */

/* struct of the arguments of init core function */
typedef struct 
{
    ext_rocketParams rocketPar;
    ext_actuatorLimits actuatorLimits;
    ext_traj trajParams;
} ext_initParams;

/* struct of the arguments of step function */
typedef struct
{
    ext_coord_t timeStep_s;
    ext_userForce userForce;
} ext_stepParams;

/* struct of the return data of the step function */
typedef struct 
{
    bool isError;
    ext_fullState state;
    ext_setpointError err;
} ext_stepRet;

/* Init function: initializes the core */
bool ext_init(ext_initParams params);

/* Step function, used to progress one step with the simulation. Returns a struct
filled with the whole system state, setpoint errors*/
ext_stepRet ext_step(ext_stepParams);

/* Get a point at time instant t along the trajectory */
ext_trajectoryPoint ext_getTrajectoryPoint(ext_coord_t t);
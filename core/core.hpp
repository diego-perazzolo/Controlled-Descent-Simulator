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
// File        : core.hpp
// Description : header file of core functionalities
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include "core_defs.hpp"
#include <cstddef>

/* Public C-like interface */

// Initializes the simulation core, returns true on error
bool core_init(void);

// Set rocket FFLQR01 parameters, returns true on error
bool core_rocketFfLqr01_init(const core_rocketParams_t rPar);

// Set quadRotor FFLQR01 parameters, returns true on error
bool core_quadRotorFfLqr01_init(const core_quadRotorParams_t rPar);

bool core_quadRotorMPC01_init(const core_quadRotorParams_t rPar);

// Set rocket MPC01 (nonlinear MPC) parameters, returns true on error
bool core_rocketMPC01_init(const core_rocketParams_t rPar);

// Set trajectory initial parameters, returns true on error
bool core_trajectoryInit(void);

// Append to the trajectory list an item of type: 4th order Polynomial trajectory, returns true on error
bool core_trajectoryAppendPoly4(const core_trajectoryPoly4Params_t tPar);

// Append to the trajectory list an item of type: Point, returns true on error
bool core_trajectoryAppendPoint(const core_trajectoryPointParams_t tPar);

// Remove last trajectory item, returns true on error
bool core_trajectoryRemoveLastItem(void);

// Get Trajectory point at a certain time instant, returns true on error
bool core_getTrajectoryPoint(core_coord_t time, Vec3& point);

// Set systems parameters (timestep, user forces). Returns true on error 
bool core_setSystemParams(const core_systemParams_t& par);

// Get system snapshot (time, state, tracking errors). Returns true on error
bool core_getSnapshot(core_snapshotData_t& par);

// Get plant snapshot (last sample: plant time, sequence, state). Returns
// true on error (no plant attached, or no sample published yet — the
// isAttached field tells which)
bool core_getPlantSnapshot(core_plantSnapshotData_t& par);

// Run simulation / plant ticking. Returns true on error
bool core_run(void);

// Stop simulation / plant ticking. Returns true on error
bool core_stop(void);

// Auto-stage the plant to (trajectory vertical range + safetyAltitude).
// Returns true on error
bool core_beginStaging(core_coord_t safetyAltitude);

// Abort auto-staging (hold in place). Returns true on error
bool core_stopStaging(void);

// Get the active controller's parameter manifest (a TSV listing of the exposed
// parameters) into `buf` (capacity `n`). Returns true on error.
bool core_getControllerManifest(char* buf, std::size_t n);

// Set one controller parameter, identified by its manifest id, to `value`.
// Returns true on error (no model, bad id, read-only or rejected value).
bool core_setControllerParam(int id, double value);

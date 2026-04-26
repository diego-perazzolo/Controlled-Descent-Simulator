
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
#include <array>

/* Types */
typedef double core_coord_t; // type for running coordinartes: position, pose, force
typedef std::array<core_coord_t, 3> Vec3;

// Rocket params
typedef struct
{
    core_coord_t m ;   // mass (kg)
    core_coord_t Ix;  // moment of inertia X (kg·m²)
    core_coord_t Iy;  // moment of inertia Y (kg·m²)
    core_coord_t Iz;   // moment of inertia Z (kg·m²)
    core_coord_t g;   // gravity (m/s²)
    core_coord_t c;   // lateral aerodynamic drag coefficient
    core_coord_t cz;   // axial aerodynamic drag coefficient
    core_coord_t F1_max; // maximum thrust (N)
} core_rocketParams_t;

// Trajectory params
typedef struct
{
    
} core_trajectoryParams_t;


typedef struct{
    Vec3 pos;
    Vec3 vel;
    Vec3 acc;
    Vec3 jerk;
    Vec3 snap;
} Reference_t;

// Integrations' step parameters
typedef struct 
{
    core_coord_t timestep;
    core_coord_t user_fX;
    core_coord_t user_fY;
    core_coord_t user_fZ;
} core_stepParams_t;

/* struct containing forces and torques computed by control*/
typedef struct
{
    /* position of main thruster's force application point*/
    core_coord_t f_posX;
    core_coord_t f_posY;
    core_coord_t f_posZ;

    /* main thruster force */
    core_coord_t fX;
    core_coord_t fY;
    core_coord_t fZ;

    /* controlled torques */
    core_coord_t tX;
    core_coord_t tY;
    core_coord_t tZ;

} core_actuators;


// System's state (position, velocity, ...)
typedef struct 
{
    /* velocities */
    core_coord_t x_dot;
    core_coord_t y_dot;
    core_coord_t z_dot;
    /* Position */
    core_coord_t x;
    core_coord_t y;
    core_coord_t z;

    /* angular velocities*/
    core_coord_t roll_dot;
    core_coord_t pitch_dot;
    core_coord_t yaw_dot;
    /* angles */
    core_coord_t roll;
    core_coord_t pitch;
    core_coord_t yaw;
} core_state_t;

// System tracking errors
typedef struct 
{
    core_coord_t x;
    core_coord_t y;
    core_coord_t z;
} core_trackingErrors_t;

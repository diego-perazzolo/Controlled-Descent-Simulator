
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
/* Types */
typedef float ext_coord_t; // type for running coordinartes: position, pose, force

// ----------------------------- external communication -------------------------

/* struct for the communication layer: contains the state to represent the 
rocket in 3d space */
typedef struct 
{
    /* velocities */
    ext_coord_t x_dot;
    ext_coord_t y_dot;
    ext_coord_t z_dot;
    /* Position */
    ext_coord_t x;
    ext_coord_t y;
    ext_coord_t z;

    /* angular velocities*/
    ext_coord_t roll_dot;
    ext_coord_t pitch_dot;
    ext_coord_t yaw_dot;
    /* angles */
    ext_coord_t roll;
    ext_coord_t pitch;
    ext_coord_t yaw;
} ext_fullState;

/* struct to represent the error With Respect To the setpoint*/
typedef struct
{
    /* Position */
    ext_coord_t xErr;
    ext_coord_t yErr;
    ext_coord_t zErr;
} ext_setpointError;

/* struct to package the system parameters*/
typedef struct
{
    ext_coord_t mass_Kg;
    ext_coord_t inertiaX_Kgm2;
    ext_coord_t inertiaY_Kgm2;
    ext_coord_t inertiaZ_Kgm2;
    ext_coord_t c;
    ext_coord_t cz;
} ext_rocketParams;

/* struct of the trajectory's parameters*/
typedef struct 
{
    ext_coord_t a0;
    ext_coord_t a1;
    ext_coord_t a2;
    ext_coord_t a3;
} ext_traj;

/* struct of the trajectory position point */
typedef struct 
{
    ext_coord_t x;
    ext_coord_t y;
    ext_coord_t z;
} ext_trajectoryPoint;

/* struct of the user input forces */
typedef struct 
{
    ext_coord_t fX;
    ext_coord_t fY;
    ext_coord_t fZ;
} ext_userForce;

/* struct of the actuator limits */
typedef struct 
{
    /* forces */
    //ext_coord_t fX_lim;
    //ext_coord_t fY_lim;
    ext_coord_t fZ_lim;
    /* torques */
    //ext_coord_t tX_lim;
    //ext_coord_t tY_lim;
    //ext_coord_t tZ_lim;
} ext_actuatorLimits;

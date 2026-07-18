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
#include "Point.hpp"
using namespace CDS;

Point::Point(const core_trajectoryPointParams_t params) : m_params(params)
{
    m_startTime = 0;
    m_endTime = m_params.time_s;
}

Point::~Point()
{

}

bool Point::GetReference(const core_coord_t&  time, Reference_t& ref) const
{
    // Reference is constant, TODO DP

    ref.pos[0] = m_params.finalPos[0]; 
    ref.pos[1] = m_params.finalPos[1];
    ref.pos[2] = m_params.finalPos[2];
    ref.yaw = m_params.finalYaw;
    
    ref.vel[0] = 0;
    ref.vel[1] = 0;
    ref.vel[2] = 0;
    ref.yawRate = 0;

    ref.acc[0] = 0;
    ref.acc[1] = 0;
    ref.acc[2] = 0;
    ref.yawAcc= 0;

    ref.jerk[0] =0;
    ref.jerk[1] =0;
    ref.jerk[2] =0;
    ref.yawJerk =0;

    ref.snap[0] =0;
    ref.snap[1] =0;
    ref.snap[2] =0;
    ref.yawSnap =0;

    return false;
}

bool Point::SetParameters(const std::map<std::string, core_coord_t>& params)
{
    return true; // TODO DP
    return false;
}

bool Point::GetParameters(std::map<std::string, core_coord_t>& params)
{
    return true; // TODO DP
    return false;
}

bool Point::SetParameter(const core_coord_t& p, size_t paramIdx)
{
    return true; // TODO DP
    return false;
}

bool Point::GetParameter(core_coord_t& p, size_t paramIdx)
{
    return true; // TODO DP
    
    return false;
}
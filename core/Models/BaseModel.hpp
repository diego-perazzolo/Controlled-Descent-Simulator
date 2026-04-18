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
#include "../core_defs.hpp"
#include "Trajectory.hpp"
#include <vector>
#include <functional>

namespace CDS
{

    // Physics model class
    class BaseModel
    {
        public:
        BaseModel();

        /* Virtual methods */
        
        virtual ~BaseModel();
        virtual bool SetModelParams(core_rocketParams_t& params) = 0;
        virtual bool SetTrajectory(Trajectory* pTrajectory) = 0;
        virtual bool PerformIntegration(core_stepParams_t& params) = 0;
        virtual bool GetState(core_state_t& state) = 0; 
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) = 0; 

        /* Private variables */

    };
}
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
// File        : BaseModel.hpp
// Description : Header file for BaseModel, implements a pure virtual class
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include "../core_defs.hpp"
#include "TrajectoryManager.hpp"
#include <vector>
#include <functional>
#include <any>

namespace CDS
{

    // Physics model class
    class BaseModel
    {
        public:
        BaseModel();

        /* Virtual methods */
        
        virtual ~BaseModel();
        virtual bool SetModelParams(const std::any& params) = 0;
        virtual bool SetTrajectoryManager(TrajectoryManager* pTrajectoryManager) = 0;
        virtual bool PerformIntegration(const core_stepParams_t& params) = 0;
        virtual bool GetState(core_state_t& state) = 0;
        virtual bool GetTrackingErrors(core_trackingErrors_t& tErrors) = 0;
        virtual bool GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds) = 0;

        // Tunable-parameter interface (interactive tuning; not on the tick
        // path), split by domain: model (structural knobs, e.g. the MPC horizon),
        // controller (cost weights / solver), observer (estimator covariances +
        // enable) and sensor (per-axis bias / noise / enable). Each
        // Get<Domain>Manifest writes a TSV listing of that domain's exposed
        // parameters (`id\tgroup\tlabel\tflags\tvalue\n` per line);
        // Set<Domain>Param sets one parameter by its manifest id. The defaults
        // are an empty manifest / no settable parameters -- a model exposes only
        // the domains it has. All return true on error.
        virtual bool GetModelManifest(char* buf, std::size_t n)      { return emptyManifest(buf, n); }
        virtual bool SetModelParam(int /*id*/, double /*value*/)      { return true; }
        virtual bool GetControllerManifest(char* buf, std::size_t n) { return emptyManifest(buf, n); }
        virtual bool SetControllerParam(int /*id*/, double /*value*/) { return true; }
        virtual bool GetObserverManifest(char* buf, std::size_t n)   { return emptyManifest(buf, n); }
        virtual bool SetObserverParam(int /*id*/, double /*value*/)   { return true; }
        virtual bool GetSensorManifest(char* buf, std::size_t n)     { return emptyManifest(buf, n); }
        virtual bool SetSensorParam(int /*id*/, double /*value*/)     { return true; }

        protected:
        // Write an empty (NUL-terminated) manifest into `buf`. Returns false (no
        // error): the shared default for a domain a model does not expose.
        static bool emptyManifest(char* buf, std::size_t n)
        {
            if (buf != nullptr && n > 0) buf[0] = '\0';
            return false;
        }
        /* Private variables */

    };
}
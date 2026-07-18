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
// File        : SystemManager.hpp
// Description : Owner of the simulated system (model, trajectory, parameters):
//               single lock boundary between the real-time tick thread and
//               the ext command layer
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <array>
#include <functional>
#include <memory>
#include <mutex>

#include "BaseModel.hpp"
#include "BasePlant.hpp"
#include "TrajectoryManager.hpp"

namespace CDS
{

    class SystemManager
    {
        public:

        typedef double sm_coord_t; // floating point type
        typedef std::unique_ptr<BaseModel> modelPtr_t;
        typedef std::unique_ptr<BasePlant> plantPtr_t;
        typedef std::unique_ptr<TrajectoryManager> trajectoryManagerPtr_t;
        typedef std::array<sm_coord_t, 3> userForces_t;
        typedef std::mutex mutex_t;
        typedef std::lock_guard<std::mutex> lockGuard_t;

        typedef struct
        {
            sm_coord_t timestep_seconds;
        } systemManagerParams_t;

        SystemManager(void);
        ~SystemManager();

        /* Install a new model (ownership transferred), system must be stopped.
           Returns true on error */
        bool InitModel(modelPtr_t&& pModel);

        /* Create a fresh empty trajectory, system must be stopped.
           Returns true on error */
        bool InitTrajectory(void);

        /* Attach a plant (ownership transferred), system must be stopped.
           The plant must arrive already configured (SetPlantParams done by
           the app). Attach connects the link (telemetry may flow while the
           system is stopped); Run()/Stop() then start/stop the mission on
           it. A plant whose link fails to connect is not attached.
           Returns true on error */
        bool AttachPlant(plantPtr_t&& pPlant);

        /* Stop the mission, disconnect the link and destroy the attached
           plant, system must be stopped. Returns true on error */
        bool DetachPlant(void);

        /* Run a function on the model under the system lock. The reference is
           valid only for the duration of the call — do not store it.
           Returns true on error */
        bool ExecuteOnModel(const std::function<bool(BaseModel&)>& modelFcn);

        /* Run a function on the plant under the system lock (this also makes
           the snapshot path a legal mailbox reader: every reader is
           serialized by the same mutex as the tick). The reference is valid
           only for the duration of the call. Returns true on error */
        bool ExecuteOnPlant(const std::function<bool(BasePlant&)>& plantFcn);

        /* Run a read-only function on the trajectory manager under the system
           lock (no side effect on the model): the const reference makes any
           mutation a compile error — use MutateTrajectoryManager for those.
           The reference is valid only for the duration of the call.
           Returns true on error */
        bool ExecuteOnTrajectoryManager(const std::function<bool(const TrajectoryManager&)>& tmFcn);

        /* Run a mutating function on the trajectory manager under the system
           lock, then re-attach the trajectory to the model (which re-anchors
           its state on the trajectory start). Returns true on error */
        bool MutateTrajectoryManager(const std::function<bool(TrajectoryManager&)>& tmFcn);

        /* Set user perturbation forces used by the next ticks. Returns true on error */
        bool SetUserForces(const userForces_t& uF);

        bool Run(void);
        bool ExecuteTick(sm_coord_t timestep_seconds);
        bool Stop(void);

        bool SetParameters(const systemManagerParams_t& params);
        bool GetParameters(systemManagerParams_t& params);
        inline bool IsRunning(void) { /* do not guard with mutex */ return m_isRunning; };
        inline bool IsModelOk(void) { /* do not guard with mutex */ return m_pModel != nullptr; };
        inline bool IsPlantOk(void) { /* do not guard with mutex */ return m_pPlant != nullptr; };

        private:

        /* Attach the current trajectory to the model, if both exist and the
           trajectory has at least one item. Caller must hold m_mutex */
        bool _attachTrajectoryToModel(void);

        modelPtr_t m_pModel;
        plantPtr_t m_pPlant;
        trajectoryManagerPtr_t m_pTrajectoryManager;

        systemManagerParams_t m_params;
        mutex_t m_mutex;
        bool m_isRunning;
        userForces_t m_userForces;
    };

} // namespace CDS

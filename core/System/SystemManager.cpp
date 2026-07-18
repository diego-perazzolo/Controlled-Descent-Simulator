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
// File        : SystemManager.cpp
// Description : Owner of the simulated system (model, trajectory, parameters):
//               single lock boundary between the real-time tick thread and
//               the ext command layer
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#include "SystemManager.hpp"
#include <cstdio>

//#define CDS_TRACE

#ifdef CDS_TRACE
#define TRACE(fmt, ...)                      \
    std::printf("[trace] %s:%d | " fmt "\n", \
                __FILE__, __LINE__ __VA_OPT__(, ) __VA_ARGS__)
#else
#define TRACE(fmt, ...) ((void)0)
#endif

/* guard clauses: early-return with error (true) when the system is not in the
   required state. Caller must hold m_mutex */
#define RETURN_ERR_IF_RUNNING          \
    do                                 \
    {                                  \
        if (IsRunning()) return true;  \
    } while (0)
#define RETURN_ERR_IF_STOPPED          \
    do                                 \
    {                                  \
        if (!IsRunning()) return true; \
    } while (0)
#define RETURN_ERR_IF_NO_MODEL         \
    do                                 \
    {                                  \
        if (!IsModelOk()) return true; \
    } while (0)

namespace CDS
{

SystemManager::SystemManager(void)
{
    TRACE("Created");
}

SystemManager::~SystemManager(void)
{
    /* no lock here: destruction requires that no other thread is using this
       object anymore — taking m_mutex (or calling Stop(), which takes it)
       would deadlock */
    m_isRunning = false;

    TRACE("Destroyed");
}

bool SystemManager::InitModel(modelPtr_t &&pModel)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;

    m_pModel = std::move(pModel);

    TRACE("OK");

    return false;
}

bool SystemManager::InitTrajectory(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;
    RETURN_ERR_IF_NO_MODEL;

    m_pTrajectoryManager = std::make_unique<TrajectoryManager>();

    TRACE("OK");

    return false;
}

bool SystemManager::ExecuteOnModel(const std::function<bool(BaseModel &)> &modelFcn)
{
    if (!modelFcn)
    {
        // Invalid function, error
        return true;
    }

    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    bool ret = modelFcn(*m_pModel);

    if (!ret)
    {
        TRACE("OK");
    }

    return ret;
}

bool SystemManager::ExecuteOnTrajectoryManager(const std::function<bool(const TrajectoryManager &)> &tmFcn)
{
    if (!tmFcn)
    {
        // Invalid function, error
        return true;
    }

    lockGuard_t lock(m_mutex);

    if (!m_pTrajectoryManager)
    {
        // No trajectory initialized, error
        return true;
    }

    bool ret = tmFcn(*m_pTrajectoryManager);

    if (!ret)
    {
        TRACE("OK");
    }

    return ret;
}

bool SystemManager::MutateTrajectoryManager(const std::function<bool(TrajectoryManager &)> &tmFcn)
{
    if (!tmFcn)
    {
        // Invalid function, error
        return true;
    }

    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    if (!m_pTrajectoryManager)
    {
        // No trajectory initialized, error
        return true;
    }

    bool ret = tmFcn(*m_pTrajectoryManager);

    if (ret)
    {
        // Mutation failed, error
        return true;
    }

    ret = _attachTrajectoryToModel();

    if (!ret)
    {
        TRACE("OK");
    }

    return ret;
}

bool SystemManager::SetUserForces(const userForces_t &uF)
{
    lockGuard_t lock(m_mutex);

    m_userForces = uF;
    return false;
}

bool SystemManager::Run(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    m_isRunning = true;

    TRACE("OK");

    return false;
}

bool SystemManager::ExecuteTick(sm_coord_t timestep_seconds)
{
    lockGuard_t lock(m_mutex);
    bool ret = false;

    RETURN_ERR_IF_NO_MODEL;
    RETURN_ERR_IF_STOPPED;

    // Compute safety considerations

    // Drive the plant, using past control data

    // Collect data from the plant

    // Perform filtering

    // Compute control

    // Perform model integration
    ret |= m_pModel->PerformIntegration({.timestep = timestep_seconds,
                                         .user_fX = m_userForces[0],
                                         .user_fY = m_userForces[1],
                                         .user_fZ = m_userForces[2]});

    if (!ret)
    {
        TRACE("OK: %f", timestep_seconds);
    }

    return ret;
}

bool SystemManager::Stop(void)
{
    lockGuard_t lock(m_mutex);

    m_isRunning = false;

    TRACE("OK");

    return false;
}

bool SystemManager::SetParameters(const systemManagerParams_t &params)
{
    lockGuard_t lock(m_mutex);

    m_params = params;

    TRACE("OK");
    return false;
}

bool SystemManager::GetParameters(systemManagerParams_t &params)
{
    lockGuard_t lock(m_mutex);

    params = m_params;

    return false;
}

bool SystemManager::_attachTrajectoryToModel(void)
{
    /* caller holds m_mutex; model and trajectory existence already checked */

    Reference_t ref;
    if (m_pTrajectoryManager->GetReference(0, ref))
    {
        // Empty trajectory: nothing to attach, not an error
        return false;
    }

    return m_pModel->SetTrajectoryManager(m_pTrajectoryManager.get());
}

} // namespace CDS

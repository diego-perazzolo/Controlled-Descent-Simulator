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
#include "log.hpp"
#include "profile.hpp"

static const auto logger = cds_log::registry().module("SystemManager");
static const auto logger_tick = cds_log::registry().module("SystemManagerTick");
static const auto profile_tick = cds_profile::registry().module("SystemManagerTick");


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
#define RETURN_ERR_IF_NO_PLANT         \
    do                                 \
    {                                  \
        if (!IsPlantOk()) return true; \
    } while (0)

namespace CDS
{

SystemManager::SystemManager(void) : m_pModel(nullptr),
                                     m_pPlant(nullptr),
                                     m_pTrajectoryManager(nullptr),
                                     m_params({0}),
                                     m_isRunning(false),
                                     m_userForces({0})
{
    CDS_LOG_INFO(logger, "Created");
}

SystemManager::~SystemManager(void)
{
    /* no lock here: destruction requires that no other thread is using this
       object anymore — taking m_mutex (or calling Stop(), which takes it)
       would deadlock */
    m_isRunning = false;

    CDS_LOG_INFO(logger, "Destroyed");
}

bool SystemManager::InitModel(modelPtr_t &&pModel)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;

    m_pModel = std::move(pModel);

    CDS_LOG_INFO(logger, "Model initialized");

    return false;
}

bool SystemManager::InitTrajectory(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;
    RETURN_ERR_IF_NO_MODEL;

    m_pTrajectoryManager = std::make_unique<TrajectoryManager>();

    CDS_LOG_INFO(logger, "Trajectory initialized");

    return false;
}

bool SystemManager::AttachPlant(plantPtr_t &&pPlant)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;

    if (!pPlant)
    {
        CDS_LOG_ERROR(logger, "Invalid plant");
        return true;
    }

    /* the link lives from attach to detach: bring it up now, so telemetry
       flows and the vehicle can be staged before any mission starts */
    if (pPlant->Connect())
    {
        CDS_LOG_ERROR(logger, "Could not connect to plant");
        return true;
    }

    m_pPlant = std::move(pPlant);

    CDS_LOG_INFO(logger, "Plant attached");

    return false;
}

bool SystemManager::DetachPlant(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;
    RETURN_ERR_IF_NO_PLANT;

    m_pPlant->Stop();
    m_pPlant->Disconnect();
    m_pPlant.reset();

    CDS_LOG_INFO(logger, "Plant detached");

    return false;
}

bool SystemManager::ExecuteOnModel(const std::function<bool(BaseModel &)> &modelFcn)
{
    if (!modelFcn)
    {
        CDS_LOG_ERROR(logger, "Cannot execute on model");
        return true;
    }

    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    bool ret = modelFcn(*m_pModel);

    if (ret)
    {
        CDS_LOG_ERROR(logger, "Execute on model");
    }

    return ret;
}

bool SystemManager::ExecuteOnPlant(const std::function<bool(BasePlant &)> &plantFcn)
{
    if (!plantFcn)
    {
        CDS_LOG_ERROR(logger, "Cannot execute on plant");
        return true;
    }

    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_PLANT;

    bool ret = plantFcn(*m_pPlant);

    if (ret)
    {
        CDS_LOG_ERROR(logger, "Execute on plant");
    }

    return ret;
}

bool SystemManager::ExecuteOnTrajectoryManager(const std::function<bool(const TrajectoryManager &)> &tmFcn)
{
    if (!tmFcn)
    {
        CDS_LOG_ERROR(logger, "Cannot execute on trajectory");
        return true;
    }

    lockGuard_t lock(m_mutex);

    if (!m_pTrajectoryManager)
    {
        CDS_LOG_ERROR(logger, "Cannot execute on trajectory");
        return true;
    }

    bool ret = tmFcn(*m_pTrajectoryManager);

    if (ret)
    {
        CDS_LOG_ERROR(logger, "Execute on trajectory");
    }

    return ret;
}

bool SystemManager::MutateTrajectoryManager(const std::function<bool(TrajectoryManager &)> &tmFcn)
{
    if (!tmFcn)
    {
        CDS_LOG_ERROR(logger, "Cannot mutate trajectory");
        return true;
    }

    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    if (!m_pTrajectoryManager)
    {
        CDS_LOG_ERROR(logger, "Cannot mutate trajectory");
        return true;
    }

    bool ret = tmFcn(*m_pTrajectoryManager);

    if (ret)
    {
        CDS_LOG_ERROR(logger, "Mutating trajectory");
        return true;
    }

    ret = _attachTrajectoryToModel();

    if (ret)
    {
        CDS_LOG_ERROR(logger, "Attaching trajectory");
    }

    return ret;
}

bool SystemManager::SetUserForces(const userForces_t &uF)
{
    lockGuard_t lock(m_mutex);

    m_userForces = uF;
    return false;
}

bool SystemManager::BeginStaging(sm_coord_t safetyAltitude)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_RUNNING;
    RETURN_ERR_IF_NO_PLANT;

    if (!m_pTrajectoryManager || safetyAltitude < 0)
    {
        // No trajectory to size the altitude, or invalid margin, error
        CDS_LOG_ERROR(logger, "Trajectory not initialized or wrong safe altitude");
        return true;
    }

    core_coord_t altitudeRange = 0;
    if (m_pTrajectoryManager->GetAltitudeRange(altitudeRange))
    {
        CDS_LOG_ERROR(logger, "Cannot get altitude range from trajectory");
        return true;
    }

    /* face the trajectory's initial heading when staged, so the mission
       starts already aligned (no yaw jump at Start) */
    Reference_t ref0;
    if (m_pTrajectoryManager->GetReference(0, ref0))
    {
        CDS_LOG_ERROR(logger, "Cannot get first trajectory sample");
        return true;
    }

    /* stage above the trajectory's vertical travel by the safety margin: an
       additive margin stays valid even for a flat trajectory (range 0) */
    return m_pPlant->BeginStaging(altitudeRange + safetyAltitude, ref0.yaw);
}

bool SystemManager::StopStaging(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_PLANT;

    return m_pPlant->StopStaging();
}

bool SystemManager::Run(void)
{
    lockGuard_t lock(m_mutex);

    RETURN_ERR_IF_NO_MODEL;

    /* The plant is OPTIONAL: the system runs with the model alone. When a
       plant is attached its link is already up (since attach): Start only
       begins the mission on the warm link */
    if (IsPlantOk())
    {
        /* Seed a fresh command at the trajectory start BEFORE starting the
           mission. The plant captures its frame offset from the first command
           it finds in the mailbox once the mission begins; without this seed
           it would read the stale command left there by the previous run (the
           held final reference), misaligning the ghost at t=0. This value
           equals what the first tick will push (model time has not advanced),
           so it introduces no inconsistency. */
        if (m_pTrajectoryManager)
        {
            BasePlant::plantCommands_t commands = {};
            if (!m_pModel->GetCurrentTimeSeconds(commands.time_seconds) &&
                !m_pTrajectoryManager->GetReference(commands.time_seconds,
                                                    commands.reference))
            {
                m_pPlant->PushCommands(commands);
            }
        }

        if (m_pPlant->Start())
        {
            CDS_LOG_ERROR(logger, "Cannot start plant");
            return true;
        }
    }

    m_isRunning = true;

    return false;
}

bool SystemManager::ExecuteTick(sm_coord_t timestep_seconds)
{
    CDS_PROFILE(profile_tick, "ExecuteTick");
    lockGuard_t lock(m_mutex);
    bool ret = false;

    
    RETURN_ERR_IF_NO_MODEL;
    RETURN_ERR_IF_STOPPED;

    // Compute safety considerations

    /* NOTE: a lagging or erroring plant must not kill the integration — the
       exchange below is best-effort, its failures are not propagated */

    // Drive the plant, using past control data
    if (m_pPlant && m_pTrajectoryManager)
    {
        BasePlant::plantCommands_t commands = {};
        if (!m_pModel->GetCurrentTimeSeconds(commands.time_seconds) &&
            !m_pTrajectoryManager->GetReference(commands.time_seconds, commands.reference))
        {
            ret |= m_pPlant->PushCommands(commands);
        }
    }

    // Collect data from the plant
    if (m_pPlant)
    {
        BasePlant::plantMeasurements_t measurements = {};
        /* failure tolerated before the first published sample; measurements
           will feed filtering and control (future) */
        ret |= m_pPlant->PullMeasurements(measurements);
    }

    // Perform filtering

    // Compute control

    // Perform model integration
    ret |= m_pModel->PerformIntegration({.timestep = timestep_seconds,
                                         .user_fX = m_userForces[0],
                                         .user_fY = m_userForces[1],
                                         .user_fZ = m_userForces[2]});

    if (ret)
    {
       CDS_LOG_ERROR(logger_tick, "Error during real-time integration");
    }

    return ret;
}

bool SystemManager::Stop(void)
{
    lockGuard_t lock(m_mutex);
    bool ret = false;

    m_isRunning = false;

    if (m_pPlant)
    {
        ret |= m_pPlant->Stop();
    }
    
    if (ret)
    {
        CDS_LOG_ERROR(logger, "Cannot stop plant");
    }
    else
    {
        CDS_LOG_INFO(logger, "Plant stopped");
    }

    return ret;
}

bool SystemManager::SetParameters(const systemManagerParams_t &params)
{
    lockGuard_t lock(m_mutex);

    m_params = params;

    CDS_LOG_INFO(logger, "Parameters set");
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
        CDS_LOG_ERROR(logger, "Cannot attach an empty trajectory");
        return false;
    }

    return m_pModel->SetTrajectoryManager(m_pTrajectoryManager.get());
}

} // namespace CDS

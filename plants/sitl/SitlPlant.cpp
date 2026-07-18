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
// File        : SitlPlant.cpp
// Description : ArduPilot SITL plant: MAVLink 2 over UDP, Guided-mode
//               setpoints out, local-frame telemetry back as measurements
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "SitlPlant.hpp"

/* sole gateway to the vendored MAVLink headers: compiling it here keeps the
   protocol pins guarding every build of this library */
#include "mavlink_pin.hpp"

#include <chrono>

using namespace plants;

using FpSeconds = std::chrono::duration<double>;

SitlPlant::SitlPlant() : m_params({.host = "127.0.0.1",
                                   .port = 14550,
                                   .setpointPeriod_seconds = 0.05,
                                   .telemetryPeriod_seconds = 0.02,
                                   .linkTimeout_seconds = 2.0}),
                         m_threadRun(false),
                         m_missionRun(false)
{

}

SitlPlant::~SitlPlant()
{
    Stop();
    Disconnect();
}

bool SitlPlant::SetPlantParams(const std::any& params)
{
    if (m_thread.joinable())
    {
        // Cannot reconfigure while connected, error
        return true;
    }

    if (params.type() != typeid(sitlParams_t&))
    {
        // Err
        return true;
    }

    const auto& p = std::any_cast<const sitlParams_t&>(params);

    if (p.host.empty() || p.port == 0 ||
        p.setpointPeriod_seconds <= 0 || p.telemetryPeriod_seconds <= 0 ||
        p.linkTimeout_seconds <= 0)
    {
        // Invalid parameters, error
        return true;
    }

    m_params = p;
    return false;
}

bool SitlPlant::Connect(void)
{
    if (m_thread.joinable())
    {
        // Already connected, error
        return true;
    }

    m_threadRun = true;
    m_thread = std::thread(&SitlPlant::_commLoop, this);

    return false;
}

bool SitlPlant::Disconnect(void)
{
    /* idempotent: disconnecting a disconnected plant is not an error */
    if (!m_thread.joinable())
    {
        return false;
    }

    m_missionRun = false;
    m_threadRun = false;
    m_thread.join();

    return false;
}

bool SitlPlant::Start(void)
{
    if (!m_thread.joinable())
    {
        // Mission on a disconnected link, error
        return true;
    }

    if (m_missionRun)
    {
        // Already started, error
        return true;
    }

    m_missionRun = true;
    return false;
}

bool SitlPlant::Stop(void)
{
    /* idempotent: stopping a stopped mission is not an error */
    m_missionRun = false;
    return false;
}

void SitlPlant::_commLoop(void)
{
    /* skeleton: the loop is already paced by the setpoint period (the
       outbound clock of the real link); socket, heartbeat exchange and the
       link state machine land with the next phase — m_missionRun will gate
       the setpoint streaming there */
    while (m_threadRun)
    {
        std::this_thread::sleep_for(FpSeconds(m_params.setpointPeriod_seconds));
    }
}

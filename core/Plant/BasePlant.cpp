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
// File        : BasePlant.cpp
// Description : Base plant class: mailbox exchange (triple buffers, sequence
//               numbering) shared by every plant implementation
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "BasePlant.hpp"

using namespace CDS;

BasePlant::BasePlant() : m_publishSequence(0)
{

}

BasePlant::~BasePlant()
{

}

bool BasePlant::PushCommands(const plantCommands_t& commands)
{
    m_commandsBuffer.Write(commands);
    return false;
}

bool BasePlant::PullMeasurements(plantMeasurements_t& measurements)
{
    return m_measurementsBuffer.Read(measurements);
}

bool BasePlant::GetState(core_state_t& state)
{
    plantMeasurements_t measurements;
    if (m_measurementsBuffer.Read(measurements))
    {
        // No sample published yet, error
        return true;
    }

    state = measurements.state;
    return false;
}

bool BasePlant::GetCurrentTimeSeconds(core_coord_t& currentTimeSeconds)
{
    plantMeasurements_t measurements;
    if (m_measurementsBuffer.Read(measurements))
    {
        // No sample published yet, error
        return true;
    }

    currentTimeSeconds = measurements.plantTime_seconds;
    return false;
}

bool BasePlant::FetchCommands(plantCommands_t& commands)
{
    return m_commandsBuffer.Read(commands);
}

bool BasePlant::PublishMeasurements(const core_state_t& state, core_coord_t plantTime_seconds)
{
    plantMeasurements_t measurements = {};
    measurements.sequence = ++m_publishSequence;
    measurements.plantTime_seconds = plantTime_seconds;
    measurements.state = state;

    m_measurementsBuffer.Write(measurements);
    return false;
}

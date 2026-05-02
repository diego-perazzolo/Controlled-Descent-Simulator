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

#include "TrajectoryManager.hpp"
#include "Poly4.hpp"
#include "Point.hpp"

using namespace CDS;

TrajectoryManager::TrajectoryManager()
{
    m_absoluteStartTime = 0;
    m_currentItemIndex = 0;
}

bool TrajectoryManager::AppendPoly4(const core_trajectoryPoly4Params_t& params)
{
    return AppendItem(std::make_unique<Poly4>(params));
}

bool TrajectoryManager::AppendPoint(const core_trajectoryPointParams_t& params)
{
    return AppendItem(std::make_unique<Point>(params));
}

bool TrajectoryManager::GoNextItem(void)
{
    core_coord_t finalTime = 0;
    m_trajectoryItems[m_currentItemIndex++]->GetEndTime(finalTime);

    if(m_currentItemIndex >= m_trajectoryItems.size())
    {
        m_currentItemIndex = m_trajectoryItems.size() - 1;
        return true;
    }

    m_absoluteStartTime += finalTime;

    return false;
}

bool TrajectoryManager::GoPrevItem(void)
{
    core_coord_t finalTime = 0;
    m_currentItemIndex--;

    if(m_currentItemIndex < 0)
    {
        m_currentItemIndex = 0;
        return true;
    }

    m_trajectoryItems[m_currentItemIndex]->GetEndTime(finalTime);
    m_absoluteStartTime -= finalTime;

    return false;
}

bool TrajectoryManager::AppendItem(std::unique_ptr<Trajectory> itemPtr)
{
    if(itemPtr == nullptr)
    {
        // Err
        return true;
    }

    m_trajectoryItems.push_back(std::move(itemPtr));

    return false;
};

bool TrajectoryManager::RemoveLastItem(void) 
{
    if(m_trajectoryItems.empty())
    {
        // Err
        return true;
    }

    m_trajectoryItems.pop_back();

    return false;
};

bool TrajectoryManager::GetReference(const core_coord_t& time, Reference_t& ref)
{
    if(m_trajectoryItems.empty())
    {
        // Err
        return true; 
    }

    core_coord_t endTime;
    m_trajectoryItems[m_currentItemIndex]->GetEndTime(endTime);

    while(time > m_absoluteStartTime + endTime)
    {
        // Try to switch to trajectory in the proper time range
        if(GoNextItem())
        {
            // there is no next item, returning final reference of current trajectory item
            return m_trajectoryItems[m_currentItemIndex]->GetReference(endTime, ref);
        }

        m_trajectoryItems[m_currentItemIndex]->GetEndTime(endTime);
    }
    
    while(time - m_absoluteStartTime < 0)
    {
        // Try to switch to trajectory in the proper time range
        if(GoPrevItem())
        {
            // there is no prev item, returning inital reference of current trajectory item
            return m_trajectoryItems[m_currentItemIndex]->GetReference(0, ref);
        }   
    }

    // Get reference from current trajectory item
    return  m_trajectoryItems[m_currentItemIndex]->GetReference(time - m_absoluteStartTime, ref);
}

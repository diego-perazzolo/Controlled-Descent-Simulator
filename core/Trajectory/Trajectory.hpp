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
// File        : Trajectory.hpp
// Description : Abstract base class for a single trajectory item (a
//               sub-trajectory): the reference-sampling interface
//               (GetReference at a time instant) and the parametric
//               getters/setters, plus the cached start/end times shared by
//               every item (Poly4, Point, ...).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>
#include "core_defs.hpp"

namespace CDS
{
    // Trajectory class
    class Trajectory
    {

        public:

        Trajectory() = default;

        /* Virtual methods */

        virtual ~Trajectory() = default;

        /* Gets reference trajectory state at a time instant. Returns true on error */
        virtual bool GetReference(const core_coord_t&  time, Reference_t& ref) const = 0;

        /* Set dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool SetParameters(const std::map<std::string, core_coord_t>& params) = 0;

        /* Get dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool GetParameters(std::map<std::string, core_coord_t>& params) = 0;

        /* Set trajectory parameter. Returns true on error */
        virtual bool SetParameter(const core_coord_t& p, size_t paramIdx) = 0;

        /* Get trajectory parameter. Returns true on error */
        virtual bool GetParameter(core_coord_t& p, size_t paramIdx) = 0;


        /* Trajectory methods */

        /* Get trajectory start time */
        void GetStartTime(core_coord_t& s) const;

         /* Get trajectory end time */
        void GetEndTime(core_coord_t& e) const;

        protected:

        /* Trajectory item is well defined within a timespan [start; end] */
        core_coord_t m_startTime; // start time of the trajectory item
        core_coord_t m_endTime; // end time of the trajectory item


    };
}
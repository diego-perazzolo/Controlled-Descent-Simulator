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
// File        : TrajectoryManager.hpp
// Description : Ordered list of trajectory items (Poly4, Point, ...): appends
//               and removes items, samples the concatenated reference at a
//               time instant, and reports the overall vertical range (used to
//               size the plant staging altitude).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include <memory>
#include <vector>
#include "core_defs.hpp"
#include "Trajectory.hpp"

namespace CDS
{
    class TrajectoryManager
    {
        public:
        TrajectoryManager();
        ~TrajectoryManager() = default;
        
        /*Append to trajectory vector one 4th polybnomial trajectory, returns true on error */
        bool AppendPoly4(const core_trajectoryPoly4Params_t& params);

        /*Append to trajectory vector one constant Point trajectory, returns true on error */
        bool AppendPoint(const core_trajectoryPointParams_t& params);

        /* Remove last trajectory item */
        bool RemoveLastItem(void);

        /* Get the reference at a time instant. Logically a pure read: const,
           but it advances the internal seek cursor (mutable). Returns true on error */
        bool GetReference(const core_coord_t& time, Reference_t& ref) const;

        /* Vertical range (max − min altitude z) of the whole trajectory, found
           by sampling every item across its duration. Used to size the plant
           staging altitude (the vehicle must climb enough to cover the
           trajectory's vertical travel). Returns true on error (empty) */
        bool GetAltitudeRange(core_coord_t& range) const;

        private:

        /* Append a trajectory item to the stack */
        bool AppendItem(std::unique_ptr<Trajectory> itemPtr);

        /* Switch to the next trajectory item */
        bool GoNextItem(void) const;

        /*Switch to the previews trajectory item*/
        bool GoPrevItem(void) const;

        /* Reset the seek cursor to the start of the sequence. MUST be called
           after any mutation of m_trajectoryItems: the cursor caches an index
           into the item list, so a stale index survives a remove/append done
           without re-initializing the manager (e.g. loading a saved trajectory
           over an existing one) and reads out of bounds once the list shrinks. */
        void ResetSeekCursor(void);

        std::vector<std::unique_ptr<Trajectory>> m_trajectoryItems;

        /* Seek cursor: caching detail of GetReference, not observable state —
           hence mutable. NOTE: const does NOT mean thread-safe here; every
           access is serialized by the SystemManager lock */
        mutable int m_currentItemIndex;
        mutable core_coord_t m_absoluteStartTime;
        
    };
}
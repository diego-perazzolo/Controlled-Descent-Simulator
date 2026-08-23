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
// File        : observer_params.hpp
// Description : Observer-domain glue: exposes a disturbance observer's tuning
//               knobs (enable + process/measurement covariances) as tunable
//               parameters through a param::ParamTable. The covariances live in
//               the model that owns the observer (its tuning); this only binds
//               getters/setters to those references and re-synthesises the gain
//               on change. The parameter labels are the observer domain's own,
//               registered here next to the observer.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <functional>

#include "param_table.hpp"   // libs/param -- the tunable-parameter registry

namespace CDS { namespace estimate {

// Append the disturbance-observer knobs to `table`: enable (0|1), the
// measurement-noise covariance and the process-noise diagonal (position,
// velocity, disturbance). The refs bind to the owner's members (same lifetime);
// booleans are carried as 0|1 doubles (>= 0.5 is true). The covariances must be
// positive and a change re-synthesises the gain via `rebuild` (off the tick
// path): the measurement-noise covariance rPos MUST be matched to the sensor's
// actual noise or the filter is mistuned -- too small an rPos makes the observer
// chase the sensor noise (and its disturbance state inject spurious forces),
// which can be worse than no filtering at all. The large qDist lets the
// disturbance state integrate the residual.
template <std::size_t CAP>
inline void appendObserverParams(param::ParamTable<CAP>& table,
                                 bool& enabled,
                                 double& qPos, double& qVel, double& qDist, double& rPos,
                                 std::function<void()> rebuild)
{
    table.add("Observer", "enable (0|1)", true,
        [&enabled] { return enabled ? 1.0 : 0.0; },
        [&enabled](double v) { enabled = (v >= 0.5); return false; });
    table.add("Observer", "measurement noise", true,
        [&rPos] { return rPos; },
        [&rPos, rebuild](double v) { if (v <= 0.0) return true; rPos = v; rebuild(); return false; });
    table.add("Observer", "process noise: position", true,
        [&qPos] { return qPos; },
        [&qPos, rebuild](double v) { if (v <= 0.0) return true; qPos = v; rebuild(); return false; });
    table.add("Observer", "process noise: velocity", true,
        [&qVel] { return qVel; },
        [&qVel, rebuild](double v) { if (v <= 0.0) return true; qVel = v; rebuild(); return false; });
    table.add("Observer", "process noise: disturbance", true,
        [&qDist] { return qDist; },
        [&qDist, rebuild](double v) { if (v <= 0.0) return true; qDist = v; rebuild(); return false; });
}

}}  // namespace CDS::estimate

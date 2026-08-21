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
// File        : estimator_params.hpp
// Description : Shared helper that appends the disturbance-observer + position-
//               sensor knobs to a model's controller-parameter manifest, so they
//               ride the existing controller-manifest ext channel (no new wire
//               protocol). One place, used by every model that carries an
//               observer, to avoid copy-pasting the same rows four times. Exposed
//               rows: Observer/enable (0|1), and per axis Sensor x|y|z with
//               enable (0|1), bias and noise std. The frontend groups them into a
//               sensor panel by the manifest `group` column.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <cstddef>
#include <functional>

#include "controller_params.hpp"   // libs/control -- the manifest ParamTable
#include "sensor_model.hpp"        // libs/sensor   -- the per-axis sensor bank

namespace CDS
{
    // Corrupt a measurement triple through the sensor bank and return it. Disabled
    // axes pass the true value through unchanged (without an estimator there is
    // nothing to coast on, so a dropped axis simply injects nothing -- coasting is
    // the observer's value-add). Used by the no-observer path to feed the
    // controller the raw (unfiltered) measurement, so sensor noise/bias bite even
    // when the estimator is off.
    template <std::size_t NP>
    inline std::array<double, NP> measuredThrough(sensor::SensorModel<NP>& s,
                                                  const std::array<double, NP>& truth)
    {
        std::array<double, NP> y{};
        std::array<bool, NP>   valid{};
        s.Apply(truth, y, valid);
        return y;
    }

    // Append the observer knobs and per-axis sensor knobs to `params`. The
    // getters/setters bind to the model's own state by reference (same lifetime
    // as the model that owns them). Booleans are carried as 0|1 doubles (>= 0.5
    // is true) so they fit the scalar manifest. Noise std rejects negatives; bias
    // is unrestricted; the covariances must be positive.
    //
    // The observer covariances are exposed and re-synthesise the gain on change
    // (via `rebuild`, off the tick path): the measurement-noise covariance rPos
    // MUST be matched to the sensor's actual noise or the filter is mistuned --
    // too small an rPos makes the observer chase the sensor noise (and its
    // disturbance state inject spurious forces), which can be worse than no
    // filtering at all. Qpos/Qvel/Qdist are the process-noise diagonal (the large
    // Qdist lets the disturbance state integrate the residual).
    template <std::size_t NP>
    inline void appendEstimatorParams(control::ParamTable<>& params,
                                      bool& obsEnabled,
                                      sensor::SensorModel<NP>& sensor,
                                      double& qPos, double& qVel, double& qDist, double& rPos,
                                      std::function<void()> rebuild)
    {
        params.add("Observer", "enable (0|1)", true,
            [&obsEnabled] { return obsEnabled ? 1.0 : 0.0; },
            [&obsEnabled](double v) { obsEnabled = (v >= 0.5); return false; });
        params.add("Observer", "measurement noise", true,
            [&rPos] { return rPos; },
            [&rPos, rebuild](double v) { if (v <= 0.0) return true; rPos = v; rebuild(); return false; });
        params.add("Observer", "process noise: position", true,
            [&qPos] { return qPos; },
            [&qPos, rebuild](double v) { if (v <= 0.0) return true; qPos = v; rebuild(); return false; });
        params.add("Observer", "process noise: velocity", true,
            [&qVel] { return qVel; },
            [&qVel, rebuild](double v) { if (v <= 0.0) return true; qVel = v; rebuild(); return false; });
        params.add("Observer", "process noise: disturbance", true,
            [&qDist] { return qDist; },
            [&qDist, rebuild](double v) { if (v <= 0.0) return true; qDist = v; rebuild(); return false; });

        static const char* const group[3] = {"Sensor x", "Sensor y", "Sensor z"};
        const std::size_t n = (NP < 3) ? NP : 3;   // named axes cover up to x/y/z
        for (std::size_t a = 0; a < n; ++a)
        {
            params.add(group[a], "enable (0|1)", true,
                [&sensor, a] { return sensor.GetChannel(a).enabled ? 1.0 : 0.0; },
                [&sensor, a](double v) { sensor.SetEnabled(a, v >= 0.5); return false; });
            params.add(group[a], "bias", true,
                [&sensor, a] { return sensor.GetChannel(a).bias; },
                [&sensor, a](double v) { sensor.SetBias(a, v); return false; });
            params.add(group[a], "noise std", true,
                [&sensor, a] { return sensor.GetChannel(a).noiseStd; },
                [&sensor, a](double v) { if (v < 0.0) return true; sensor.SetNoiseStd(a, v); return false; });
        }
    }
}

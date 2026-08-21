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

#include "controller_params.hpp"   // libs/control -- the manifest ParamTable
#include "sensor_model.hpp"        // libs/sensor   -- the per-axis sensor bank

namespace CDS
{
    // Append the observer toggle and per-axis sensor knobs to `params`. The
    // getters/setters bind to the model's own `obsEnabled` flag and `sensor`
    // bank by reference (same lifetime as the model that owns them). Booleans
    // are carried as 0|1 doubles (>= 0.5 is true) so they fit the scalar
    // manifest. Noise std rejects negatives; bias is unrestricted.
    template <std::size_t NP>
    inline void appendEstimatorParams(control::ParamTable<>& params,
                                      bool& obsEnabled,
                                      sensor::SensorModel<NP>& sensor)
    {
        params.add("Observer", "enable (0|1)", true,
            [&obsEnabled] { return obsEnabled ? 1.0 : 0.0; },
            [&obsEnabled](double v) { obsEnabled = (v >= 0.5); return false; });

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

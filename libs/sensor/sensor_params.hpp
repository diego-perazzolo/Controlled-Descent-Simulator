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
// File        : sensor_params.hpp
// Description : Sensor-domain glue: exposes a SensorModel's per-channel knobs
//               (enable / bias / noise std) as tunable parameters through a
//               param::ParamTable, and a small helper to run a measurement
//               triple through the bank. The parameter labels are the sensor
//               domain's own (registered here, next to the bank); only the
//               per-channel group names come from the caller.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cstddef>

#include "param_table.hpp"    // libs/param -- the tunable-parameter registry
#include "sensor_model.hpp"   // the per-channel measurement corruptor

namespace CDS { namespace sensor {

// Corrupt a measurement triple through the sensor bank and return it. Disabled
// channels pass the true value through unchanged (without an estimator there is
// nothing to coast on, so a dropped channel simply injects nothing). Used by the
// no-observer path to feed the controller the raw (unfiltered) measurement, so
// sensor noise/bias bite even when the estimator is off.
template <std::size_t NP>
inline std::array<double, NP> measuredThrough(SensorModel<NP>& s,
                                              const std::array<double, NP>& truth)
{
    std::array<double, NP> y{};
    std::array<bool, NP>   valid{};
    s.Apply(truth, y, valid);
    return y;
}

// Append the per-channel sensor knobs to `table`: for each channel, enable
// (0|1), bias and noise std, under the group name `groups[i]` (the caller owns
// those NCH strings; noise std rejects negatives, bias is unrestricted). The
// getters/setters bind to `sensor` by reference (same lifetime).
template <std::size_t NCH, std::size_t CAP>
inline void appendSensorParams(param::ParamTable<CAP>& table,
                               SensorModel<NCH>& sensor,
                               const char* const* groups)
{
    for (std::size_t a = 0; a < NCH; ++a)
    {
        table.add(groups[a], "enable (0|1)", true,
            [&sensor, a] { return sensor.GetChannel(a).enabled ? 1.0 : 0.0; },
            [&sensor, a](double v) { sensor.SetEnabled(a, v >= 0.5); return false; });
        table.add(groups[a], "bias", true,
            [&sensor, a] { return sensor.GetChannel(a).bias; },
            [&sensor, a](double v) { sensor.SetBias(a, v); return false; });
        table.add(groups[a], "noise std", true,
            [&sensor, a] { return sensor.GetChannel(a).noiseStd; },
            [&sensor, a](double v) { if (v < 0.0) return true; sensor.SetNoiseStd(a, v); return false; });
    }
}

}}  // namespace CDS::sensor

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
// File        : sensor_model.hpp
// Description : Generic, heap-free per-channel measurement corruptor emulating a
//               sensor suite. It is the dual companion of the observer
//               (libs/estimate/observer.hpp): the observer removes disturbance,
//               this ADDS it. Given a truth vector it produces a measurement in
//               which each channel carries a constant bias plus zero-mean
//               Gaussian noise, and can be enabled / disabled at run time. A
//               disabled channel reports its measurement as invalid so a
//               downstream observer can skip that correction (predict-only) --
//               the sensor drop-out path. Domain-agnostic: it knows nothing about
//               vehicles or which physical quantity a channel is; the caller maps
//               state components to channels. Fully deterministic (a seeded
//               splitmix64 PRNG with Box-Muller), so tests get reproducible
//               streams across platforms. Fixed compile-time channel count NCH;
//               no heap; header-only, depends only on <array>, <cstddef>,
//               <cstdint>, <cmath>, so it may live under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cmath>

namespace CDS { namespace sensor {

// -----------------------------------------------------------------------------
// Example (standalone) -- a 3-channel position sensor, second channel biased and
// noisy, third channel switched off (a dropped sensor):
//
//   CDS::sensor::SensorModel<3> s;                 // identity by default
//   s.SetChannel(1, {.enabled = true,  .bias = 0.5, .noiseStd = 0.1});
//   s.SetEnabled(2, false);                        // drop the third sensor
//   std::array<double,3> truth{{1.0, 2.0, 3.0}}, meas;
//   std::array<bool,3>   valid;
//   s.Apply(truth, meas, valid);
//   // meas[0] == 1.0, valid[0] == true                (untouched, passthrough)
//   // meas[1] ~  2.5 +/- 0.1, valid[1] == true         (biased + noisy)
//   // meas[2] == 3.0, valid[2] == false                (withheld: predict-only)
// -----------------------------------------------------------------------------

template <std::size_t NCH>
class SensorModel
{
    public:

    // Per-channel corruption knobs. A default-constructed channel is the
    // identity (enabled, no bias, no noise) so a fresh SensorModel passes the
    // truth through untouched -- the feature is opt-in.
    struct Channel
    {
        bool   enabled;   // false -> measurement withheld, marked invalid
        double bias;      // additive constant offset
        double noiseStd;  // std-dev of the additive zero-mean Gaussian (>= 0)
    };

    // Default seed: the splitmix64 golden-ratio increment. A fixed default keeps
    // runs reproducible unless the caller reseeds.
    explicit SensorModel(std::uint64_t seed = 0x9E3779B97F4A7C15ull);

    // Runtime configuration (interactive; not a hot-path concern). Out-of-range
    // indices are ignored so a caller cannot corrupt memory from the frontend.
    void SetChannel(std::size_t i, const Channel& c);
    void SetEnabled(std::size_t i, bool on);
    void SetBias(std::size_t i, double bias);
    void SetNoiseStd(std::size_t i, double noiseStd);
    Channel GetChannel(std::size_t i) const;

    // Restart the noise stream (leaves the per-channel knobs untouched).
    void Reseed(std::uint64_t seed);

    // Corrupt a truth vector into a measurement. For each channel i:
    //   enabled  -> meas[i] = truth[i] + bias + N(0, noiseStd^2), valid[i]=true
    //   disabled -> meas[i] = truth[i] (passthrough), valid[i]=false
    // The passthrough value on a disabled channel is a convenience only; a
    // downstream observer must gate on valid[i], not read meas[i].
    void Apply(const std::array<double, NCH>& truth,
               std::array<double, NCH>&       meas,
               std::array<bool,   NCH>&       valid);

    private:

    // splitmix64: a single 64-bit state, good statistical quality, trivially
    // portable and heap-free -- the whole point of hand-rolling instead of
    // pulling in <random>'s engines whose streams vary across implementations.
    std::uint64_t NextU64(void);
    double        NextUniform(void);   // uniform in [0, 1)
    double        NextGaussian(void);  // zero-mean, unit-variance (Box-Muller)

    std::array<Channel, NCH> m_ch;
    std::uint64_t            m_rng;      // splitmix64 state
    double                   m_spare;    // cached second Box-Muller deviate
    bool                     m_hasSpare; // whether m_spare is live
};

// ---- construction -----------------------------------------------------------

template <std::size_t NCH>
SensorModel<NCH>::SensorModel(std::uint64_t seed)
    : m_ch{}, m_rng(seed), m_spare(0.0), m_hasSpare(false)
{
    for (std::size_t i = 0; i < NCH; ++i)
        m_ch[i] = Channel{true, 0.0, 0.0};   // identity: passthrough
}

// ---- runtime configuration --------------------------------------------------

template <std::size_t NCH>
void SensorModel<NCH>::SetChannel(std::size_t i, const Channel& c)
{
    if (i < NCH) m_ch[i] = c;
}

template <std::size_t NCH>
void SensorModel<NCH>::SetEnabled(std::size_t i, bool on)
{
    if (i < NCH) m_ch[i].enabled = on;
}

template <std::size_t NCH>
void SensorModel<NCH>::SetBias(std::size_t i, double bias)
{
    if (i < NCH) m_ch[i].bias = bias;
}

template <std::size_t NCH>
void SensorModel<NCH>::SetNoiseStd(std::size_t i, double noiseStd)
{
    if (i < NCH) m_ch[i].noiseStd = noiseStd;
}

template <std::size_t NCH>
typename SensorModel<NCH>::Channel SensorModel<NCH>::GetChannel(std::size_t i) const
{
    return (i < NCH) ? m_ch[i] : Channel{true, 0.0, 0.0};
}

template <std::size_t NCH>
void SensorModel<NCH>::Reseed(std::uint64_t seed)
{
    m_rng      = seed;
    m_hasSpare = false;
}

// ---- PRNG -------------------------------------------------------------------

template <std::size_t NCH>
std::uint64_t SensorModel<NCH>::NextU64(void)
{
    std::uint64_t z = (m_rng += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

template <std::size_t NCH>
double SensorModel<NCH>::NextUniform(void)
{
    // 53-bit mantissa uniform in [0, 1).
    return static_cast<double>(NextU64() >> 11) * (1.0 / 9007199254740992.0);
}

template <std::size_t NCH>
double SensorModel<NCH>::NextGaussian(void)
{
    // Box-Muller, caching the second deviate so both are used.
    if (m_hasSpare)
    {
        m_hasSpare = false;
        return m_spare;
    }
    // u1 in (0, 1] to keep log() finite.
    const double u1 = 1.0 - NextUniform();
    const double u2 = NextUniform();
    const double r  = std::sqrt(-2.0 * std::log(u1));
    const double a  = 2.0 * 3.14159265358979323846 * u2;
    m_spare    = r * std::sin(a);
    m_hasSpare = true;
    return r * std::cos(a);
}

// ---- apply ------------------------------------------------------------------

template <std::size_t NCH>
void SensorModel<NCH>::Apply(const std::array<double, NCH>& truth,
                             std::array<double, NCH>&       meas,
                             std::array<bool,   NCH>&       valid)
{
    for (std::size_t i = 0; i < NCH; ++i)
    {
        if (!m_ch[i].enabled)
        {
            meas[i]  = truth[i];   // passthrough; caller must gate on valid[i]
            valid[i] = false;
            continue;
        }
        double y = truth[i] + m_ch[i].bias;
        if (m_ch[i].noiseStd > 0.0) y += m_ch[i].noiseStd * NextGaussian();
        meas[i]  = y;
        valid[i] = true;
    }
}

}}  // namespace CDS::sensor

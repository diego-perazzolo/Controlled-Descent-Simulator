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
// File        : rk4.hpp
// Description : Generic fixed-control 4th-order Runge-Kutta step. Domain-agnostic
//               numerics: integrates any dx/dt = deriv(x) supplied as a callable.
//               It carries no control law and no projection -- the caller builds
//               the derivative closure (capturing its command, forces, model) and
//               applies any post-step projection (e.g. quaternion renorm) itself.
//               Header-only; depends only on <array>, so it may live under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <cstddef>

namespace CDS { namespace integrate {

// -----------------------------------------------------------------------------
// Example (standalone) -- advance a damped oscillator x'' = -x - 0.1 x' by one
// step. The state is [position, velocity]; the caller builds the derivative
// closure (capturing any control/forces) and reprojects afterwards if needed:
//
//   using State = std::array<double, 2>;
//   State x{{1.0, 0.0}};
//   const double u = 0.0;                              // constant control, if any
//   auto deriv = [&](const State& s) -> State {
//       return {{ s[1], -s[0] - 0.1 * s[1] + u }};     // dx/dt = f(x, u)
//   };
//   x = CDS::integrate::rk4_step<2>(x, 0.01, deriv);   // advance by dt = 0.01 s
//   // If the state has a constraint (e.g. a unit quaternion), reproject x here.
// -----------------------------------------------------------------------------

// Advance x by dt with classic RK4, holding the control fixed over the step.
// `deriv` maps a state to its time-derivative:
//   std::array<double, N> deriv(const std::array<double, N>& x)
template <std::size_t N, class Deriv>
std::array<double, N> rk4_step(const std::array<double, N>& x, double dt, Deriv&& deriv)
{
    const std::array<double, N> k1 = deriv(x);
    std::array<double, N> x2{}, x3{}, x4{};
    for (std::size_t i = 0; i < N; ++i) x2[i] = x[i] + 0.5 * dt * k1[i];
    const std::array<double, N> k2 = deriv(x2);
    for (std::size_t i = 0; i < N; ++i) x3[i] = x[i] + 0.5 * dt * k2[i];
    const std::array<double, N> k3 = deriv(x3);
    for (std::size_t i = 0; i < N; ++i) x4[i] = x[i] + dt * k3[i];
    const std::array<double, N> k4 = deriv(x4);

    std::array<double, N> xn{};
    for (std::size_t i = 0; i < N; ++i)
        xn[i] = x[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    return xn;
}

}} // namespace CDS::integrate

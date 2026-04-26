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

#include "Poly4.hpp"

using namespace CDS;

Poly4::Poly4()
 {
    m_startTime = 0;
    m_endTime = 20; // seconds
 }

Poly4::~Poly4()
{

}
        
  bool Poly4::GetReference(const core_coord_t&  time, Reference_t& ref)
  {
    core_coord_t t = time;  // local alias
    
    if(t > m_endTime) t = m_endTime;
    else if(t < m_startTime) t = m_startTime;
    
    // pos
    ref.pos[0] = ((((0.0009375)*t - 0.05)*t + 0.75)*t + 0)*t - 50.0;
    ref.pos[1] = ((((-0.0015625)*t + 0.0875)*t - 1.5)*t + 5.0)*t + 50.0;
    ref.pos[2] = ((((0.0034375)*t - 0.225)*t + 5.25)*t - 50.0)*t + 150.0;

    // vel
    ref.vel[0] = (((0.00375)*t - 0.15)*t + 1.5)*t + 0;
    ref.vel[1] = (((-0.00625)*t + 0.2625)*t - 3.0)*t + 5.0;
    ref.vel[2] = (((0.01375)*t - 0.675)*t + 10.5)*t - 50.0;

    // acc
    ref.acc[0] = ((0.01125)*t - 0.3)*t + 1.5;
    ref.acc[1] = ((-0.01875)*t + 0.525)*t - 3.0;
    ref.acc[2] = ((0.04125)*t - 1.35)*t + 10.5;

    // jerk
    ref.jerk[0] = (0.0225)*t - 0.3;
    ref.jerk[1] = (-0.0375)*t + 0.525;
    ref.jerk[2] = (0.0825)*t - 1.35;

    // snap
    ref.snap[0] = 0.0225;
    ref.snap[1] = -0.0375;
    ref.snap[2] = 0.0825;

    // TODO DP: registra anche tempo finale traiettoria

    return false;
  }

  bool Poly4::SetParameters(const std::map<std::string, core_coord_t>& params)
  {

    return false;
  }

  bool Poly4::GetParameters(std::map<std::string, core_coord_t>& params)
  {

    return false;
  }

  bool Poly4::SetParameter(const core_coord_t& p, size_t paramIdx)
  {

    return false;
  }

  bool Poly4::GetParameter(core_coord_t& p, size_t paramIdx)
  {

    return false;
  }
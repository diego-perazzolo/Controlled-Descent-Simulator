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
#pragma once
#include "Trajectory.hpp"

namespace CDS
{
    class Poly4 : public Trajectory
    {

        public:

        Poly4();
        virtual ~Poly4() override;

        /* Virtual methods */
         /* Gets reference trajectory state at a time instant. Returns true on error */
        virtual bool GetReference(const core_coord_t&  time, Reference_t& ref) override;

        /* Set dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool SetParameters(const std::map<std::string, core_coord_t>& params) override;

        /* Get dictionary of trajectory parameters and their value. Returns true on error */
        virtual bool GetParameters(std::map<std::string, core_coord_t>& params) override;

        /* Set trajectory parameter. Returns true on error */
        virtual bool SetParameter(const core_coord_t& p, size_t paramIdx) override;

        /* Get trajectory parameter. Returns true on error */
        virtual bool GetParameter(core_coord_t& p, size_t paramIdx) override;

    };
};
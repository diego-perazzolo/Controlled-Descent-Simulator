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
// File        : UniqueFile.hpp
// Description : Shared helper that turns a base output path into a unique one, so
//               the diagnostics file sinks (log, profiler raw CSV, data recorder
//               CSV) never overwrite a previous run and reopen a FRESH file each
//               time serialization is toggled back on. It inserts a
//               "_YYYYmmdd_HHMMSS_<n>" stamp before the extension; <n> is a
//               process-wide monotonic counter so two opens within the same
//               second never collide. Consumer-side only (off the tick path), so
//               ordinary blocking calls are fine. Header-only; standard library
//               only, so it may live under libs/ (no app/core includes).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <atomic>
#include <cstring>
#include <ctime>
#include <string>

namespace cds_log
{

    // "out_data/cds.log" -> "out_data/cds_20260815_143201_1.log".
    // A path with no extension just gets the stamp appended. Each call yields a
    // distinct name.
    inline std::string uniqueFilePath(const char* base)
    {
        std::string p = (base && base[0]) ? base : "out";

        // Split off the extension, but only a '.' that comes after the last path
        // separator (so a dotted directory does not fool it).
        const std::string::size_type slash = p.find_last_of("/\\");
        const std::string::size_type dot   = p.find_last_of('.');
        std::string stem = p;
        std::string ext;
        if (dot != std::string::npos && (slash == std::string::npos || dot > slash))
        {
            stem = p.substr(0, dot);
            ext  = p.substr(dot); // keeps the leading '.'
        }

        std::time_t t = std::time(nullptr);
        std::tm     tmv{};
#if defined(_WIN32)
        localtime_s(&tmv, &t);
#else
        localtime_r(&t, &tmv);
#endif
        char stamp[24];
        std::strftime(stamp, sizeof(stamp), "%Y%m%d_%H%M%S", &tmv);

        static std::atomic<unsigned> counter{0};
        const unsigned n = counter.fetch_add(1, std::memory_order_relaxed) + 1;

        return stem + "_" + stamp + "_" + std::to_string(n) + ext;
    }

} // namespace cds_log

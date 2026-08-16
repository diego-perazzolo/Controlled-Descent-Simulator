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
// File        : ProfileReport.hpp
// Description : Human-readable dump of a profiler Snapshot: one row per scope
//               with count and mean/min/max/stddev in microseconds. This is the
//               consumer side of the profiler's file sink (the server rewinds a
//               file and writes the latest snapshot periodically). Kept separate
//               from profile.hpp so the hot-path header stays free of <cstdio>.
//               Header-only; may live under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <cstdio>

#include "profile.hpp"

namespace cds_profile
{

    // Write a snapshot as a text table to a C stream. Module/scope names and the
    // scope kind are read from the registry (the snapshot stores stats by scope
    // id, in the same order registry() assigns them). Timed scopes are shown in
    // microseconds ("us"); value scopes show the raw quantity ("val"). Get the
    // snapshot from Registry::snapshot() (fed by pump()).
    inline void writeReport(std::FILE* stream, Registry& reg, const Snapshot& snap)
    {
        std::fprintf(stream, "%-14s %-14s %4s %8s %10s %10s %10s %10s %10s %10s %10s\n",
                     "module", "scope", "kind", "count",
                     "mean", "std", "min", "max", "p50", "p95", "p99");
        for (std::size_t i = 0; i < snap.count; ++i)
        {
            const scopeId_t s = static_cast<scopeId_t>(i);
            const ScopeStats& st = snap.stats[i];
            const bool isVal = reg.scopeIsValue(s);
            const double k = isVal ? 1.0 : 1.0 / 1000.0; // ns -> us for timed scopes
            std::fprintf(stream,
                         "%-14s %-14s %4s %8llu %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f %10.3f\n",
                         reg.moduleName(reg.scopeModule(s)), reg.scopeName(s),
                         isVal ? "val" : "us",
                         static_cast<unsigned long long>(st.count),
                         st.mean() * k, st.stddev() * k, st.min * k, st.max * k,
                         st.p50 * k, st.p95 * k, st.p99 * k);
        }
    }

    // CSV of the raw sample stream, for offline statistical analysis. `value`
    // is raw (nanoseconds for a timed scope, the recorded value for a value
    // scope — `kind` says which). Write the header once, then a line per sample.
    inline void writeRawHeader(std::FILE* stream)
    {
        std::fprintf(stream, "timestamp_ns,module,scope,kind,value\n");
    }

    inline void writeRawSample(std::FILE* stream, Registry& reg, const RawSample& r)
    {
        std::fprintf(stream, "%llu,%s,%s,%s,%.6f\n",
                     static_cast<unsigned long long>(r.timestampNs),
                     reg.moduleName(reg.scopeModule(r.scope)), reg.scopeName(r.scope),
                     reg.scopeIsValue(r.scope) ? "val" : "ns",
                     r.value);
    }

} // namespace cds_profile

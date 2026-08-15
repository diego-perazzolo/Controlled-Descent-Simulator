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
// File        : profile_test.cpp
// Description : Self-contained acid test for the scope profiler (libs/profile).
//               Checks: (1) a disabled module records nothing (opt-in); (2) an
//               enabled scope accumulates a coherent aggregate (count exact,
//               min <= mean <= max, stddev finite); (3) two scopes stay
//               independent; (4) a published snapshot round-trips through the
//               wait-free TripleBuffer mailbox and matches the live aggregates.
//               Timing uses a small busy loop; only structural invariants are
//               asserted, never absolute nanoseconds.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "profile.hpp"

#include <cstdio>

using namespace cds_profile;

namespace {

int _failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);\
            ++_failures;                                               \
        }                                                              \
    } while (0)

// a little wall-clock work so durations are non-zero
volatile double _sink = 0.0;
void burn(int n)
{
    double acc = 0.0;
    for (int i = 0; i < n; ++i) acc += std::sqrt(static_cast<double>(i) + 1.0);
    _sink = acc;
}

} // namespace

int main()
{
    const moduleId_t solver = registry().module("ilqr");
    CHECK(solver != INVALID_MODULE);
    CHECK(registry().module("ilqr") == solver); // idempotent

    // ---- (1) disabled module records nothing ----
    for (int i = 0; i < 5; ++i)
    {
        CDS_PROFILE(solver, "backward");
        burn(2000);
    }
    // scope id resolves even while disabled
    const scopeId_t backward = registry().scope(solver, "backward");
    CHECK(backward != INVALID_SCOPE);
    CHECK(registry().stats(backward).count == 0); // nothing recorded: module was off

    // ---- (2) enabled scope accumulates a coherent aggregate ----
    registry().setEnabled(solver, true);
    const int reps = 20;
    for (int i = 0; i < reps; ++i)
    {
        CDS_PROFILE(solver, "backward");
        burn(2000);
    }
    ScopeStats st = registry().stats(backward);
    CHECK(st.count == static_cast<std::uint64_t>(reps));
    CHECK(st.min > 0.0);
    CHECK(st.max >= st.min);
    CHECK(st.mean() >= st.min && st.mean() <= st.max);
    CHECK(std::isfinite(st.stddev()) && st.stddev() >= 0.0);
    CHECK(std::isfinite(st.variance()) && st.variance() >= 0.0);
    CHECK(st.p50 >= st.min && st.p99 <= st.max);

    // ---- (3) a second scope stays independent ----
    const scopeId_t forward = registry().scope(solver, "forward");
    CHECK(forward != backward);
    for (int i = 0; i < 3; ++i)
    {
        CDS_PROFILE(solver, "forward");
        burn(500);
    }
    CHECK(registry().stats(forward).count == 3);
    CHECK(registry().stats(backward).count == static_cast<std::uint64_t>(reps)); // untouched

    // ---- (3c) value scope: arbitrary values + streaming quantiles ----
    for (int i = 1; i <= 1000; ++i) CDS_PROFILE_VALUE(solver, "residual", static_cast<double>(i));
    const scopeId_t resid = registry().scope(solver, "residual", true);
    ScopeStats vs = registry().stats(resid);
    CHECK(vs.count == 1000);
    CHECK(vs.min == 1.0 && vs.max == 1000.0);
    CHECK(std::fabs(vs.mean() - 500.5) < 1.0);
    CHECK(std::fabs(vs.p50 - 500.0) < 40.0); // P-square is approximate
    CHECK(std::fabs(vs.p95 - 950.0) < 40.0);
    CHECK(registry().scopeIsValue(resid) == true);
    CHECK(registry().scopeIsValue(backward) == false);

    // ---- (4) snapshot round-trips: publish (writer) -> pump (reader) -> read ----
    registry().publish();
    CHECK(registry().pump() == false); // a snapshot was moved into the cache
    Snapshot snap;
    CHECK(registry().snapshot(snap) == false);
    CHECK(snap.count == registry().scopeCount());
    CHECK(snap.stats[backward].count == registry().stats(backward).count);
    CHECK(snap.stats[forward].count == registry().stats(forward).count);

    if (_failures)
    {
        std::printf("PROFILE TEST FAILED (%d checks)\n", _failures);
        return 1;
    }
    std::printf("PROFILE TEST PASSED\n");
    return 0;
}

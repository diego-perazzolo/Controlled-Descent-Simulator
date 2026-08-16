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
// File        : log_test.cpp
// Description : Self-contained acid test for the deferred-formatting logger
//               (libs/log). Certifies the property that makes the design worth
//               it: the hot path serialises the *values*, and formatting happens
//               later in drain(). Checks: (1) deferred formatting reproduces the
//               values as they were at log time even if the source variable is
//               mutated before drain; (2) mixed argument types (ints, doubles,
//               strings) round-trip through the thunk; (3) per-module runtime
//               level filtering; (4) full-ring drop counting (no blocking). A
//               small ring capacity is set below to exercise the drop path fast.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

// small ring so the drop-path test overflows quickly
#define CDS_LOG_RING_CAPACITY 16

#include "log.hpp"

#include <cstdio>
#include <string>
#include <vector>

using namespace cds_log;

namespace {

int _failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond);\
            ++_failures;                                               \
        }                                                              \
    } while (0)

// captures formatted lines so the test can inspect them
class CaptureSink : public Sink
{
    public:
    void Write(const LogLine& line) override
    {
        lines.emplace_back(line.text);
        levels.push_back(line.level);
        modules.emplace_back(line.moduleName);
    }
    void clear() { lines.clear(); levels.clear(); modules.clear(); }

    std::vector<std::string> lines;
    std::vector<Level>       levels;
    std::vector<std::string> modules;
};

bool contains(const std::string& hay, const char* needle)
{
    return hay.find(needle) != std::string::npos;
}

} // namespace

int main()
{
    CaptureSink sink;
    registry().addSink(&sink);

    const moduleId_t ilqr = registry().module("ilqr");
    const moduleId_t rk4  = registry().module("rk4");
    CHECK(ilqr != INVALID_MODULE);
    CHECK(rk4 != INVALID_MODULE);
    CHECK(registry().module("ilqr") == ilqr); // idempotent
    CHECK(registry().count() == 2);

    // the default level is Warn; these tests exercise Info/Debug, so lower it
    registry().setLevel(ilqr, Level::Info);
    registry().setLevel(rk4, Level::Info);

    // ---- (1) deferred formatting: value captured at log time, not at drain ----
    int iter = 41;
    double resid = 1.5;
    CDS_LOG_INFO(ilqr, "iter {} resid {}", iter, resid);
    iter = 99;      // mutate AFTER logging, BEFORE draining
    resid = -7.0;
    registry().drain();
    CHECK(sink.lines.size() == 1);
    CHECK(contains(sink.lines[0], "iter 41"));   // the old value, proving it was serialised
    CHECK(contains(sink.lines[0], "resid 1.5"));
    CHECK(sink.levels[0] == Level::Info);
    CHECK(sink.modules[0] == "ilqr");

    // ---- (2) mixed argument types incl. inline-copied strings ----
    sink.clear();
    const char* phase = "backward";
    CDS_LOG_WARN(rk4, "phase {} step {} dt {}", phase, 7, 0.02);
    registry().drain();
    CHECK(sink.lines.size() == 1);
    CHECK(contains(sink.lines[0], "phase backward"));
    CHECK(contains(sink.lines[0], "step 7"));
    CHECK(contains(sink.lines[0], "dt 0.02"));

    // string copied inline: mutating the buffer after logging must not change it
    sink.clear();
    char buf[16];
    std::strcpy(buf, "alpha");
    CDS_LOG_INFO(ilqr, "tag {}", static_cast<const char*>(buf));
    std::strcpy(buf, "omega");
    registry().drain();
    CHECK(sink.lines.size() == 1);
    CHECK(contains(sink.lines[0], "tag alpha"));

    // ---- (3) per-module runtime level filtering ----
    sink.clear();
    // default level is Info: Debug is below and must be dropped
    CDS_LOG_DEBUG(ilqr, "should be filtered {}", 1);
    registry().drain();
    CHECK(sink.lines.empty());

    registry().setLevel(ilqr, Level::Debug);
    CDS_LOG_DEBUG(ilqr, "now visible {}", 2);
    registry().drain();
    CHECK(sink.lines.size() == 1);
    CHECK(contains(sink.lines[0], "now visible 2"));
    registry().setLevel(ilqr, Level::Info);

    // ---- (3b) per-module sampling: a call site emits 1 message in N ----
    sink.clear();
    registry().setSample(ilqr, 5);
    for (int i = 0; i < 10; ++i) CDS_LOG_INFO_SAMPLED(ilqr, "sampled {}", i);
    registry().drain();
    CHECK(sink.lines.size() == 2); // i=0 and i=5 (this call site's own counter)
    if (sink.lines.size() == 2)
    {
        CHECK(contains(sink.lines[0], "sampled 0"));
        CHECK(contains(sink.lines[1], "sampled 5"));
    }
    registry().setSample(ilqr, 1); // reset: emit all

    // ---- (4) full-ring drop counting: never block, drop-and-count ----
    sink.clear();
    const std::uint64_t dropsBefore = registry().drops();
    const int overshoot = 20; // > ring capacity (16)
    for (int i = 0; i < overshoot; ++i) // no drain in between: ring fills, rest dropped
    {
        CDS_LOG_INFO(rk4, "flood {}", i);
    }
    const std::uint64_t newDrops = registry().drops() - dropsBefore;
    CHECK(newDrops == static_cast<std::uint64_t>(overshoot - CDS_LOG_RING_CAPACITY));
    const std::size_t drained = registry().drain();
    CHECK(drained == CDS_LOG_RING_CAPACITY);

    if (_failures)
    {
        std::printf("LOG TEST FAILED (%d checks)\n", _failures);
        return 1;
    }
    std::printf("LOG TEST PASSED\n");
    return 0;
}

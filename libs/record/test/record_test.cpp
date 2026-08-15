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
// File        : record_test.cpp
// Description : Self-contained acid test of libs/record (Recorder.hpp), reusing
//               the wait-free libs/log/LogRing.hpp. Exercises: schema + metadata
//               header, record -> drain -> wide CSV round-trip (values preserved,
//               row count, t_sim as channel 0), disabled recorder is a no-op,
//               explicit drop counting when the ring overflows, and the
//               type-erased IRecorder / activeRecorder() handoff. No core, no
//               protocol, no threads required. Build and run:
//                 cmake -S libs/record/test -B build-record-test \
//                       -DCMAKE_BUILD_TYPE=Release
//                 cmake --build build-record-test
//                 ./build-record-test/record_test
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <array>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "Recorder.hpp"

// ---- tiny check harness -----------------------------------------------------
static int g_failures = 0;
#define CHECK(cond, msg)                                                        \
    do {                                                                        \
        if (!(cond)) { std::printf("  FAIL: %s\n", (msg)); ++g_failures; }      \
        else         { std::printf("  ok  : %s\n", (msg)); }                    \
    } while (0)

// Slurp a whole file into a string (small test files only).
static std::string slurp(const char* path)
{
    std::FILE* f = std::fopen(path, "rb");
    if (!f) return {};
    std::string out;
    char buf[4096];
    std::size_t n;
    while ((n = std::fread(buf, 1, sizeof(buf), f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

static std::size_t countChar(const std::string& s, char c)
{
    std::size_t n = 0;
    for (char ch : s) if (ch == c) ++n;
    return n;
}

int main()
{
    std::printf("== record_test ==\n");

    // A small, quad-like schema: t_sim + 4 fake channels.
    constexpr std::size_t N = 5;
    const std::array<const char*, N> names{ "t_sim", "x", "y", "z", "u0" };

    // -------------------------------------------------------------------------
    // 1. Disabled recorder is a no-op (nothing captured, nothing dropped).
    // -------------------------------------------------------------------------
    {
        cds_record::Recorder<double, N> rec("Test", names);
        for (int i = 0; i < 100; ++i)
        {
            std::array<double, N> row{ double(i), 1.0, 2.0, 3.0, 4.0 };
            CDS_RECORD(rec, row);
        }
        const char* path = "record_test_disabled.csv";
        std::FILE* f = std::fopen(path, "w");
        rec.writeHeader(f);
        const std::size_t n = rec.drainRows(f);
        std::fclose(f);
        CHECK(n == 0, "disabled recorder captures no rows");
        CHECK(rec.dropped() == 0, "disabled recorder drops nothing");
        std::remove(path);
    }

    // -------------------------------------------------------------------------
    // 2. Enabled: record -> drain -> CSV round-trip with metadata + header.
    // -------------------------------------------------------------------------
    {
        cds_record::Recorder<double, N> rec("Quadrotor MPC", names);
        rec.clearMeta();
        rec.addMeta("mass_kg", 1.5);
        rec.addMeta("trajectory_items", (long long)4);
        rec.setEnabled(true);

        const int ROWS = 64;
        for (int i = 0; i < ROWS; ++i)
        {
            std::array<double, N> row{ 0.01 * i, 10.0 + i, 20.0 + i, 30.0 + i, 0.25 * i };
            CDS_RECORD(rec, row);
        }

        const char* path = "record_test_run.csv";
        std::FILE* f = std::fopen(path, "w");
        rec.writeHeader(f);
        const std::size_t n = rec.drainRows(f);
        rec.writeTrailer(f);
        std::fclose(f);

        CHECK(n == (std::size_t)ROWS, "all enabled rows are drained");
        CHECK(rec.dropped() == 0, "no drops well under ring capacity");

        const std::string csv = slurp(path);
        CHECK(csv.find("# recorder: Quadrotor MPC") != std::string::npos,
              "metadata carries the recorder name");
        CHECK(csv.find("# mass_kg: 1.5") != std::string::npos,
              "metadata carries a double field");
        CHECK(csv.find("# trajectory_items: 4") != std::string::npos,
              "metadata carries an integer field");
        CHECK(csv.find("t_wall_ns,t_sim,x,y,z,u0") != std::string::npos,
              "column header is t_wall_ns + channel names in order");
        CHECK(csv.find("# dropped: 0 rows") != std::string::npos,
              "trailer reports the drop count");
        // ROWS data lines + header + recorder-name + 2 meta + trailer comments.
        const std::size_t dataLines = countChar(csv, '\n') - 1 /*hdr*/ - 1 /*name*/
                                      - 2 /*meta*/ - 1 /*trailer*/;
        CHECK(dataLines == (std::size_t)ROWS, "exactly ROWS data lines emitted");
        // spot-check a value survived the round-trip (row 10, channel x = 20)
        CHECK(csv.find(",20,") != std::string::npos || csv.find(",20.") != std::string::npos
              || csv.find(",20\n") != std::string::npos,
              "a channel value survives the CSV round-trip");
        std::remove(path);
    }

    // -------------------------------------------------------------------------
    // 3. Explicit drop counting: overflow a tiny ring and verify the count.
    //    Capacity 8 ring, push 20 without draining -> 12 dropped, 8 kept.
    // -------------------------------------------------------------------------
    {
        constexpr std::size_t CAP = 8;
        cds_record::Recorder<double, N, CAP> rec("Tiny", names);
        rec.setEnabled(true);
        const int PUSHES = 20;
        for (int i = 0; i < PUSHES; ++i)
        {
            std::array<double, N> row{ double(i), 0, 0, 0, 0 };
            CDS_RECORD(rec, row);
        }
        const char* path = "record_test_drop.csv";
        std::FILE* f = std::fopen(path, "w");
        const std::size_t kept = rec.drainRows(f);
        std::fclose(f);
        std::remove(path);
        CHECK(kept == CAP, "a full ring keeps exactly Capacity rows");
        CHECK(rec.dropped() == (std::uint64_t)(PUSHES - CAP),
              "dropped count is explicit and exact (PUSHES - Capacity)");
    }

    // -------------------------------------------------------------------------
    // 4. Type-erased handoff: the active recorder is reachable via IRecorder*.
    // -------------------------------------------------------------------------
    {
        CHECK(cds_record::activeRecorder() == nullptr, "no active recorder initially");
        cds_record::Recorder<double, N> rec("Active", names);
        rec.activate();
        cds_record::IRecorder* ir = cds_record::activeRecorder();
        CHECK(ir == &rec, "activate() publishes this recorder to the active slot");
        CHECK(ir != nullptr && std::strcmp(ir->name(), "Active") == 0,
              "active recorder is reachable by name through the base interface");
        ir->setEnabled(true);
        CHECK(ir->enabled(), "toggle through the type-erased interface works");
        cds_record::setActiveRecorder(nullptr); // reset for tidiness
    }

    std::printf(g_failures == 0 ? "\nALL PASS\n" : "\n%d FAILURE(S)\n", g_failures);
    return g_failures == 0 ? 0 : 1;
}

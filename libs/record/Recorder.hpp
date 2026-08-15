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
// File        : Recorder.hpp
// Description : Per-tick "black box" data recorder (lossless wide telemetry).
//               A third thing next to the logger (text, lossy) and the profiler
//               (aggregates, lossy): it captures, ONCE PER TICK, a full wide row
//               -- state + input + reference + tracking error + disturbance --
//               for a single active model, so runs can be validated and compared
//               offline. The tick thread pushes fixed-width rows into a wait-free
//               MPSC ring (reused from libs/log/LogRing.hpp); an off-tick drain
//               thread on the server writes them out as a wide CSV with a run
//               metadata header. A full ring drops-and-COUNTS (explicit, unlike
//               the profiler raw stream): losing rows is always observable.
//
//               Row width N and value type T are compile-time template params so
//               every model has its exact row with no padding (quad quaternion:
//               T=double, N=32). Because differently-shaped models are distinct
//               Recorder<T,N> types, the process-wide drain/toggle talk to the
//               currently-running model through the type-erased IRecorder base
//               and a single activeRecorder() slot -- one model records at a time.
//
//               Layering: infrastructure only. Depends on libs/log/LogRing.hpp
//               and the standard library; never includes app or core headers
//               (golden rule 3). Header-only.
//
// Usage (the model owns a static Recorder in its .cpp and fills the calls):
//
//     // channel 0 is always t_sim [s]; the rest are model-defined
//     static cds_record::Recorder<double, 32> g_rec("Quadrotor MPC",
//         { "t_sim", "x","y","z", "qw","qx","qy","qz", ... });
//     ...
//     g_rec.activate();                 // make this the active recorder
//     g_rec.clearMeta();                // at run start, then describe the run:
//     g_rec.addMeta("mass_kg", p.m);
//     ...
//     std::array<double,32> row{ m_time, m_state[0], ... };
//     CDS_RECORD(g_rec, row);           // wait-free push on the tick thread
//
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <mutex>
#include <string>

#include "LogRing.hpp" // generic wait-free MPSC ring (reused, cds_log::LogRing)

// Compile-time master switch. Define CDS_RECORD_ENABLED=0 to compile CDS_RECORD
// down to nothing (zero cost, for performance baselining). Default: on.
#ifndef CDS_RECORD_ENABLED
#define CDS_RECORD_ENABLED 1
#endif

// Ring capacity (rows buffered between drains). Power of two. One row per tick;
// sized so an off-tick drain keeps up at a high tick rate without dropping.
#ifndef CDS_RECORD_CAPACITY
#define CDS_RECORD_CAPACITY 8192
#endif

// Sanity ceiling on channel count, so a wrong N cannot silently produce a
// monstrous CSV. Templating on N means there is no padding; this is a guard.
#ifndef CDS_RECORD_MAX_CHANNELS
#define CDS_RECORD_MAX_CHANNELS 48
#endif

namespace cds_record
{

    // Steady, monotonic wall time in nanoseconds. Used only to order rows and
    // reveal gaps left by dropped rows; the model's own simulation time rides as
    // channel 0 (t_sim), which is the real x-axis for offline analysis.
    inline std::uint64_t nowNs()
    {
        return static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch())
                .count());
    }

    // One recorded row: a monotonic wall timestamp plus the N model channels
    // (channel 0 is t_sim). POD, memcpy-friendly; lives in the ring cell.
    template <typename T, std::size_t N>
    struct Row
    {
        std::uint64_t t_wall_ns;
        T             values[N];
    };

    // ------------------------------------------------------------------------ //
    // Type-erased handle. The server drain thread and the ext toggle hold an     //
    // IRecorder* to "the active model's recorder" without knowing its T/N. Only  //
    // one recorder is active at a time (one model runs).                         //
    // ------------------------------------------------------------------------ //
    class IRecorder
    {
        public:

        virtual ~IRecorder() = default;

        virtual const char* name() const = 0;

        // Toggle capture. When off, record() is a no-op and the ring stays idle.
        virtual void setEnabled(bool on) = 0;
        virtual bool enabled() const     = 0;

        // Consumer side (single drain thread / tick-end). writeHeader emits the
        // metadata block + column header (call once per freshly opened file);
        // drainRows flushes all pending rows and returns how many were written;
        // writeTrailer emits the final "# dropped N rows" accounting at close.
        virtual void        writeHeader(std::FILE* f) = 0;
        virtual std::size_t drainRows(std::FILE* f)   = 0;
        virtual void        writeTrailer(std::FILE* f) = 0;

        // Explicit, always-observable count of rows lost to a full ring.
        virtual std::uint64_t dropped() const = 0;
    };

    // Process-wide slot for the active recorder. Set by a model when it becomes
    // the running one; read by the drain thread and the ext adapter.
    inline std::atomic<IRecorder*>& activeSlot()
    {
        static std::atomic<IRecorder*> slot{nullptr};
        return slot;
    }
    inline IRecorder* activeRecorder() { return activeSlot().load(std::memory_order_acquire); }
    inline void setActiveRecorder(IRecorder* r) { activeSlot().store(r, std::memory_order_release); }

    // ------------------------------------------------------------------------ //
    // The recorder: fixed-width wide telemetry for one model shape (T,N). The    //
    // model constructs one (static in its .cpp), names its channels once, and    //
    // pushes a row each tick. Header-only; the ring is the only heavyweight       //
    // member.                                                                    //
    // ------------------------------------------------------------------------ //
    template <typename T, std::size_t N, std::size_t Capacity = CDS_RECORD_CAPACITY>
    class Recorder : public IRecorder
    {
        static_assert(N >= 1, "a recorder needs at least the t_sim channel");
        static_assert(N <= CDS_RECORD_MAX_CHANNELS,
                      "channel count exceeds CDS_RECORD_MAX_CHANNELS guard");

        public:

        using Value = T;
        static constexpr std::size_t channels() { return N; }

        // Name the recorder and its N channels (string literals; not copied).
        // Channel 0 is expected to be t_sim by convention.
        Recorder(const char* name, const std::array<const char*, N>& channelNames)
            : m_name(name), m_names(channelNames), m_ring(), m_enabled(false),
              m_dropped(0), m_meta(), m_metaMutex()
        {
        }

        Recorder(const Recorder&)            = delete;
        Recorder& operator=(const Recorder&) = delete;

        const char* name() const override { return m_name; }

        // Make this the process-wide active recorder (the running model calls it).
        void activate() { setActiveRecorder(this); }

        void setEnabled(bool on) override { m_enabled.store(on, std::memory_order_relaxed); }
        bool enabled() const override     { return m_enabled.load(std::memory_order_relaxed); }

        std::uint64_t dropped() const override { return m_dropped.load(std::memory_order_relaxed); }

        // ---- run metadata (off-tick: called from setup/command threads) ------- //
        // Cleared at run start, then filled with everything useful; written as
        // "# key: value" comment lines ahead of the CSV header.
        void clearMeta()
        {
            std::lock_guard<std::mutex> lk(m_metaMutex);
            m_meta.clear();
        }
        void addMeta(const char* key, const char* value)
        {
            std::lock_guard<std::mutex> lk(m_metaMutex);
            m_meta += "# ";
            m_meta += key;
            m_meta += ": ";
            m_meta += value;
            m_meta += '\n';
        }
        void addMeta(const char* key, double value)
        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%.10g", value);
            addMeta(key, buf);
        }
        void addMeta(const char* key, long long value)
        {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "%lld", value);
            addMeta(key, buf);
        }

        // ---- hot path: push one row (tick thread), wait-free -------------------//
        // A full ring drops the row and bumps the explicit drop counter; it never
        // blocks and never allocates (golden rule 9). values[0] is t_sim.
        void record(const std::array<T, N>& values)
        {
            if (!m_enabled.load(std::memory_order_relaxed)) return;
            const bool full = m_ring.Produce([&](Row<T, N>& r) {
                r.t_wall_ns = nowNs();
                for (std::size_t i = 0; i < N; ++i) r.values[i] = values[i];
            });
            if (full) m_dropped.fetch_add(1, std::memory_order_relaxed);
        }

        // ---- consumer side (single drain thread) ------------------------------//
        void writeHeader(std::FILE* f) override
        {
            if (!f) return;
            {
                std::lock_guard<std::mutex> lk(m_metaMutex);
                std::fprintf(f, "# recorder: %s\n", m_name);
                std::fputs(m_meta.c_str(), f);
            }
            std::fputs("t_wall_ns", f);
            for (std::size_t i = 0; i < N; ++i) std::fprintf(f, ",%s", m_names[i]);
            std::fputc('\n', f);
        }

        std::size_t drainRows(std::FILE* f) override
        {
            if (!f) return 0;
            return m_ring.Drain([&](const Row<T, N>& r) {
                std::fprintf(f, "%llu", static_cast<unsigned long long>(r.t_wall_ns));
                for (std::size_t i = 0; i < N; ++i)
                    std::fprintf(f, ",%.10g", static_cast<double>(r.values[i]));
                std::fputc('\n', f);
            });
        }

        void writeTrailer(std::FILE* f) override
        {
            if (!f) return;
            std::fprintf(f, "# dropped: %llu rows\n",
                         static_cast<unsigned long long>(dropped()));
        }

        private:

        const char*                      m_name;
        std::array<const char*, N>       m_names;
        cds_log::LogRing<Row<T, N>, Capacity> m_ring;
        std::atomic<bool>                m_enabled;
        std::atomic<std::uint64_t>       m_dropped;
        std::string                      m_meta;
        mutable std::mutex               m_metaMutex;
    };

} // namespace cds_record

// Convenience macro so the recording call site compiles out entirely when
// CDS_RECORD_ENABLED=0 (performance baselining). rec is a Recorder<T,N>&, row a
// std::array<T,N>.
#if CDS_RECORD_ENABLED
#define CDS_RECORD(rec, row) do { (rec).record(row); } while (0)
#else
#define CDS_RECORD(rec, row) do { (void)sizeof(rec); (void)sizeof(row); } while (0)
#endif

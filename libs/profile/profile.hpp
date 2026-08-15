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
// File        : profile.hpp
// Description : Named, per-module scope profiler for fine timing of controllers
//               and integrators. A module is a named channel; each named scope
//               under it keeps a running aggregate {count, sum, min, max, sumsq}
//               of its wall-clock durations, from which mean and standard
//               deviation are derived. Measurement is RAII: CDS_PROFILE(mod,
//               "name") times the enclosing block. The hot path only takes two
//               steady_clock samples and accumulates into a fixed struct -- no
//               allocation, no I/O, no lock -- so it is safe inside the real-time
//               tick (golden rule 9). Aggregates are written by a single thread
//               per scope (the tick thread); a consistent snapshot is handed to
//               other threads wait-free via a cds_sync::TripleBuffer, published
//               on the writer side (typically once per tick). The whole facility
//               compiles away when CDS_PROFILE_ENABLED is 0. Header-only; depends
//               only on the standard library and libs/sync, so it may live under
//               libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "TripleBuffer.hpp"
#include "P2Quantile.hpp"

// --------------------------------------------------------------------------- //
// Compile-time configuration (override with -D before including)               //
// --------------------------------------------------------------------------- //
#ifndef CDS_PROFILE_MAX_MODULES
#define CDS_PROFILE_MAX_MODULES 32
#endif
#ifndef CDS_PROFILE_MAX_SCOPES
#define CDS_PROFILE_MAX_SCOPES 64
#endif
#ifndef CDS_PROFILE_NAME_MAX
#define CDS_PROFILE_NAME_MAX 32
#endif
// Set to 0 to compile out every CDS_PROFILE scope (zero overhead).
#ifndef CDS_PROFILE_ENABLED
#define CDS_PROFILE_ENABLED 1
#endif

namespace cds_profile
{

    typedef std::uint16_t moduleId_t;
    typedef std::uint32_t scopeId_t;
    static constexpr moduleId_t INVALID_MODULE = static_cast<moduleId_t>(-1);
    static constexpr scopeId_t  INVALID_SCOPE  = static_cast<scopeId_t>(-1);

    // Running aggregate of a scope's samples. The measured quantity is a wall
    // time in nanoseconds for a timed scope (CDS_PROFILE) or an arbitrary value
    // for a value scope (CDS_PROFILE_VALUE) — the scope's `isValue` kind tells
    // consumers which, and hence the unit. p50/p95/p99 are streaming estimates
    // (P-square). Single writer per scope; readers see it only through a
    // published snapshot copy.
    struct ScopeStats
    {
        std::uint64_t count;
        double        sum;
        double        min;
        double        max;
        double        sumsq;
        double        p50;
        double        p95;
        double        p99;

        double mean() const { return count ? sum / static_cast<double>(count) : 0.0; }
        double variance() const
        {
            if (count < 2) return 0.0;
            const double m = mean();
            const double v = sumsq / static_cast<double>(count) - m * m;
            return v > 0.0 ? v : 0.0;
        }
        double stddev() const { return std::sqrt(variance()); }
    };

    // A snapshot of every scope's aggregate at one instant, for cross-thread
    // reading via the TripleBuffer. Fixed size (POD), no heap.
    struct Snapshot
    {
        std::size_t count;
        ScopeStats  stats[CDS_PROFILE_MAX_SCOPES];
    };

    // ------------------------------------------------------------------------ //
    // The registry: module table, flat scope table (scope -> module + name +    //
    // live aggregate), and the wait-free snapshot mailbox. One process-wide      //
    // instance via registry().                                                   //
    // ------------------------------------------------------------------------ //
    class Registry
    {
        public:

        Registry() : m_moduleCount(0), m_scopeCount(0), m_cache{}, m_cacheValid(false)
        {
            for (std::size_t i = 0; i < CDS_PROFILE_MAX_SCOPES; ++i) resetStats(m_scopes[i].stats);
        }

        /* Register a module by name (idempotent) or find it. Returns its id or
           INVALID_MODULE if the table is full. Profiling is enabled per module;
           default is disabled (opt in from the frontend). Thread-safe. */
        moduleId_t module(const char* name)
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            for (std::size_t i = 0; i < m_moduleCount; ++i)
            {
                if (std::strncmp(m_modules[i].name, name, CDS_PROFILE_NAME_MAX) == 0)
                {
                    return static_cast<moduleId_t>(i);
                }
            }
            if (m_moduleCount >= CDS_PROFILE_MAX_MODULES) return INVALID_MODULE;
            const std::size_t id = m_moduleCount++;
            std::strncpy(m_modules[id].name, name, CDS_PROFILE_NAME_MAX - 1);
            m_modules[id].name[CDS_PROFILE_NAME_MAX - 1] = '\0';
            m_modules[id].enabled.store(false, std::memory_order_relaxed);
            return static_cast<moduleId_t>(id);
        }

        /* Register a scope (module + name), idempotent, or find it. isValue marks
           a value scope (CDS_PROFILE_VALUE) vs a timed scope; it only affects how
           consumers label the unit. Returns its id or INVALID_SCOPE if the table
           is full. Thread-safe. */
        scopeId_t scope(moduleId_t mod, const char* name, bool isValue = false)
        {
            if (mod == INVALID_MODULE) return INVALID_SCOPE;
            std::lock_guard<std::mutex> lk(m_mutex);
            for (std::size_t i = 0; i < m_scopeCount; ++i)
            {
                if (m_scopes[i].module == mod &&
                    std::strncmp(m_scopes[i].name, name, CDS_PROFILE_NAME_MAX) == 0)
                {
                    return static_cast<scopeId_t>(i);
                }
            }
            if (m_scopeCount >= CDS_PROFILE_MAX_SCOPES) return INVALID_SCOPE;
            const std::size_t id = m_scopeCount++;
            m_scopes[id].module = mod;
            m_scopes[id].isValue = isValue;
            std::strncpy(m_scopes[id].name, name, CDS_PROFILE_NAME_MAX - 1);
            m_scopes[id].name[CDS_PROFILE_NAME_MAX - 1] = '\0';
            m_scopes[id].pq[0].init(0.50);
            m_scopes[id].pq[1].init(0.95);
            m_scopes[id].pq[2].init(0.99);
            resetStats(m_scopes[id].stats);
            return static_cast<scopeId_t>(id);
        }

        void setEnabled(moduleId_t mod, bool on)
        {
            if (mod < m_moduleCount) m_modules[mod].enabled.store(on, std::memory_order_relaxed);
        }

        bool moduleEnabled(moduleId_t mod) const
        {
            return mod < m_moduleCount && m_modules[mod].enabled.load(std::memory_order_relaxed);
        }

        /* Whether a scope should be timed: cheap, called by every ScopeTimer
           constructor on the hot path. */
        bool scopeActive(scopeId_t s) const
        {
            if (s >= m_scopeCount) return false;
            return moduleEnabled(m_scopes[s].module);
        }

        /* Hot path: fold one sample (a duration in ns for a timed scope, or an
           arbitrary value for a value scope) into the scope aggregate and its
           streaming quantiles. Single writer per scope (the tick thread). No
           lock, no allocation. */
        void record(scopeId_t s, double v)
        {
            if (s >= m_scopeCount) return;
            Scope& sc = m_scopes[s];
            ScopeStats& st = sc.stats;
            if (st.count == 0 || v < st.min) st.min = v;
            if (st.count == 0 || v > st.max) st.max = v;
            st.sum += v;
            st.sumsq += v * v;
            ++st.count;
            sc.pq[0].Add(v);
            sc.pq[1].Add(v);
            sc.pq[2].Add(v);
            st.p50 = sc.pq[0].estimate();
            st.p95 = sc.pq[1].estimate();
            st.p99 = sc.pq[2].estimate();
        }

        std::size_t moduleCount() const { return m_moduleCount; }
        std::size_t scopeCount() const { return m_scopeCount; }
        const char* moduleName(moduleId_t m) const { return m < m_moduleCount ? m_modules[m].name : "?"; }
        const char* scopeName(scopeId_t s) const { return s < m_scopeCount ? m_scopes[s].name : "?"; }
        moduleId_t  scopeModule(scopeId_t s) const { return s < m_scopeCount ? m_scopes[s].module : INVALID_MODULE; }
        bool        scopeIsValue(scopeId_t s) const { return s < m_scopeCount && m_scopes[s].isValue; }

        /* Same-thread read of a live aggregate (tests, single-threaded use). For
           cross-thread reading use publish()/readLatest(). */
        ScopeStats stats(scopeId_t s) const
        {
            return s < m_scopeCount ? m_scopes[s].stats : ScopeStats{};
        }

        void resetScope(scopeId_t s) { if (s < m_scopeCount) resetScopeState(m_scopes[s]); }
        void resetAll() { for (std::size_t i = 0; i < m_scopeCount; ++i) resetScopeState(m_scopes[i]); }

        /* Writer side: hand the current aggregates to the reader as a wait-free
           snapshot (typically once per tick). Tick/writer thread only: the RT
           path stays wait-free (no lock), the mailbox absorbs the handoff.
           Skips the work entirely when no module is enabled (the default), so
           profiling off costs next to nothing on the tick. */
        void publish()
        {
            bool any = false;
            for (std::size_t i = 0; i < m_moduleCount; ++i)
            {
                if (m_modules[i].enabled.load(std::memory_order_relaxed)) { any = true; break; }
            }
            if (!any) return;

            Snapshot snap;
            snap.count = m_scopeCount;
            for (std::size_t i = 0; i < m_scopeCount; ++i) snap.stats[i] = m_scopes[i].stats;
            m_mailbox.Write(snap);
        }

        /* Single designated reader (the drain thread on the server, the tick
           thread at the drain-point in the single-threaded wasm build): move the
           latest published snapshot from the wait-free mailbox into the
           mutex-guarded UI cache, so several consumers (file dump + ext command)
           can read it. Returns false if a snapshot was pumped. Never runs on the
           tick thread on the server, so the lock here never blocks the tick. */
        bool pump()
        {
            Snapshot snap;
            if (m_mailbox.Read(snap)) return true; // nothing published yet
            std::lock_guard<std::mutex> lk(m_cacheMutex);
            m_cache = snap;
            m_cacheValid = true;
            return false;
        }

        /* Any consumer thread: copy the cached snapshot. Returns true (error) if
           nothing has been pumped yet. */
        bool snapshot(Snapshot& out)
        {
            std::lock_guard<std::mutex> lk(m_cacheMutex);
            if (!m_cacheValid) return true;
            out = m_cache;
            return false;
        }

        private:

        static void resetStats(ScopeStats& s) { s = ScopeStats{}; }

        struct Module
        {
            char              name[CDS_PROFILE_NAME_MAX];
            std::atomic<bool> enabled;
        };

        struct Scope
        {
            char        name[CDS_PROFILE_NAME_MAX];
            moduleId_t  module;
            bool        isValue;
            ScopeStats  stats;
            P2Quantile  pq[3]; // p50, p95, p99 (RT-private, not in the snapshot)
        };

        static void resetScopeState(Scope& s)
        {
            resetStats(s.stats);
            s.pq[0].reset();
            s.pq[1].reset();
            s.pq[2].reset();
        }

        Module      m_modules[CDS_PROFILE_MAX_MODULES];
        Scope       m_scopes[CDS_PROFILE_MAX_SCOPES];
        std::size_t m_moduleCount;
        std::size_t m_scopeCount;
        std::mutex  m_mutex; // guards registration only (not the hot path)

        cds_sync::TripleBuffer<Snapshot> m_mailbox; // wait-free RT -> reader handoff

        Snapshot   m_cache;      // UI-facing snapshot, filled by pump()
        bool       m_cacheValid; // false until the first pump()
        std::mutex m_cacheMutex; // guards the cache (touched off the tick path)
    };

    inline Registry& registry()
    {
        static Registry instance;
        return instance;
    }

    // ------------------------------------------------------------------------ //
    // RAII timer: samples steady_clock on construction and destruction and folds //
    // the elapsed nanoseconds into the scope aggregate. Does nothing when the    //
    // scope's module is disabled.                                                //
    // ------------------------------------------------------------------------ //
    class ScopeTimer
    {
        public:

        explicit ScopeTimer(scopeId_t s)
            : m_scope(s), m_active(registry().scopeActive(s))
        {
            if (m_active) m_start = std::chrono::steady_clock::now();
        }

        ~ScopeTimer()
        {
            if (!m_active) return;
            const std::chrono::duration<double, std::nano> elapsed =
                std::chrono::steady_clock::now() - m_start;
            registry().record(m_scope, elapsed.count());
        }

        ScopeTimer(const ScopeTimer&) = delete;
        ScopeTimer& operator=(const ScopeTimer&) = delete;

        private:

        scopeId_t                                      m_scope;
        bool                                           m_active;
        std::chrono::steady_clock::time_point          m_start;
    };

} // namespace cds_profile

// --------------------------------------------------------------------------- //
// Macros. CDS_PROFILE(mod, name) times the enclosing block. It registers the    //
// scope once (function-local static) and drops a ScopeTimer on the stack. When  //
// CDS_PROFILE_ENABLED is 0 it expands to nothing.                               //
// --------------------------------------------------------------------------- //
#define _CDS_PROFILE_CAT(a, b) _CDS_PROFILE_CAT_(a, b)
#define _CDS_PROFILE_CAT_(a, b) a##b

#if CDS_PROFILE_ENABLED
#define CDS_PROFILE(mod, name)                                                        \
    static const ::cds_profile::scopeId_t _CDS_PROFILE_CAT(_cds_prof_sid_, __LINE__) = \
        ::cds_profile::registry().scope((mod), (name));                               \
    ::cds_profile::ScopeTimer _CDS_PROFILE_CAT(_cds_prof_timer_, __LINE__)(           \
        _CDS_PROFILE_CAT(_cds_prof_sid_, __LINE__))

// Fold an arbitrary scalar value (a residual, an error norm, an iteration
// count, ...) into a value scope's statistics. `value` is evaluated only when
// the module is enabled.
#define CDS_PROFILE_VALUE(mod, name, value)                                            \
    do                                                                                 \
    {                                                                                  \
        static const ::cds_profile::scopeId_t _CDS_PROFILE_CAT(_cds_prof_vsid_, __LINE__) = \
            ::cds_profile::registry().scope((mod), (name), /*isValue=*/true);          \
        if (::cds_profile::registry().scopeActive(_CDS_PROFILE_CAT(_cds_prof_vsid_, __LINE__))) \
            ::cds_profile::registry().record(_CDS_PROFILE_CAT(_cds_prof_vsid_, __LINE__), \
                                             static_cast<double>(value));              \
    } while (0)
#else
#define CDS_PROFILE(mod, name) ((void)0)
#define CDS_PROFILE_VALUE(mod, name, value) ((void)0)
#endif

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
// File        : log.hpp
// Description : Named, per-module logger with deferred (postponed) formatting.
//               A module is a named channel with a runtime level; code obtains a
//               ModuleId once and logs through the CDS_LOG_* macros. The hot path
//               NEVER formats: the macro captures a per-call-site static LogMeta
//               (format string + level + module + file:line) and the enqueue path
//               serialises the raw arguments (PODs by memcpy, strings copied
//               inline, length-prefixed) into a fixed cell of the lock-free ring,
//               together with a compile-time "format thunk" that knows the exact
//               argument types. The actual std::format happens later, off the hot
//               path, in Drain() on the consumer side. Levels below
//               CDS_LOG_COMPILE_LEVEL compile away entirely (if constexpr), so the
//               same source builds with zero logging overhead when wanted.
//               Format strings use std::format's "{}" syntax; when the toolchain
//               lacks <format> a minimal "{}"-only fallback is used (no format
//               specs). Header-only; may live under libs/ (no app/core includes).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <mutex>
#include <string_view>
#include <tuple>
#include <type_traits>

#if defined(__cpp_lib_format) && __cpp_lib_format >= 201907L
#include <format>
#define CDS_LOG_HAS_STD_FORMAT 1
#else
#include <charconv>
#include <cstdio>
#define CDS_LOG_HAS_STD_FORMAT 0
#endif

#include "LogRing.hpp"

// --------------------------------------------------------------------------- //
// Compile-time configuration (override with -D before including)               //
// --------------------------------------------------------------------------- //
#ifndef CDS_LOG_MAX_MODULES
#define CDS_LOG_MAX_MODULES 32
#endif
#ifndef CDS_LOG_NAME_MAX
#define CDS_LOG_NAME_MAX 32
#endif
#ifndef CDS_LOG_MAX_SINKS
#define CDS_LOG_MAX_SINKS 8
#endif
#ifndef CDS_LOG_PAYLOAD_BYTES
#define CDS_LOG_PAYLOAD_BYTES 240
#endif
#ifndef CDS_LOG_LINE_MAX
#define CDS_LOG_LINE_MAX 256
#endif
#ifndef CDS_LOG_RING_CAPACITY
#define CDS_LOG_RING_CAPACITY 4096
#endif
// Levels strictly below this integer are stripped at compile time (see the
// CDS_LOG_LEVEL_* integer constants below). Default 0 = keep everything.
#ifndef CDS_LOG_COMPILE_LEVEL
#define CDS_LOG_COMPILE_LEVEL 0
#endif

#define CDS_LOG_LEVEL_TRACE 0
#define CDS_LOG_LEVEL_DEBUG 1
#define CDS_LOG_LEVEL_INFO  2
#define CDS_LOG_LEVEL_WARN  3
#define CDS_LOG_LEVEL_ERROR 4
#define CDS_LOG_LEVEL_OFF   5

namespace cds_log
{

// -----------------------------------------------------------------------------
// Usage
//
//   #include "log.hpp"          // sinks live in LogSinks.hpp / LogUiSink.hpp
//
//   // 1. Register a module once (a named channel); cache the id in a static:
//   static const auto net = cds_log::registry().module("net");
//
//   // 2. Log with std::format "{}" syntax (format specs work: {:.3f}, {:04d}):
//   CDS_LOG_INFO (net, "connected to {}:{}", host, port);
//   CDS_LOG_WARN (net, "retry {} of {}", attempt, maxAttempts);
//   CDS_LOG_ERROR(net, "giving up: {}", reason);
//   // Levels: TRACE < DEBUG < INFO < WARN < ERROR. The default runtime level is
//   // WARN, so INFO and below are dropped until raised per module (from the
//   // frontend). A dropped line is never formatted.
//
//   // 3. High-rate call sites: the _SAMPLED variants emit 1 message in N, where
//   //    N is the module's frontend-set divisor and each call site counts on
//   //    its own. WARN/ERROR have no sampled variant (never dropped):
//   CDS_LOG_DEBUG_SAMPLED(net, "rx seq={}", seq);
//
//   // The hot path only serialises the arguments into a lock-free ring -- it
//   // never formats and never blocks; the std::format runs later in drain() on
//   // the consumer side, where each line gets a wall-clock microsecond stamp.
//   // Raising CDS_LOG_COMPILE_LEVEL strips lower levels at compile time.
//
//   // Sinks and draining are wired by the app, not the call site:
//   //   registry().addSink(&sink);   // ConsoleSink / FileSink / uiSink()
//   //   registry().drain();          // drain thread (server) / tick end (wasm)
// -----------------------------------------------------------------------------

    typedef std::uint16_t moduleId_t;
    static constexpr moduleId_t INVALID_MODULE = static_cast<moduleId_t>(-1);

    enum class Level : std::uint8_t
    {
        Trace = CDS_LOG_LEVEL_TRACE,
        Debug = CDS_LOG_LEVEL_DEBUG,
        Info  = CDS_LOG_LEVEL_INFO,
        Warn  = CDS_LOG_LEVEL_WARN,
        Error = CDS_LOG_LEVEL_ERROR,
        Off   = CDS_LOG_LEVEL_OFF,
    };

    inline const char* levelName(Level l)
    {
        switch (l)
        {
            case Level::Trace: return "TRACE";
            case Level::Debug: return "DEBUG";
            case Level::Info:  return "INFO";
            case Level::Warn:  return "WARN";
            case Level::Error: return "ERROR";
            case Level::Off:   return "OFF";
        }
        return "?";
    }

    // Wall-clock timestamps: the log records carry system_clock nanoseconds
    // since the epoch (captured on the hot path), which this renders as local
    // time "YYYY-MM-DD HH:MM:SS.uuuuuu" with microsecond precision. Needs a
    // buffer of at least 27 bytes; truncates into a smaller one.
    inline void formatTimestamp(char* buf, std::size_t cap, std::uint64_t ns)
    {
        if (cap == 0) return;
        const std::time_t secs = static_cast<std::time_t>(ns / 1000000000ULL);
        const unsigned    us   = static_cast<unsigned>((ns % 1000000000ULL) / 1000);
        std::tm tmv{};
#if defined(_WIN32)
        localtime_s(&tmv, &secs);
#else
        localtime_r(&secs, &tmv);
#endif
        const std::size_t n = std::strftime(buf, cap, "%Y-%m-%d %H:%M:%S", &tmv);
        if (n > 0 && n < cap) std::snprintf(buf + n, cap - n, ".%06u", us);
        else buf[cap - 1] = '\0';
    }

    // ------------------------------------------------------------------------ //
    // Fixed-size format target: the thunk writes the final line here, on the    //
    // consumer side. Truncates silently at CDS_LOG_LINE_MAX.                    //
    // ------------------------------------------------------------------------ //
    class FmtBuffer
    {
        public:

        FmtBuffer() : m_len(0) {}

        void append(std::string_view s)
        {
            const std::size_t room = sizeof(m_data) - m_len;
            const std::size_t n = s.size() < room ? s.size() : room;
            std::memcpy(m_data + m_len, s.data(), n);
            m_len += n;
        }

        std::string_view view() const { return std::string_view(m_data, m_len); }
        void clear() { m_len = 0; }

        private:

        char        m_data[CDS_LOG_LINE_MAX];
        std::size_t m_len;
    };

    // ------------------------------------------------------------------------ //
    // Argument (de)serialisation. Each user argument is mapped to a "stored"    //
    // type: string-like arguments become std::string_view (copied inline);      //
    // everything else must be trivially copyable and is memcpy'd.               //
    // ------------------------------------------------------------------------ //
    template <typename T>
    struct isStringLike : std::false_type {};
    template <> struct isStringLike<const char*> : std::true_type {};
    template <> struct isStringLike<char*>       : std::true_type {};
    template <> struct isStringLike<std::string_view> : std::true_type {};

    // stored type for a (decayed) user argument type
    template <typename T>
    struct MapArg { using type = T; };
    template <> struct MapArg<const char*>      { using type = std::string_view; };
    template <> struct MapArg<char*>            { using type = std::string_view; };
    template <> struct MapArg<std::string_view> { using type = std::string_view; };

    // map a value to its stored representation
    template <typename T>
    inline auto mapValue(const T& v)
    {
        if constexpr (isStringLike<std::decay_t<T>>::value)
        {
            return std::string_view(v);
        }
        else
        {
            return v;
        }
    }

    // serialise one stored value; returns bytes written (clamped to cap)
    template <typename T>
    inline std::size_t storeArg(std::byte* dst, std::size_t cap, const T& v)
    {
        if constexpr (std::is_same_v<T, std::string_view>)
        {
            const std::size_t avail = cap >= 2 ? cap - 2 : 0;
            std::uint16_t n = static_cast<std::uint16_t>(v.size() < avail ? v.size() : avail);
            std::memcpy(dst, &n, sizeof(n));
            std::memcpy(dst + sizeof(n), v.data(), n);
            return sizeof(n) + n;
        }
        else
        {
            static_assert(std::is_trivially_copyable_v<T>,
                          "cds_log: only trivially-copyable or string arguments are supported");
            if (sizeof(T) > cap) return 0;
            std::memcpy(dst, &v, sizeof(T));
            return sizeof(T);
        }
    }

    // deserialise one stored value, advancing the read pointer
    template <typename T>
    inline T loadArg(const std::byte*& p)
    {
        if constexpr (std::is_same_v<T, std::string_view>)
        {
            std::uint16_t n = 0;
            std::memcpy(&n, p, sizeof(n));
            p += sizeof(n);
            std::string_view sv(reinterpret_cast<const char*>(p), n);
            p += n;
            return sv;
        }
        else
        {
            T v;
            std::memcpy(&v, p, sizeof(T));
            p += sizeof(T);
            return v;
        }
    }

    // ------------------------------------------------------------------------ //
    // Final formatting (consumer side).                                         //
    // ------------------------------------------------------------------------ //
#if CDS_LOG_HAS_STD_FORMAT
    template <typename... A>
    inline void formatInto(FmtBuffer& out, const char* fmt, A&... a)
    {
        try
        {
            out.append(std::vformat(fmt, std::make_format_args(a...)));
        }
        catch (...)
        {
            out.append(fmt); // malformed format string: emit it verbatim
        }
    }
#else
    inline void appendOne(FmtBuffer& out, std::string_view v) { out.append(v); }
    template <typename T>
    inline void appendOne(FmtBuffer& out, const T& v)
    {
        char tmp[48];
        if constexpr (std::is_floating_point_v<T>)
        {
            // to_chars(float) is missing on some older libc++ (emscripten): snprintf
            const int n = std::snprintf(tmp, sizeof(tmp), "%g", static_cast<double>(v));
            out.append(std::string_view(tmp, n > 0 ? static_cast<std::size_t>(n) : 0));
        }
        else
        {
            auto res = std::to_chars(tmp, tmp + sizeof(tmp), v);
            out.append(std::string_view(tmp, static_cast<std::size_t>(res.ptr - tmp)));
        }
    }
    inline void substitute(FmtBuffer& out, const char*& f) { /* no more args */ (void)out; (void)f; }
    template <typename T, typename... Rest>
    inline void substitute(FmtBuffer& out, const char*& f, const T& v, const Rest&... rest)
    {
        while (*f)
        {
            if (f[0] == '{' && f[1] == '}') { f += 2; appendOne(out, v); return substitute(out, f, rest...); }
            out.append(std::string_view(f, 1));
            ++f;
        }
    }
    template <typename... A>
    inline void formatInto(FmtBuffer& out, const char* fmt, A&... a)
    {
        const char* f = fmt;
        substitute(out, f, a...);
        out.append(std::string_view(f)); // tail after the last placeholder
    }
#endif

    // A "format thunk" is generated per call-site argument-type list: it knows
    // the exact stored types, reads them back from the payload (left-to-right,
    // guaranteed by brace-init order) and runs the final format.
    typedef void (*formatThunk_t)(const std::byte* payload, FmtBuffer& out, const char* fmt);

    template <typename... Stored>
    void formatThunk(const std::byte* payload, FmtBuffer& out, const char* fmt)
    {
        const std::byte* p = payload;
        std::tuple<Stored...> vals{ loadArg<Stored>(p)... }; // brace-init: left-to-right
        std::apply([&](auto&... a) { formatInto(out, fmt, a...); }, vals);
    }

    // ------------------------------------------------------------------------ //
    // Per-call-site metadata (lives in a function-local static, so its pointer  //
    // is stable forever and only 1x per call site is ever stored in a record).  //
    // ------------------------------------------------------------------------ //
    struct LogMeta
    {
        const char* fmt;
        Level       level;
        moduleId_t  module;
        const char* file;
        int         line;
    };

    struct LogRecord
    {
        std::uint64_t timestampNs;
        const LogMeta* meta;
        formatThunk_t  thunk;
        std::uint16_t  argBytes;
        std::byte      payload[CDS_LOG_PAYLOAD_BYTES];
    };

    // ------------------------------------------------------------------------ //
    // A formatted line handed to the sinks (consumer side).                     //
    // ------------------------------------------------------------------------ //
    struct LogLine
    {
        std::uint64_t    timestampNs;
        Level            level;
        moduleId_t       module;
        const char*      moduleName;
        std::string_view text;
        const char*      file;
        int              line;
    };

    class Sink
    {
        public:
        virtual ~Sink() = default;
        virtual void Write(const LogLine& line) = 0;
    };

    // ------------------------------------------------------------------------ //
    // The registry: module table (name + runtime level), the ring, the sink     //
    // list and the drain path. One process-wide instance via registry().        //
    // ------------------------------------------------------------------------ //
    class Registry
    {
        public:

        Registry() : m_count(0), m_sinkCount(0), m_drops(0) {}

        /* Register a module by name (idempotent) or find an existing one.
           Returns its id, or INVALID_MODULE if the table is full. The default
           runtime level is Warn (only Warn/Error emit until the frontend raises
           it). Called rarely (typically once per call site, cached in a static).
           Thread-safe. */
        moduleId_t module(const char* name)
        {
            std::lock_guard<std::mutex> lk(m_regMutex);
            const std::size_t n = m_count.load(std::memory_order_relaxed);
            for (std::size_t i = 0; i < n; ++i)
            {
                if (std::strncmp(m_modules[i].name, name, CDS_LOG_NAME_MAX) == 0)
                {
                    return static_cast<moduleId_t>(i);
                }
            }
            if (n >= CDS_LOG_MAX_MODULES) return INVALID_MODULE;
            std::strncpy(m_modules[n].name, name, CDS_LOG_NAME_MAX - 1);
            m_modules[n].name[CDS_LOG_NAME_MAX - 1] = '\0';
            m_modules[n].level.store(Level::Warn, std::memory_order_relaxed);
            m_modules[n].sampleN.store(1, std::memory_order_relaxed);
            m_count.store(n + 1, std::memory_order_release);
            return static_cast<moduleId_t>(n);
        }

        bool enabled(moduleId_t m, Level l) const
        {
            if (m == INVALID_MODULE || l == Level::Off) return false;
            // reject ids not yet registered: guards against a static-init-order
            // window where a module id (default 0) is used before its module()
            // has run, whose slot would otherwise read a value-initialised level
            if (m >= m_count.load(std::memory_order_acquire)) return false;
            return static_cast<std::uint8_t>(l) >=
                   static_cast<std::uint8_t>(m_modules[m].level.load(std::memory_order_relaxed));
        }

        void setLevel(moduleId_t m, Level l)
        {
            if (m < m_count.load(std::memory_order_acquire))
            {
                m_modules[m].level.store(l, std::memory_order_relaxed);
            }
        }

        Level level(moduleId_t m) const
        {
            if (m >= m_count.load(std::memory_order_acquire)) return Level::Off;
            return m_modules[m].level.load(std::memory_order_relaxed);
        }

        /* Sampling divisor for a module: the _SAMPLED macros emit 1 message every
           N per call site (N = 1 emits all). Set from the frontend. */
        void setSample(moduleId_t m, std::uint32_t n)
        {
            if (m < m_count.load(std::memory_order_acquire))
            {
                m_modules[m].sampleN.store(n ? n : 1, std::memory_order_relaxed);
            }
        }

        std::uint32_t sampleN(moduleId_t m) const
        {
            if (m >= m_count.load(std::memory_order_acquire)) return 1;
            return m_modules[m].sampleN.load(std::memory_order_relaxed);
        }

        const char* name(moduleId_t m) const
        {
            if (m >= m_count.load(std::memory_order_acquire)) return "?";
            return m_modules[m].name;
        }

        std::size_t count() const { return m_count.load(std::memory_order_acquire); }
        std::uint64_t drops() const { return m_drops.load(std::memory_order_relaxed); }

        /* Register a sink (not owned). Not thread-safe against concurrent Drain;
           install sinks at setup time. Returns true on error (table full). */
        bool addSink(Sink* s)
        {
            if (m_sinkCount >= CDS_LOG_MAX_SINKS) return true;
            m_sinks[m_sinkCount++] = s;
            return false;
        }

        void clearSinks() { m_sinkCount = 0; }

        /* Hot path: serialise the arguments into a ring cell. Never formats,
           never allocates, never blocks. On a full ring the record is dropped
           and counted. Any producer thread. */
        template <typename... A>
        void enqueue(const LogMeta& meta, const A&... args)
        {
            const bool dropped = m_ring.Produce([&](LogRecord& rec) {
                rec.timestampNs = nowNs();
                rec.meta  = &meta;
                rec.thunk = &formatThunk<typename MapArg<std::decay_t<A>>::type...>;
                std::byte* p = rec.payload;
                std::size_t off = 0;
                (( off += storeArg(p + off, sizeof(rec.payload) - off, mapValue(args)) ), ...);
                rec.argBytes = static_cast<std::uint16_t>(off);
            });
            if (dropped) m_drops.fetch_add(1, std::memory_order_relaxed);
        }

        /* Consumer side: format every pending record and hand each line to the
           sinks. Returns how many records were drained. Single consumer thread
           (the drain thread on the server, the main thread at a fixed point in
           the wasm build). */
        std::size_t drain()
        {
            return m_ring.Drain([&](const LogRecord& rec) {
                FmtBuffer buf;
                rec.thunk(rec.payload, buf, rec.meta->fmt);
                LogLine line{ rec.timestampNs, rec.meta->level, rec.meta->module,
                              name(rec.meta->module), buf.view(),
                              rec.meta->file, rec.meta->line };
                for (std::size_t i = 0; i < m_sinkCount; ++i)
                {
                    m_sinks[i]->Write(line);
                }
            });
        }

        private:

        static std::uint64_t nowNs()
        {
            // wall clock (not steady): the timestamp is for human-readable dates
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
        }

        struct Module
        {
            char                       name[CDS_LOG_NAME_MAX];
            std::atomic<Level>         level;
            std::atomic<std::uint32_t> sampleN; // _SAMPLED macros: emit 1 in N
        };

        Module                   m_modules[CDS_LOG_MAX_MODULES];
        std::atomic<std::size_t> m_count;
        std::mutex               m_regMutex;

        Sink*       m_sinks[CDS_LOG_MAX_SINKS];
        std::size_t m_sinkCount;

        LogRing<LogRecord, CDS_LOG_RING_CAPACITY> m_ring;
        std::atomic<std::uint64_t>                m_drops;
    };

    inline Registry& registry()
    {
        static Registry instance;
        return instance;
    }

} // namespace cds_log

// --------------------------------------------------------------------------- //
// Macros. CDS_LOG_AT is the workhorse; the per-level wrappers pin the level.    //
//   - the `if constexpr` strips levels below CDS_LOG_COMPILE_LEVEL at compile   //
//     time (zero code emitted);                                                 //
//   - the runtime `enabled()` check gates by the module's current level;        //
//   - the LogMeta is a per-call-site static (stable address, built once).       //
// --------------------------------------------------------------------------- //
#define CDS_LOG_AT(mod, lvl, lvlint, fmt, ...)                                       \
    do                                                                               \
    {                                                                                \
        if constexpr ((lvlint) >= CDS_LOG_COMPILE_LEVEL)                             \
        {                                                                            \
            if (::cds_log::registry().enabled((mod), (lvl)))                         \
            {                                                                        \
                static const ::cds_log::LogMeta _cds_log_meta{                       \
                    (fmt), (lvl), (mod), __FILE__, __LINE__ };                       \
                ::cds_log::registry().enqueue(_cds_log_meta __VA_OPT__(, ) __VA_ARGS__); \
            }                                                                        \
        }                                                                            \
    } while (0)

#define CDS_LOG_TRACE(mod, ...) \
    CDS_LOG_AT((mod), ::cds_log::Level::Trace, CDS_LOG_LEVEL_TRACE, __VA_ARGS__)
#define CDS_LOG_DEBUG(mod, ...) \
    CDS_LOG_AT((mod), ::cds_log::Level::Debug, CDS_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define CDS_LOG_INFO(mod, ...) \
    CDS_LOG_AT((mod), ::cds_log::Level::Info, CDS_LOG_LEVEL_INFO, __VA_ARGS__)
#define CDS_LOG_WARN(mod, ...) \
    CDS_LOG_AT((mod), ::cds_log::Level::Warn, CDS_LOG_LEVEL_WARN, __VA_ARGS__)
#define CDS_LOG_ERROR(mod, ...) \
    CDS_LOG_AT((mod), ::cds_log::Level::Error, CDS_LOG_LEVEL_ERROR, __VA_ARGS__)

// --------------------------------------------------------------------------- //
// Sampled variants: for high-rate call sites. Each call site keeps its OWN      //
// counter (so several sampled sites in one module never steal each other's      //
// turns) and emits one message every N, where N is the module's frontend-set    //
// sampling divisor (1 = emit all). Warn/Error have no sampled variant on         //
// purpose — you never want to drop them.                                        //
// --------------------------------------------------------------------------- //
#define CDS_LOG_SAMPLED_AT(mod, lvl, lvlint, fmt, ...)                              \
    do                                                                             \
    {                                                                              \
        if constexpr ((lvlint) >= CDS_LOG_COMPILE_LEVEL)                           \
        {                                                                          \
            if (::cds_log::registry().enabled((mod), (lvl)))                       \
            {                                                                      \
                static std::atomic<std::uint32_t> _cds_log_ctr{0};                 \
                const std::uint32_t _cds_log_n = ::cds_log::registry().sampleN((mod)); \
                if (_cds_log_ctr.fetch_add(1, std::memory_order_relaxed)           \
                        % (_cds_log_n ? _cds_log_n : 1) == 0)                      \
                {                                                                  \
                    static const ::cds_log::LogMeta _cds_log_meta{                 \
                        (fmt), (lvl), (mod), __FILE__, __LINE__ };                 \
                    ::cds_log::registry().enqueue(_cds_log_meta __VA_OPT__(, ) __VA_ARGS__); \
                }                                                                  \
            }                                                                      \
        }                                                                          \
    } while (0)

#define CDS_LOG_TRACE_SAMPLED(mod, ...) \
    CDS_LOG_SAMPLED_AT((mod), ::cds_log::Level::Trace, CDS_LOG_LEVEL_TRACE, __VA_ARGS__)
#define CDS_LOG_DEBUG_SAMPLED(mod, ...) \
    CDS_LOG_SAMPLED_AT((mod), ::cds_log::Level::Debug, CDS_LOG_LEVEL_DEBUG, __VA_ARGS__)
#define CDS_LOG_INFO_SAMPLED(mod, ...) \
    CDS_LOG_SAMPLED_AT((mod), ::cds_log::Level::Info, CDS_LOG_LEVEL_INFO, __VA_ARGS__)

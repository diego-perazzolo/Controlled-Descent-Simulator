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
// File        : LogSinks.hpp
// Description : Ready-to-use log sinks, both formatting one line per record as
//               "[LEVEL] module: message". ConsoleSink writes to a non-owned C
//               stream (stderr by default). FileSink owns a file it opens (append
//               mode) and closes on destruction, flushing every line so a crash
//               keeps the log. Sinks are consumer-side only: they run inside
//               Registry::drain(), never on the tick hot path, so ordinary
//               blocking I/O here is fine. A real filesystem exists only in the
//               native (server) build; the wasm build uses ConsoleSink only.
//               Header-only; may live under libs/ (no app/core includes).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <atomic>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>

#include "log.hpp"
#include "UniqueFile.hpp"

namespace cds_log
{

    // formats a line into "YYYY-MM-DD HH:MM:SS.uuuuuu [LEVEL] module: message\n"
    inline void writeLine(std::FILE* stream, const LogLine& line)
    {
        char ts[28];
        formatTimestamp(ts, sizeof(ts), line.timestampNs);
        std::fprintf(stream, "%s [%-5s] %s: %.*s\n",
                     ts, levelName(line.level), line.moduleName,
                     static_cast<int>(line.text.size()), line.text.data());
    }

    // Writes each line to a C stream (stderr by default). Not owned: the caller
    // keeps the stream alive for the sink's lifetime.
    class ConsoleSink : public Sink
    {
        public:

        explicit ConsoleSink(std::FILE* stream = stderr) : m_stream(stream) {}

        void Write(const LogLine& line) override { writeLine(m_stream, line); }

        private:

        std::FILE* m_stream;
    };

    // A togglable file sink. The path set via setPath() is a BASE name; each time
    // the sink is enabled it lazily opens a fresh, uniquely-named file derived
    // from it (uniqueFilePath: a timestamp stamped before the extension), so a
    // new run never overwrites a previous one and toggling recording off then on
    // reopens a new file rather than appending. Flushes each line so a crash
    // keeps the log. setEnabled() and setPath() are safe to call from another
    // thread than Write() (which runs in the drain). Disabled and pathless by
    // default. Native builds only (the wasm build has no real filesystem).
    // Reachable process-wide via fileSink() so an ext command can toggle the same
    // instance the app registered.
    class FileSink : public Sink
    {
        public:

        FileSink() : m_file(nullptr), m_enabled(false) { m_base[0] = '\0'; }
        ~FileSink() override { if (m_file) std::fclose(m_file); }

        FileSink(const FileSink&) = delete;
        FileSink& operator=(const FileSink&) = delete;

        void setPath(const char* path)
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            std::strncpy(m_base, path ? path : "", sizeof(m_base) - 1);
            m_base[sizeof(m_base) - 1] = '\0';
            if (m_file) { std::fclose(m_file); m_file = nullptr; } // reopen under the new base
        }

        void setEnabled(bool on)
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            m_enabled.store(on, std::memory_order_relaxed);
            // On disable, close the file so a later re-enable opens a FRESH,
            // uniquely-named one — every logging session is its own file.
            if (!on && m_file) { std::fclose(m_file); m_file = nullptr; }
        }
        bool enabled() const { return m_enabled.load(std::memory_order_relaxed); }

        void Write(const LogLine& line) override
        {
            if (!m_enabled.load(std::memory_order_relaxed)) return;
            std::lock_guard<std::mutex> lk(m_mutex);
            if (!m_file)
            {
                if (m_base[0] == '\0') return;
                m_file = std::fopen(uniqueFilePath(m_base).c_str(), "w");
                if (!m_file) return;
            }
            writeLine(m_file, line);
            std::fflush(m_file);
        }

        private:

        char              m_base[256];
        std::FILE*        m_file;
        std::atomic<bool> m_enabled;
        std::mutex        m_mutex;
    };

    inline FileSink& fileSink()
    {
        static FileSink instance;
        return instance;
    }

} // namespace cds_log

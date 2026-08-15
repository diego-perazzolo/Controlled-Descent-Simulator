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

#include <cstdio>

#include "log.hpp"

namespace cds_log
{

    // formats a line into "[LEVEL] module: message\n" on a C stream
    inline void writeLine(std::FILE* stream, const LogLine& line)
    {
        std::fprintf(stream, "[%-5s] %s: %.*s\n",
                     levelName(line.level), line.moduleName,
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

    // Owns a file opened in append mode; flushes each line so a crash keeps the
    // log. isOpen() reports whether the path could be opened. Native builds only.
    class FileSink : public Sink
    {
        public:

        explicit FileSink(const char* path) : m_file(std::fopen(path, "a")) {}
        ~FileSink() override { if (m_file) std::fclose(m_file); }

        FileSink(const FileSink&) = delete;
        FileSink& operator=(const FileSink&) = delete;

        bool isOpen() const { return m_file != nullptr; }

        void Write(const LogLine& line) override
        {
            if (!m_file) return;
            writeLine(m_file, line);
            std::fflush(m_file);
        }

        private:

        std::FILE* m_file;
    };

} // namespace cds_log

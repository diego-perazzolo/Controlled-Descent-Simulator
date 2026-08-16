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
// File        : LogUiSink.hpp
// Description : A log sink that keeps the most recent formatted lines in a small
//               bounded buffer for the frontend to poll through the ext API.
//               Unlike the main log ring (single consumer, drained to the
//               console/file sinks), this is a second, independent buffer: the
//               drain thread writes into it via Write(), and the ext command
//               thread pops from it via Pop(). Both sides take a mutex -- but off
//               the tick hot path (Write runs in drain(), Pop in an ext command),
//               so this never touches real-time code. When full the oldest line
//               is overwritten and counted (TakeDropped), so a slow frontend
//               never stalls logging. One process-wide instance via uiSink().
//               Header-only; may live under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <mutex>

#include "log.hpp"

#ifndef CDS_LOG_UI_CAPACITY
#define CDS_LOG_UI_CAPACITY 256
#endif

namespace cds_log
{

    // Bounded ring of the most recent formatted lines, mutex-guarded. Producer:
    // the drain thread (Write). Consumer: the ext command thread (Pop).
    class RecentLinesSink : public Sink
    {
        public:

        struct Line
        {
            std::uint64_t timestampNs;
            Level         level;
            char          module[CDS_LOG_NAME_MAX];
            char          text[CDS_LOG_LINE_MAX];
        };

        RecentLinesSink() : m_head(0), m_tail(0), m_dropped(0) {}

        /* Sink interface: called from the drain thread for every formatted line. */
        void Write(const LogLine& line) override
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            Line& slot = m_buf[m_head % CDS_LOG_UI_CAPACITY];
            slot.timestampNs = line.timestampNs;
            slot.level = line.level;
            copyStr(slot.module, sizeof(slot.module), line.moduleName,
                    line.moduleName ? std::strlen(line.moduleName) : 0);
            copyStr(slot.text, sizeof(slot.text), line.text.data(), line.text.size());
            ++m_head;
            if (m_head - m_tail > CDS_LOG_UI_CAPACITY)
            {
                m_tail = m_head - CDS_LOG_UI_CAPACITY; // overwrite oldest
                ++m_dropped;
            }
        }

        /* Pop the oldest pending line. Returns false when empty. Consumer side. */
        bool Pop(Line& out)
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            if (m_tail == m_head) return false;
            out = m_buf[m_tail % CDS_LOG_UI_CAPACITY];
            ++m_tail;
            return true;
        }

        /* Number of lines dropped since the last call, then reset. Consumer side. */
        std::uint64_t TakeDropped()
        {
            std::lock_guard<std::mutex> lk(m_mutex);
            const std::uint64_t d = m_dropped;
            m_dropped = 0;
            return d;
        }

        private:

        static void copyStr(char* dst, std::size_t cap, const char* src, std::size_t n)
        {
            const std::size_t k = n < cap - 1 ? n : cap - 1;
            if (src && k) std::memcpy(dst, src, k);
            dst[k] = '\0';
        }

        Line          m_buf[CDS_LOG_UI_CAPACITY];
        std::size_t   m_head; // total written
        std::size_t   m_tail; // total popped
        std::uint64_t m_dropped;
        std::mutex    m_mutex;
    };

    inline RecentLinesSink& uiSink()
    {
        static RecentLinesSink instance;
        return instance;
    }

} // namespace cds_log

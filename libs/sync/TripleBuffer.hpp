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
// File        : TripleBuffer.hpp
// Description : Wait-free single-writer / single-reader latest-wins mailbox.
//               Three slots: at any instant one belongs to the writer, one to
//               the reader, one holds the newest published sample; ownership
//               rotates via atomic index swaps, so neither side ever waits
//               and a torn read is impossible.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <atomic>
#include <cstdint>

namespace cds_sync
{

    // Latest-wins mailbox. Contract: exactly ONE writer thread and ONE reader
    // thread; neither method blocks. Unread samples are overwritten (that is
    // the point: the reader always gets the most recent one).
    template <typename T>
    class TripleBuffer
    {
        public:

        TripleBuffer() : m_slots(),
                         m_writeIdx(0),
                         m_readIdx(1),
                         m_everRead(false),
                         m_latest(2)
        {
        }

        /* Publish a sample, overwriting an unconsumed one. Wait-free.
           Writer thread only */
        void Write(const T& value)
        {
            m_slots[m_writeIdx] = value;

            /* swap the freshly written slot with the "latest" slot */
            const uint32_t prev = m_latest.exchange(m_writeIdx | FRESH_FLAG,
                                                    std::memory_order_acq_rel);
            m_writeIdx = prev & INDEX_MASK;
        }

        /* Get the most recent published sample. Non-consuming: with no new
           publication, re-reading returns the same sample. Wait-free.
           Reader thread only. Returns true on error (nothing published yet) */
        bool Read(T& out)
        {
            if (m_latest.load(std::memory_order_acquire) & FRESH_FLAG)
            {
                /* claim the newest slot, hand our old one back to the rotation */
                const uint32_t prev = m_latest.exchange(m_readIdx,
                                                        std::memory_order_acq_rel);
                m_readIdx = prev & INDEX_MASK;
                m_everRead = true;
            }

            if (!m_everRead)
            {
                // Nothing published yet, error
                return true;
            }

            out = m_slots[m_readIdx];
            return false;
        }

        private:

        static constexpr uint32_t FRESH_FLAG = 0x4;
        static constexpr uint32_t INDEX_MASK = 0x3;

        T m_slots[3];
        uint32_t m_writeIdx;               // writer-private
        uint32_t m_readIdx;                // reader-private
        bool m_everRead;                   // reader-private
        std::atomic<uint32_t> m_latest;    // newest slot index (+ fresh flag)
    };

} // namespace cds_sync

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
// File        : LogRing.hpp
// Description : Bounded lock-free MPSC ring of fixed-size cells (Vyukov bounded
//               queue). Many producer threads (the real-time tick thread and the
//               command threads) push records; a single consumer (the drain
//               thread on the server, or the same thread at a fixed drain-point
//               in the single-threaded wasm build) pops them. Producers never
//               block and never allocate: a full ring makes Produce() fail so the
//               caller drops-and-counts rather than stall the tick (golden rule
//               9). The payload is filled/consumed in place through a callable,
//               so the ring never copies the (large) record. Capacity is a
//               compile-time power of two. Header-only; depends only on <atomic>,
//               <cstddef>, <cstdint>, so it may live under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace cds_log
{

    // Bounded MPSC ring of T. Contract: any number of producer threads calling
    // Produce(), exactly ONE consumer thread calling Consume()/Drain(); neither
    // blocks. Capacity must be a power of two. A full ring makes Produce() return
    // false (the record is dropped), an empty ring makes Consume() return false.
    template <typename T, std::size_t Capacity>
    class LogRing
    {
        static_assert((Capacity & (Capacity - 1)) == 0,
                      "LogRing capacity must be a power of two");

        public:

        LogRing() : m_cells(), m_enqueue(0), m_dequeue(0)
        {
            for (std::size_t i = 0; i < Capacity; ++i)
            {
                m_cells[i].seq.store(i, std::memory_order_relaxed);
            }
        }

        /* Claim a cell and fill it in place via fill(T&). Returns false without
           calling fill when the ring is full (caller drops-and-counts). Producer
           threads, wait-free under no contention, lock-free under contention. */
        template <typename Fill>
        bool Produce(Fill&& fill)
        {
            Cell* cell = nullptr;
            std::size_t pos = m_enqueue.load(std::memory_order_relaxed);
            for (;;)
            {
                cell = &m_cells[pos & (Capacity - 1)];
                const std::size_t seq = cell->seq.load(std::memory_order_acquire);
                const std::intptr_t diff =
                    static_cast<std::intptr_t>(seq) - static_cast<std::intptr_t>(pos);
                if (diff == 0)
                {
                    if (m_enqueue.compare_exchange_weak(pos, pos + 1,
                                                        std::memory_order_relaxed))
                    {
                        break;
                    }
                }
                else if (diff < 0)
                {
                    return true; // full: drop
                }
                else
                {
                    pos = m_enqueue.load(std::memory_order_relaxed);
                }
            }

            fill(cell->data);
            cell->seq.store(pos + 1, std::memory_order_release);
            return false;
        }

        /* Consume one cell in place via consume(const T&). Returns true (error)
           without calling consume when the ring is empty. Single consumer thread. */
        template <typename Consume>
        bool ConsumeOne(Consume&& consume)
        {
            Cell* cell = nullptr;
            std::size_t pos = m_dequeue.load(std::memory_order_relaxed);
            for (;;)
            {
                cell = &m_cells[pos & (Capacity - 1)];
                const std::size_t seq = cell->seq.load(std::memory_order_acquire);
                const std::intptr_t diff =
                    static_cast<std::intptr_t>(seq) - static_cast<std::intptr_t>(pos + 1);
                if (diff == 0)
                {
                    if (m_dequeue.compare_exchange_weak(pos, pos + 1,
                                                        std::memory_order_relaxed))
                    {
                        break;
                    }
                }
                else if (diff < 0)
                {
                    return true; // empty
                }
                else
                {
                    pos = m_dequeue.load(std::memory_order_relaxed);
                }
            }

            consume(cell->data);
            cell->seq.store(pos + Capacity, std::memory_order_release);
            return false;
        }

        /* Consume every currently-available cell, calling consume(const T&) on
           each. Returns how many were drained. Single consumer thread. */
        template <typename Consume>
        std::size_t Drain(Consume&& consume)
        {
            std::size_t n = 0;
            while (!ConsumeOne(consume))
            {
                ++n;
            }
            return n;
        }

        static constexpr std::size_t capacity() { return Capacity; }

        private:

        struct Cell
        {
            std::atomic<std::size_t> seq;
            T                        data;
        };

        Cell                     m_cells[Capacity];
        alignas(64) std::atomic<std::size_t> m_enqueue; // next producer ticket
        alignas(64) std::atomic<std::size_t> m_dequeue; // next consumer ticket
    };

} // namespace cds_log

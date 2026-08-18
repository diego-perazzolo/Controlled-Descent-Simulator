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
// File        : controller_params.hpp
// Description : A small, self-describing registry of a controller's exposed
//               parameters, shared by the runtime models (LQR, MPC) to present a
//               uniform tuning interface to the frontend. Each entry carries a
//               group and a label (its meaning), a read/write flag, and get/set
//               accessors bound to the owning model. The registry builds the wire
//               *manifest* -- a compact TSV text buffer, one line per parameter --
//               and dispatches a set-by-id. It is used only interactively (open a
//               panel, read all, write one coefficient at a time), never on the
//               simulation tick, so it favours clarity over zero-allocation; it
//               is fixed-capacity and keeps no state on the tick path. Domain- and
//               protocol-agnostic: depends only on the standard library, so it
//               lives under libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <cstddef>
#include <cstdio>
#include <functional>

namespace CDS { namespace control {

// Registry of exposed controller parameters. CAP bounds the number of entries
// (parameters are few -- e.g. ~20 for LQR weights, ~8 for the MPC knobs).
template <std::size_t CAP = 40>
class ParamTable
{
public:
    using Getter = std::function<double()>;
    using Setter = std::function<bool(double)>;   // returns true on error / rejected value

    ParamTable() : m_count(0) {}

    void clear() { m_count = 0; }
    std::size_t size() const { return m_count; }

    // Register a parameter. `group` and `label` are stable strings (the model
    // owns them). `set` may be empty for a read-only parameter (writable=false).
    void add(const char* group, const char* label, bool writable, Getter get, Setter set = {})
    {
        if (m_count >= CAP) return;
        Entry& e = m_entries[m_count++];
        e.group = group; e.label = label; e.writable = writable;
        e.get = std::move(get); e.set = std::move(set);
    }

    // Emit the manifest as TSV, one line per parameter:
    //   id \t group \t label \t flags \t value \n      (flags = "rw" | "ro")
    // The whole buffer is always NUL-terminated; it truncates cleanly if `n` is
    // too small (a line is never half-written past the end).
    void buildManifest(char* buf, std::size_t n) const
    {
        if (buf == nullptr || n == 0) return;
        std::size_t off = 0;
        buf[0] = '\0';
        for (std::size_t i = 0; i < m_count; ++i)
        {
            const Entry& e = m_entries[i];
            const int w = std::snprintf(buf + off, n - off, "%zu\t%s\t%s\t%s\t%.9g\n",
                                        i, e.group, e.label, e.writable ? "rw" : "ro",
                                        e.get ? e.get() : 0.0);
            if (w < 0) break;
            if (static_cast<std::size_t>(w) >= n - off) { buf[n - 1] = '\0'; return; }
            off += static_cast<std::size_t>(w);
        }
    }

    // Set parameter `id` to `value`. Returns true on error -- bad id, read-only
    // parameter, or the setter rejected the value (bool-is-error convention).
    bool set(int id, double value) const
    {
        if (id < 0 || static_cast<std::size_t>(id) >= m_count) return true;
        const Entry& e = m_entries[static_cast<std::size_t>(id)];
        if (!e.writable || !e.set) return true;
        return e.set(value);
    }

private:
    struct Entry
    {
        const char* group;
        const char* label;
        bool        writable;
        Getter      get;
        Setter      set;
    };
    Entry       m_entries[CAP];
    std::size_t m_count;
};

}} // namespace CDS::control

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
// File        : P2Quantile.hpp
// Description : Single-quantile streaming estimator (the P-square algorithm of
//               Jain & Chlamtac, 1985). Estimates a fixed quantile p of a stream
//               in O(1) time and O(1) memory per sample — five markers, no stored
//               history — so it fits the profiler hot path (one Add() per sample,
//               no allocation, no lock; one writer per instance). The estimate is
//               marker-based and therefore approximate, which is exactly the
//               trade for not keeping every sample; before five samples it falls
//               back to the exact quantile of the buffered values. Header-only;
//               depends only on <algorithm> and <cstddef>, so it may live under
//               libs/.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once

#include <algorithm>
#include <cstddef>

namespace cds_profile
{

    // Estimates the p-quantile (0 < p < 1) of a stream. Reset() before reuse.
    class P2Quantile
    {
        public:

        P2Quantile() : m_p(0.5), m_count(0) {}

        void init(double p)
        {
            m_p = p;
            m_count = 0;
        }

        void reset() { m_count = 0; }

        /* Fold one sample into the estimate. Single writer. */
        void Add(double x)
        {
            if (m_count < 5)
            {
                m_q[m_count++] = x;
                if (m_count == 5)
                {
                    std::sort(m_q, m_q + 5);
                    for (int i = 0; i < 5; ++i) m_n[i] = i + 1;
                    m_np[0] = 1.0;
                    m_np[1] = 1.0 + 2.0 * m_p;
                    m_np[2] = 1.0 + 4.0 * m_p;
                    m_np[3] = 3.0 + 2.0 * m_p;
                    m_np[4] = 5.0;
                    m_dn[0] = 0.0;
                    m_dn[1] = m_p / 2.0;
                    m_dn[2] = m_p;
                    m_dn[3] = (1.0 + m_p) / 2.0;
                    m_dn[4] = 1.0;
                }
                return;
            }

            // locate the cell k (0..3) the sample falls into, extending the
            // extreme markers when the sample lies outside the current range
            int k;
            if (x < m_q[0]) { m_q[0] = x; k = 0; }
            else if (x >= m_q[4]) { m_q[4] = x; k = 3; }
            else
            {
                k = 3;
                for (int i = 0; i < 4; ++i)
                {
                    if (x < m_q[i + 1]) { k = i; break; }
                }
            }

            for (int i = k + 1; i < 5; ++i) m_n[i] += 1;
            for (int i = 0; i < 5; ++i) m_np[i] += m_dn[i];

            // adjust the three interior markers toward their desired positions
            for (int i = 1; i <= 3; ++i)
            {
                const double d = m_np[i] - m_n[i];
                if ((d >= 1.0 && m_n[i + 1] - m_n[i] > 1) ||
                    (d <= -1.0 && m_n[i - 1] - m_n[i] < -1))
                {
                    const int s = d >= 0.0 ? 1 : -1;
                    const double qp = parabolic(i, s);
                    if (m_q[i - 1] < qp && qp < m_q[i + 1]) m_q[i] = qp;
                    else m_q[i] = linear(i, s);
                    m_n[i] += s;
                }
            }
            ++m_count;
        }

        /* Current estimate. Exact quantile of the buffer before five samples,
           the P-square marker estimate afterwards. */
        double estimate() const
        {
            if (m_count == 0) return 0.0;
            if (m_count < 5)
            {
                double buf[5];
                for (std::size_t i = 0; i < m_count; ++i) buf[i] = m_q[i];
                std::sort(buf, buf + m_count);
                const double idx = m_p * static_cast<double>(m_count - 1);
                const std::size_t lo = static_cast<std::size_t>(idx);
                const double frac = idx - static_cast<double>(lo);
                if (lo + 1 >= m_count) return buf[m_count - 1];
                return buf[lo] + frac * (buf[lo + 1] - buf[lo]);
            }
            return m_q[2];
        }

        private:

        double parabolic(int i, int s) const
        {
            const double a = static_cast<double>(m_n[i + 1] - m_n[i]);
            const double b = static_cast<double>(m_n[i] - m_n[i - 1]);
            return m_q[i] + s / static_cast<double>(m_n[i + 1] - m_n[i - 1]) *
                   ((b + s) * (m_q[i + 1] - m_q[i]) / a +
                    (a - s) * (m_q[i] - m_q[i - 1]) / b);
        }

        double linear(int i, int s) const
        {
            return m_q[i] + s * (m_q[i + s] - m_q[i]) /
                   static_cast<double>(m_n[i + s] - m_n[i]);
        }

        double      m_p;        // target quantile
        double      m_q[5];     // marker heights (also the first-5 buffer)
        int         m_n[5];     // marker positions
        double      m_np[5];    // desired marker positions
        double      m_dn[5];    // desired-position increments
        std::size_t m_count;    // samples seen
    };

} // namespace cds_profile

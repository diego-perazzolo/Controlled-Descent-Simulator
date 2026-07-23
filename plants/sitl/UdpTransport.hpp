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
// File        : UdpTransport.hpp
// Description : Minimal POSIX UDP endpoint (bind + sendto/recvfrom) for the
//               plant communication threads. This class is the transport
//               seam of the SITL plant: the serial link for real hardware
//               will be its drop-in sibling. Plants-internal — never
//               included by core or apps code.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <cstddef>
#include <cstdint>
#include <string>

#include <netinet/in.h>

namespace plants
{

    // Minimal UDP endpoint: one socket, bounded blocking receive
    class UdpTransport
    {
        public:

        UdpTransport();
        ~UdpTransport();

        /* owns the file descriptor: no copies */
        UdpTransport(const UdpTransport&) = delete;
        UdpTransport& operator=(const UdpTransport&) = delete;

        /* Bind on bindHost:port; recvTimeout_seconds (must be > 0) bounds
           every blocking Recv. Returns true on error */
        bool Open(const std::string& bindHost, uint16_t port,
                  double recvTimeout_seconds);

        /* Close the socket. Idempotent */
        void Close(void);

        bool IsOpen(void) const;

        /* Blocking receive, bounded by the Open timeout. Returns the number
           of received bytes and fills peer; 0 on timeout, -1 on error */
        int Recv(uint8_t* buffer, size_t capacity, sockaddr_in& peer);

        /* Non-blocking receive: same contract, 0 when nothing is queued */
        int TryRecv(uint8_t* buffer, size_t capacity, sockaddr_in& peer);

        /* Send one datagram to peer. Returns true on error */
        bool Send(const uint8_t* buffer, size_t length,
                  const sockaddr_in& peer);

        private:

        int m_fd;
    };

} // namespace plants

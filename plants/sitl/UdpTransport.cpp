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
// File        : UdpTransport.cpp
// Description : Minimal POSIX UDP endpoint (bind + sendto/recvfrom) for the
//               plant communication threads
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "UdpTransport.hpp"

#include <cerrno>
#include <cmath>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

using namespace plants;

UdpTransport::UdpTransport() : m_fd(-1)
{

}

UdpTransport::~UdpTransport()
{
    Close();
}

bool UdpTransport::Open(const std::string& bindHost, uint16_t port,
                        double recvTimeout_seconds)
{
    if (m_fd >= 0)
    {
        // Already open, error
        return true;
    }

    if (bindHost.empty() || port == 0 || recvTimeout_seconds <= 0)
    {
        // Invalid parameters, error
        return true;
    }

    m_fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (m_fd < 0)
    {
        // Socket creation failed, error
        return true;
    }

    /* bound blocking receive: this timeout paces the communication loop */
    struct timeval timeout = {};
    timeout.tv_sec = static_cast<time_t>(recvTimeout_seconds);
    timeout.tv_usec = static_cast<suseconds_t>(
        (recvTimeout_seconds - std::floor(recvTimeout_seconds)) * 1e6);
    if (::setsockopt(m_fd, SOL_SOCKET, SO_RCVTIMEO,
                     &timeout, sizeof(timeout)) != 0)
    {
        Close();
        return true;
    }

    sockaddr_in local = {};
    local.sin_family = AF_INET;
    local.sin_port = htons(port);
    if (::inet_pton(AF_INET, bindHost.c_str(), &local.sin_addr) != 1)
    {
        // Not a valid IPv4 literal, error
        Close();
        return true;
    }

    if (::bind(m_fd, reinterpret_cast<const sockaddr*>(&local),
               sizeof(local)) != 0)
    {
        Close();
        return true;
    }

    return false;
}

void UdpTransport::Close(void)
{
    if (m_fd >= 0)
    {
        ::close(m_fd);
        m_fd = -1;
    }
}

bool UdpTransport::IsOpen(void) const
{
    return m_fd >= 0;
}

int UdpTransport::Recv(uint8_t* buffer, size_t capacity, sockaddr_in& peer)
{
    if (m_fd < 0 || !buffer)
    {
        return -1;
    }

    socklen_t peerLength = sizeof(peer);
    const ssize_t received =
        ::recvfrom(m_fd, buffer, capacity, 0,
                   reinterpret_cast<sockaddr*>(&peer), &peerLength);

    if (received < 0)
    {
        /* timeout is not an error: it is the loop pacing */
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }

    return static_cast<int>(received);
}

int UdpTransport::TryRecv(uint8_t* buffer, size_t capacity, sockaddr_in& peer)
{
    if (m_fd < 0 || !buffer)
    {
        return -1;
    }

    socklen_t peerLength = sizeof(peer);
    const ssize_t received =
        ::recvfrom(m_fd, buffer, capacity, MSG_DONTWAIT,
                   reinterpret_cast<sockaddr*>(&peer), &peerLength);

    if (received < 0)
    {
        return (errno == EAGAIN || errno == EWOULDBLOCK) ? 0 : -1;
    }

    return static_cast<int>(received);
}

bool UdpTransport::Send(const uint8_t* buffer, size_t length,
                        const sockaddr_in& peer)
{
    if (m_fd < 0 || !buffer)
    {
        // Not open / invalid buffer, error
        return true;
    }

    const ssize_t sent =
        ::sendto(m_fd, buffer, length, 0,
                 reinterpret_cast<const sockaddr*>(&peer), sizeof(peer));

    return sent != static_cast<ssize_t>(length);
}

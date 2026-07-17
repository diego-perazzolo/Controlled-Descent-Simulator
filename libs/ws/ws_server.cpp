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
// File        : ws_server.cpp
// Description : Minimal dependency-free WebSocket server (RFC 6455).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include "ws_server.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>
#include <cstring>
#include <string>

/* --------------------------- SHA-1 (RFC 3174) --------------------------- */
/* Needed only for the Sec-WebSocket-Accept handshake digest */

static void _sha1(const uint8_t* data, size_t len, uint8_t out[20])
{
    uint32_t h[5] = {0x67452301, 0xEFCDAB89, 0x98BADCFE, 0x10325476, 0xC3D2E1F0};

    uint64_t bitLen = (uint64_t)len * 8;
    size_t paddedLen = ((len + 8) / 64 + 1) * 64;
    std::vector<uint8_t> msg(paddedLen, 0);
    memcpy(msg.data(), data, len);
    msg[len] = 0x80;
    for(int i = 0; i < 8; i++)
    {
        msg[paddedLen - 1 - i] = (uint8_t)(bitLen >> (8 * i));
    }

    for(size_t chunk = 0; chunk < paddedLen; chunk += 64)
    {
        uint32_t w[80];
        for(int i = 0; i < 16; i++)
        {
            w[i] = ((uint32_t)msg[chunk + 4 * i] << 24) |
                   ((uint32_t)msg[chunk + 4 * i + 1] << 16) |
                   ((uint32_t)msg[chunk + 4 * i + 2] << 8) |
                   ((uint32_t)msg[chunk + 4 * i + 3]);
        }
        for(int i = 16; i < 80; i++)
        {
            uint32_t v = w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16];
            w[i] = (v << 1) | (v >> 31);
        }

        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4];
        for(int i = 0; i < 80; i++)
        {
            uint32_t f, k;
            if(i < 20)      { f = (b & c) | ((~b) & d);         k = 0x5A827999; }
            else if(i < 40) { f = b ^ c ^ d;                    k = 0x6ED9EBA1; }
            else if(i < 60) { f = (b & c) | (b & d) | (c & d);  k = 0x8F1BBCDC; }
            else            { f = b ^ c ^ d;                    k = 0xCA62C1D6; }

            uint32_t tmp = ((a << 5) | (a >> 27)) + f + e + k + w[i];
            e = d;
            d = c;
            c = (b << 30) | (b >> 2);
            b = a;
            a = tmp;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e;
    }

    for(int i = 0; i < 5; i++)
    {
        out[4 * i]     = (uint8_t)(h[i] >> 24);
        out[4 * i + 1] = (uint8_t)(h[i] >> 16);
        out[4 * i + 2] = (uint8_t)(h[i] >> 8);
        out[4 * i + 3] = (uint8_t)(h[i]);
    }
}

static std::string _base64(const uint8_t* data, size_t len)
{
    static const char tbl[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for(size_t i = 0; i < len; i += 3)
    {
        uint32_t v = (uint32_t)data[i] << 16;
        if(i + 1 < len) v |= (uint32_t)data[i + 1] << 8;
        if(i + 2 < len) v |= (uint32_t)data[i + 2];

        out += tbl[(v >> 18) & 0x3F];
        out += tbl[(v >> 12) & 0x3F];
        out += (i + 1 < len) ? tbl[(v >> 6) & 0x3F] : '=';
        out += (i + 2 < len) ? tbl[v & 0x3F] : '=';
    }
    return out;
}

/* ------------------------------ socket I/O ------------------------------ */

/* Read exactly n bytes, returns true on error / connection closed */
static bool _readFull(int fd, uint8_t* buf, size_t n)
{
    size_t got = 0;
    while(got < n)
    {
        ssize_t r = recv(fd, buf + got, n - got, 0);
        if(r <= 0)
        {
            // Err: peer closed or socket failure
            return true;
        }
        got += (size_t)r;
    }
    return false;
}

/* Write exactly n bytes, returns true on error */
static bool _writeFull(int fd, const uint8_t* buf, size_t n)
{
    size_t sent = 0;
    while(sent < n)
    {
        ssize_t w = send(fd, buf + sent, n - sent, 0);
        if(w <= 0)
        {
            // Err
            return true;
        }
        sent += (size_t)w;
    }
    return false;
}

/* Send one unmasked server->client frame, returns true on error */
static bool _sendFrame(int fd, uint8_t opcode, const uint8_t* payload, size_t len)
{
    uint8_t hdr[10];
    size_t hdrLen = 2;

    hdr[0] = 0x80 | opcode; // FIN + opcode
    if(len < 126)
    {
        hdr[1] = (uint8_t)len;
    }
    else if(len <= 0xFFFF)
    {
        hdr[1] = 126;
        hdr[2] = (uint8_t)(len >> 8);
        hdr[3] = (uint8_t)len;
        hdrLen = 4;
    }
    else
    {
        hdr[1] = 127;
        for(int i = 0; i < 8; i++)
        {
            hdr[2 + i] = (uint8_t)(len >> (8 * (7 - i)));
        }
        hdrLen = 10;
    }

    if(_writeFull(fd, hdr, hdrLen))
    {
        return true;
    }
    return _writeFull(fd, payload, len);
}

/* Read one client->server frame; fills opcode/payload, returns true on error */
static bool _readFrame(int fd, uint8_t& opcode, bool& fin, std::vector<uint8_t>& payload)
{
    uint8_t hdr[2];
    if(_readFull(fd, hdr, 2))
    {
        return true;
    }

    fin = (hdr[0] & 0x80) != 0;
    opcode = hdr[0] & 0x0F;
    bool masked = (hdr[1] & 0x80) != 0;
    uint64_t len = hdr[1] & 0x7F;

    if(len == 126)
    {
        uint8_t ext[2];
        if(_readFull(fd, ext, 2)) return true;
        len = ((uint64_t)ext[0] << 8) | ext[1];
    }
    else if(len == 127)
    {
        uint8_t ext[8];
        if(_readFull(fd, ext, 8)) return true;
        len = 0;
        for(int i = 0; i < 8; i++)
        {
            len = (len << 8) | ext[i];
        }
    }

    /* RFC 6455: client frames must be masked; also cap the size defensively */
    if(!masked || len > (1u << 20))
    {
        // Err: protocol violation
        return true;
    }

    uint8_t mask[4];
    if(_readFull(fd, mask, 4))
    {
        return true;
    }

    payload.resize((size_t)len);
    if(len > 0 && _readFull(fd, payload.data(), (size_t)len))
    {
        return true;
    }

    for(size_t i = 0; i < payload.size(); i++)
    {
        payload[i] ^= mask[i % 4];
    }

    return false;
}

/* ------------------------------- WsServer ------------------------------- */

WsServer::WsServer(uint16_t port, Handler handler)
    : m_port(port), m_handler(std::move(handler))
{
}

bool WsServer::Handshake(int fd)
{
    /* Read the HTTP upgrade request (up to the blank line) */
    std::string request;
    char c;
    while(request.find("\r\n\r\n") == std::string::npos)
    {
        if(request.size() > 8192 || _readFull(fd, (uint8_t*)&c, 1))
        {
            // Err
            return true;
        }
        request += c;
    }

    /* Extract Sec-WebSocket-Key (headers are case-insensitive) */
    std::string lower = request;
    for(auto& ch : lower)
    {
        ch = (char)tolower((unsigned char)ch);
    }
    const std::string keyHdr = "sec-websocket-key:";
    size_t pos = lower.find(keyHdr);
    if(pos == std::string::npos)
    {
        // Err: not a WebSocket upgrade request
        return true;
    }
    pos += keyHdr.size();
    size_t end = request.find("\r\n", pos);
    std::string key = request.substr(pos, end - pos);
    key.erase(0, key.find_first_not_of(" \t"));
    key.erase(key.find_last_not_of(" \t") + 1);

    /* Accept token: base64(SHA1(key + RFC 6455 GUID)) */
    std::string accept = key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    uint8_t digest[20];
    _sha1((const uint8_t*)accept.data(), accept.size(), digest);

    std::string response =
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: " + _base64(digest, 20) + "\r\n"
        "\r\n";

    return _writeFull(fd, (const uint8_t*)response.data(), response.size());
}

bool WsServer::ServeClient(int fd)
{
    if(Handshake(fd))
    {
        // Err
        return true;
    }

    printf("[cds-server] client connected\n");

    std::vector<uint8_t> message; // reassembly buffer for fragmented messages

    for(;;)
    {
        uint8_t opcode;
        bool fin;
        std::vector<uint8_t> payload;

        if(_readFrame(fd, opcode, fin, payload))
        {
            // Err / connection closed
            return true;
        }

        switch(opcode)
        {
            case 0x0: // continuation
            case 0x1: // text (tolerated, handled as binary)
            case 0x2: // binary
            {
                message.insert(message.end(), payload.begin(), payload.end());
                if(!fin)
                {
                    break;
                }

                /* Transport frame: the first 4 bytes are the correlation id,
                   owned by this transport. The handler only sees the payload;
                   the id is echoed back verbatim on the response. */
                if(message.size() < 4)
                {
                    // Err: malformed frame, no way to correlate an answer
                    return true;
                }

                std::vector<uint8_t> request(message.begin() + 4, message.end());
                std::vector<uint8_t> response = m_handler(request);

                if(!response.empty())
                {
                    response.insert(response.begin(), message.begin(), message.begin() + 4);
                    if(_sendFrame(fd, 0x2, response.data(), response.size()))
                    {
                        // Err
                        return true;
                    }
                }
                message.clear();
                break;
            }

            case 0x8: // close
                _sendFrame(fd, 0x8, payload.data(), payload.size() > 2 ? 2 : payload.size());
                return false;

            case 0x9: // ping -> pong
                if(_sendFrame(fd, 0xA, payload.data(), payload.size()))
                {
                    return true;
                }
                break;

            case 0xA: // pong: ignore
                break;

            default:
                // Err: reserved opcode
                return true;
        }
    }
}

bool WsServer::Run()
{
    int listenFd = socket(AF_INET, SOCK_STREAM, 0);
    if(listenFd < 0)
    {
        // Err
        perror("socket");
        return true;
    }

    int yes = 1;
    setsockopt(listenFd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(m_port);

    if(bind(listenFd, (sockaddr*)&addr, sizeof(addr)) < 0)
    {
        // Err
        perror("bind");
        close(listenFd);
        return true;
    }

    if(listen(listenFd, 1) < 0)
    {
        // Err
        perror("listen");
        close(listenFd);
        return true;
    }

    printf("[cds-server] listening on ws://0.0.0.0:%u\n", m_port);

    for(;;)
    {
        int clientFd = accept(listenFd, nullptr, nullptr);
        if(clientFd < 0)
        {
            // Err
            perror("accept");
            close(listenFd);
            return true;
        }

        /* Low-latency: the protocol is small request/response frames */
        setsockopt(clientFd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        ServeClient(clientFd);
        close(clientFd);
        printf("[cds-server] client disconnected\n");
    }
}

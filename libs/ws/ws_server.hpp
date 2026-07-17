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
// File        : ws_server.hpp
// Description : Minimal dependency-free WebSocket RPC server (RFC 6455).
//               Serves one client at a time; each binary message is passed to
//               a handler whose return value is sent back as a binary frame.
//               Request/response correlation is owned by the transport: every
//               wire message starts with a 4-byte id, stripped before the
//               handler and echoed back on the response. The browser-side
//               counterpart is ws_rpc_client. Payloads are protocol-agnostic
//               opaque bytes.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include <cstdint>
#include <functional>
#include <vector>

class WsServer
{
public:
    /* Handler: request payload in, response payload out */
    using Handler = std::function<std::vector<uint8_t>(const std::vector<uint8_t>&)>;

    WsServer(uint16_t port, Handler handler);

    /* Blocking accept/serve loop, returns true on error */
    bool Run();

private:
    /* Serve one accepted connection until it closes, returns true on error */
    bool ServeClient(int fd);

    /* Perform the HTTP -> WebSocket upgrade handshake, returns true on error */
    bool Handshake(int fd);

    uint16_t m_port;
    Handler m_handler;
};

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
// File        : main.cpp
// Description : cds_server entry point: wires the libs/ws WebSocket server to
//               the ws_protocol dispatcher (dispatch.cpp). The WASM proxy
//               built from apps/ws-served/client is the client.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <cstdio>
#include <cstdlib>

#include "dispatch.hpp"
#include "ws_protocol.hpp"
#include "ws_server.hpp"

int main(int argc, char** argv)
{
    /* line-buffered logs even when stdout is redirected to a file */
    setvbuf(stdout, nullptr, _IOLBF, 0);

    uint16_t port = ws_proto::WS_DEFAULT_PORT;
    if(argc > 1)
    {
        port = (uint16_t)atoi(argv[1]);
    }

    WsServer server(port, server_dispatch);
    if(server.Run())
    {
        // Err
        return 1;
    }

    return 0;
}

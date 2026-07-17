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
// File        : ws_rpc_client.hpp
// Description : Synchronous request/response RPC over WebSocket for WASM
//               (browser side). Protocol-agnostic: moves opaque byte buffers.
//               Request/response correlation is owned by this transport — a
//               4-byte id is prepended on the wire and stripped before the
//               payload reaches either endpoint (see WsServer for the native
//               counterpart).
//
//               The embind API is synchronous while browser WebSockets are
//               asynchronous, so the socket lives in a dedicated Web Worker
//               and the calling (main) thread spin-waits on a
//               SharedArrayBuffer. Consequences:
//                 - the page must be cross-origin isolated (COOP/COEP);
//                 - the worker is spawned by ws_rpc_client_pre.js BEFORE the
//                   WASM runtime is ready (a worker never starts while the
//                   main thread is blocked) — link the pre-js, or every call
//                   fails. Both are wired up by the libs/ws CMake target.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include <cstdint>

/* Round-trips one request over the WebSocket bridge worker and blocks until
   the matching response arrives. Returns the response payload length in
   bytes, or -1 on failure (no worker, socket down, timeout, buffer too
   small). Payloads are opaque to the transport. */
extern "C" int ws_rpc(const uint8_t* req, int reqLen, uint8_t* resp, int respCap);

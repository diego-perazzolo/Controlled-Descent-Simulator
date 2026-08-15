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
// File        : ws_rpc_client.cpp
// Description : Synchronous WebSocket RPC transport for WASM — see the header
//               for the contract. Wire format (owned by the transport):
//               [u32 correlation id, little endian][payload]. The bridge
//               worker (ws_rpc_client_pre.js) strips the id from responses
//               and publishes it in the shared buffer for matching.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#include <cstdint>
#include <emscripten/em_js.h>
#include "ws_rpc_client.hpp"

// clang-format off
/* Shared-buffer layout (see also ws_rpc_client_pre.js):
   i32[0] flag: 0 idle, 1 request pending, 2 response ready, -1 transport dead
   i32[1] byte length of the data area content
   i32[2] correlation id of the received response
   byte 16... data area (request: [id|payload] out, response: payload in) */
EM_JS(int, ws_rpc, (const uint8_t* req, int reqLen, uint8_t* resp, int respCap), {
    var c = globalThis.__cdsWs;
    if (!c) return -1; /* no SharedArrayBuffer: pre-js already logged why */
    if (Atomics.load(c.i32, 0) === -1) return -1;

    var id = (c.nextId = ((c.nextId | 0) + 1) | 0);

    /* transport frame: 4-byte correlation id + opaque payload */
    c.u8[16] = id & 255;
    c.u8[17] = (id >> 8) & 255;
    c.u8[18] = (id >> 16) & 255;
    c.u8[19] = (id >> 24) & 255;
    c.u8.set(HEAPU8.subarray(req, req + reqLen), 20);
    c.i32[1] = reqLen + 4;
    Atomics.store(c.i32, 0, 1);
    c.w.postMessage(0);

    /* Spin-wait: the socket lives in the worker, so the response arrives even
       though this (main) thread is blocked. Localhost RTT keeps this short.
       The timeout bounds how long a stalled server can freeze the main thread
       (Atomics.wait is forbidden here, so this is a busy-wait): keep it well
       above the worst legitimate RPC latency but low enough that the UI
       recovers quickly — the frontend auto-stops when a call runs this long. */
    var RPC_TIMEOUT_MS = 1000;
    var t0 = Date.now();
    for (;;) {
        var f = Atomics.load(c.i32, 0);
        if (f === 2) {
            if (c.i32[2] === id) break;
            Atomics.store(c.i32, 0, 0); /* stale response: drop, keep waiting */
        } else if (f === -1) {
            return -1;
        }
        if (Date.now() - t0 > RPC_TIMEOUT_MS) {
            Atomics.store(c.i32, 0, 0);
            console.error('[cds-ws] RPC timeout (id=' + id + ')');
            return -1;
        }
    }

    var len = c.i32[1];
    Atomics.store(c.i32, 0, 0);
    if (len > respCap) return -1;
    HEAPU8.set(c.u8.subarray(16, 16 + len), resp);
    return len;
});
// clang-format on

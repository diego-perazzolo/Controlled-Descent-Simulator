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
// File        : ws_protocol.hpp
// Description : Binary wire protocol shared between the WASM WebSocket proxy
//               (client/ext_comm_ws.cpp) and the native core server
//               (server/main.cpp + dispatch.cpp).
//               One request -> one response. Packed little-endian structs;
//               both peers are little-endian (WASM and x86/ARM).
//               Request/response correlation is NOT part of this protocol:
//               it is owned by the libs/ws transport, which frames every
//               message with its own id before it reaches the wire.
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================
#pragma once
#include <cstdint>
#include "ext_defs.hpp"

namespace ws_proto {

constexpr uint16_t WS_DEFAULT_PORT = 9002;

/* Message types: request and matching response carry the same type id */
enum MsgType : uint8_t
{
    WS_MSG_INIT_ROCKET       = 1, // -> respBool_t
    WS_MSG_INIT_QUADROTOR    = 2, // -> respBool_t
    WS_MSG_STEP              = 3, // -> respStep_t
    WS_MSG_TRAJ_GET_POINT    = 4, // -> respPoint_t
    WS_MSG_TRAJ_APPEND_POLY4 = 5, // -> respBool_t
    WS_MSG_TRAJ_APPEND_POINT = 6, // -> respBool_t
    WS_MSG_TRAJ_REMOVE_LAST  = 7, // -> respBool_t
};

#pragma pack(push, 1)

/* Common message header: every request/response starts with this */
typedef struct
{
    uint8_t type; // MsgType, echoed in the response
} header_t;

/* ------------------------------- requests ------------------------------- */

typedef struct
{
    header_t h;
    ext_rocketParams params;
    ext_rocketActuatorLimits actuatorLimits;
} reqInitRocket_t;

typedef struct
{
    header_t h;
    ext_quadRotorParams params;
    ext_quadRotorActuatorLimits actuatorLimits;
} reqInitQuadRotor_t;

typedef struct
{
    header_t h;
    ext_coord_t timeStep_s;
    ext_userForce userForce;
} reqStep_t;

typedef struct
{
    header_t h;
    ext_coord_t t;
} reqTrajGetPoint_t;

typedef struct
{
    header_t h;
    ext_trajectoryPoly4Params_t params;
} reqTrajAppendPoly4_t;

typedef struct
{
    header_t h;
    ext_trajectoryPointParams_t params;
} reqTrajAppendPoint_t;

typedef struct
{
    header_t h;
} reqTrajRemoveLast_t;

/* ------------------------------- responses ------------------------------ */

/* generic boolean response: isError follows the core convention (1 = error) */
typedef struct
{
    header_t h;
    uint8_t isError;
} respBool_t;

typedef struct
{
    header_t h;
    uint8_t isError;
    ext_fullState state;
    ext_setpointError err;
} respStep_t;

typedef struct
{
    header_t h;
    ext_trajectoryPoint point;
} respPoint_t;

#pragma pack(pop)

/* Largest message either peer can send: used to size the shared RPC buffer */
constexpr uint32_t WS_MAX_MSG_SIZE = 256;

static_assert(sizeof(reqTrajAppendPoly4_t) < WS_MAX_MSG_SIZE, "wire msg too big");
static_assert(sizeof(respStep_t) < WS_MAX_MSG_SIZE, "wire msg too big");

/* Exact wire sizes, hand-computed (f = sizeof(ext_coord_t) = 4): each peer
   checks them against its own ABI at compile time, so any layout drift
   (padding from mixed-size fields, architecture-dependent type widths)
   breaks the build instead of silently corrupting the parsing.
   On an intentional protocol change, recompute and update the number here;
   never "fix" a failing assert by tweaking the number to make one platform
   compile — that turns a build error into corrupted data on the wire. */
static_assert(sizeof(header_t)              ==  1, "wire layout drift");
static_assert(sizeof(reqInitRocket_t)       == 57, "wire layout drift"); // 1 + 6f + 8f
static_assert(sizeof(reqInitQuadRotor_t)    == 49, "wire layout drift"); // 1 + 10f + 2f
static_assert(sizeof(reqStep_t)             == 17, "wire layout drift"); // 1 + 1f + 3f
static_assert(sizeof(reqTrajGetPoint_t)     ==  5, "wire layout drift"); // 1 + 1f
static_assert(sizeof(reqTrajAppendPoly4_t)  == 85, "wire layout drift"); // 1 + 21f
static_assert(sizeof(reqTrajAppendPoint_t)  == 21, "wire layout drift"); // 1 + 5f
static_assert(sizeof(reqTrajRemoveLast_t)   ==  1, "wire layout drift");
static_assert(sizeof(respBool_t)            ==  2, "wire layout drift"); // 1 + u8
static_assert(sizeof(respStep_t)            == 66, "wire layout drift"); // 1 + u8 + 12f + 4f
static_assert(sizeof(respPoint_t)           == 13, "wire layout drift"); // 1 + 3f

} // namespace ws_proto

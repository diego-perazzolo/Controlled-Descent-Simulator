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
// File        : mavlink_pin.hpp
// Description : Sole entry point to the vendored MAVLink headers, guarded by
//               compatibility pins. Every SITL source includes MAVLink through
//               this file, never mavlink/ directly. The static_asserts freeze
//               the wire contract of the messages the plant uses (id, payload
//               length, CRC_EXTRA seed) and the command/enum values sent as
//               plain numbers: re-vendoring headers where any of these drifted
//               breaks the build instead of silently changing the protocol.
//               Peer-side compatibility is covered at runtime: the CRC_EXTRA
//               pinned here is seeded into every packet checksum, so a peer
//               built from diverged message definitions fails CRC and its
//               packets are dropped by the parser (the link watchdog then
//               surfaces the resulting silence as an explicit link error).
// Author      : Diego Perazzolo
// Created     : 2026
// =============================================================================

#pragma once
#include <string_view>

#include "mavlink/common/mavlink.h"

/* Wire protocol generation: this plant speaks MAVLink 2 only */
static_assert(std::string_view(MAVLINK_WIRE_PROTOCOL_VERSION) == "2.0",
              "MAVLink pin: wire protocol is no longer 2.0");
static_assert(MAVLINK_VERSION == 3,
              "MAVLink pin: HEARTBEAT mavlink_version field value changed");

/* Message wire contracts: id, payload length, CRC_EXTRA seed.
   MAVLink 2 zero-truncates trailing zero payload bytes, so LEN is the
   maximum on the wire; CRC_EXTRA changes whenever a message definition
   changes incompatibly. */
static_assert(MAVLINK_MSG_ID_HEARTBEAT == 0 &&
              MAVLINK_MSG_ID_HEARTBEAT_LEN == 9 &&
              MAVLINK_MSG_ID_HEARTBEAT_CRC == 50,
              "MAVLink pin: HEARTBEAT contract drifted");

static_assert(MAVLINK_MSG_ID_SYSTEM_TIME == 2 &&
              MAVLINK_MSG_ID_SYSTEM_TIME_LEN == 12 &&
              MAVLINK_MSG_ID_SYSTEM_TIME_CRC == 137,
              "MAVLink pin: SYSTEM_TIME contract drifted");

static_assert(MAVLINK_MSG_ID_ATTITUDE == 30 &&
              MAVLINK_MSG_ID_ATTITUDE_LEN == 28 &&
              MAVLINK_MSG_ID_ATTITUDE_CRC == 39,
              "MAVLink pin: ATTITUDE contract drifted");

static_assert(MAVLINK_MSG_ID_LOCAL_POSITION_NED == 32 &&
              MAVLINK_MSG_ID_LOCAL_POSITION_NED_LEN == 28 &&
              MAVLINK_MSG_ID_LOCAL_POSITION_NED_CRC == 185,
              "MAVLink pin: LOCAL_POSITION_NED contract drifted");

static_assert(MAVLINK_MSG_ID_COMMAND_LONG == 76 &&
              MAVLINK_MSG_ID_COMMAND_LONG_LEN == 33 &&
              MAVLINK_MSG_ID_COMMAND_LONG_CRC == 152,
              "MAVLink pin: COMMAND_LONG contract drifted");

static_assert(MAVLINK_MSG_ID_COMMAND_ACK == 77 &&
              MAVLINK_MSG_ID_COMMAND_ACK_LEN == 10 &&
              MAVLINK_MSG_ID_COMMAND_ACK_CRC == 143,
              "MAVLink pin: COMMAND_ACK contract drifted");

static_assert(MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED == 84 &&
              MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_LEN == 53 &&
              MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED_CRC == 143,
              "MAVLink pin: SET_POSITION_TARGET_LOCAL_NED contract drifted");

/* Enum values that travel on the wire as plain numbers */
static_assert(MAV_CMD_NAV_TAKEOFF == 22 &&
              MAV_CMD_COMPONENT_ARM_DISARM == 400 &&
              MAV_CMD_SET_MESSAGE_INTERVAL == 511,
              "MAVLink pin: MAV_CMD command number drifted");

static_assert(MAV_FRAME_LOCAL_NED == 1,
              "MAVLink pin: MAV_FRAME_LOCAL_NED value drifted");

static_assert(MAV_TYPE_GCS == 6 &&
              MAV_AUTOPILOT_INVALID == 8 &&
              MAV_STATE_ACTIVE == 4 &&
              MAV_COMP_ID_AUTOPILOT1 == 1 &&
              MAV_COMP_ID_MISSIONPLANNER == 190,
              "MAVLink pin: heartbeat/component enum value drifted");

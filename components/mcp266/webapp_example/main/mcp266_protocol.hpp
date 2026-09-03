#pragma once

// Wire protocol for the USB <-> MCP266 web console example.
//
// Unlike the CAN bridge (which forwards raw CAN frames and runs CANopen in the
// browser), this example runs the espp::Mcp266 driver ON the device and exposes
// a small, HIGH-LEVEL command protocol: the browser sends "configure axis",
// "move to position", "get status" and the firmware translates each to Mcp266
// calls over CANopen. So the web app needs no CANopen/DS402 knowledge.
//
// Framed with the espp stream_frame v2 codec and routed by an espp::Dispatcher
// on MODULE ID 6. Every frame sets module = 6. The `type` byte's high nibble is
// 6 for host->device requests and E for device->host replies/events; the reply
// types (0xE_) additionally set the frame reply flag (build_frame derives it
// from the type's high bit). Both the vendor (WebUSB) and CDC (Web Serial)
// interfaces carry this same protocol.
//
// Axis selector byte: 0 = M1, 1 = M2.

#include <cstdint>

namespace mcp266_protocol {

/// Dispatcher module id owned by the MCP266 console protocol.
static constexpr uint8_t kModuleId = 6;

/// Axis selector used in request payloads (matches espp::Mcp266::Axis order).
enum : uint8_t {
  kAxisM1 = 0,
  kAxisM2 = 1,
};

/// Host -> device (requests, high nibble 6).
enum : uint8_t {
  kStart = 0x60,                 ///< NMT-start the node + clear latched faults (no payload)
  kResetFaults = 0x61,           ///< clear latched CiA 402 faults on both axes (no payload)
  kResetEstop = 0x62,            ///< attempt an e-stop reset (no payload)
  kConfigurePositionLoop = 0x63, ///< [axis u8][min i32][max i32][fallback_p i32]
  kSetPositionLimits = 0x64,     ///< CiA 402 software limits: [axis u8][min i32][max i32]
  kMoveToPosition = 0x65,        ///< [axis u8][target i32][vel u32][accel u32][decel u32]
  kDriveSpeed = 0x66,            ///< [axis u8][qpps i32] (inert on tested firmware)
  kDriveDuty = 0x67,             ///< [axis u8][duty i16] (inert on tested firmware)
  kGetStatus = 0x68,             ///< request one STATUS snapshot (no payload)
  kSetStatusStream = 0x69,       ///< [enable u8][period_ms u16] periodic STATUS streaming
  kGetDeviceInfo = 0x6A,         ///< request DEVICE_INFO (no payload)
};

/// Device -> host (replies / events, high nibble E => reply flag set).
enum : uint8_t {
  kStatus = 0xE0,     ///< status snapshot (see StatusPayload layout below)
  kOk = 0xE1,         ///< ack for a request: [request_type u8]
  kError = 0xE2,      ///< failure: [request_type u8][code u32][utf8 message]
  kDeviceInfo = 0xE3, ///< [device_type u32][utf8 name]
};

/// STATUS payload layout (all multi-byte fields little-endian), 25 bytes:
///   per axis M1 then M2:
///     [position i32][velocity i32][statusword u16]   (10 bytes each)
///   then device-level:
///     [battery_decivolts u16]  (tenths of a volt)
///     [temp_decidegrees u16]   (tenths of a degree C)
///     [flags u8]               (bit0 = node responded to the last poll)
static constexpr uint8_t kStatusFlagOnline = 0x01;
static constexpr uint8_t kAxisStatusSize = 10;                                 ///< i32 + i32 + u16
static constexpr uint8_t kStatusPayloadSize = 2 * kAxisStatusSize + 2 + 2 + 1; // = 25

} // namespace mcp266_protocol

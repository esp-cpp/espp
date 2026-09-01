#pragma once

// Wire protocol for the USB <-> CAN (TWAI) bridge example.
//
// Framed with the espp stream_frame v2 codec and routed by espp::Dispatcher on
// MODULE ID 5. The v2 frame has a DEDICATED module byte, so EVERY frame here —
// requests and replies alike — sets module = 5 and routes to dispatcher module
// 5 (this is NOT the retired v1 scheme where the module was derived from the
// type's high nibble). The 0x5X / 0xD_ values below are the `type` byte, not
// the module; the reply/event types (0xD_) additionally set the frame reply
// flag (build derives it from the type's high bit). The hosted CAN console web
// app speaks this exact protocol over WebUSB / Web Serial.
//
// A CAN frame is encoded as a compact payload:
//   [id u32 LE][flags u8][dlc u8][data: dlc bytes]
//   flags: bit0 = extended (29-bit id), bit1 = remote-transmission-request (RTR)
// so a frame payload is 6..14 bytes (dlc 0..8).

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "stream_frame.hpp"

namespace can_bridge {

/// Dispatcher module id owned by the CAN bridge protocol.
static constexpr uint8_t kModuleId = 5;

/// Host -> device (requests, high nibble 5).
enum : uint8_t {
  kCanTx = 0x50,     ///< transmit a CAN frame (payload: encoded CanFrame)
  kSetConfig = 0x51, ///< set bus config (payload: u32 baudrate, u8 mode, u8 reserved)
  kStart = 0x52,     ///< bring the bus up with the current config (no payload)
  kStop = 0x53,      ///< take the bus down (no payload)
  kGetStatus = 0x54, ///< request the current config + counters (no payload)
};

/// Device -> host (replies / events, high nibble D).
enum : uint8_t {
  kCanRx = 0xD0,  ///< a received CAN frame (payload: encoded CanFrame)
  kOk = 0xD1,     ///< ack for a request (payload: empty)
  kError = 0xD2,  ///< failure (payload: u32 code + UTF-8 message)
  kStatus = 0xD3, ///< status reply (see StatusPayload below)
};

/// Bus mode requested by SET_CONFIG.
enum : uint8_t {
  kModeNormal = 0,     ///< actively participate (ACK frames) — "master" / send+receive
  kModeListenOnly = 1, ///< passive monitor (never ACK/transmit) — "inspection"
};

/// CAN-frame flag bits used in the encoded payload.
enum : uint8_t {
  kFlagExtended = 0x01, ///< 29-bit extended identifier
  kFlagRtr = 0x02,      ///< remote-transmission-request frame (no data)
};

/// A decoded classic-CAN (2.0) frame (mirrors espp::Twai::Message fields).
struct CanFrame {
  uint32_t id{0};
  bool extended{false};
  bool rtr{false};
  uint8_t dlc{0};
  std::array<uint8_t, 8> data{};
};

/// Encode a CanFrame to its wire payload: [id u32][flags u8][dlc u8][data].
inline std::vector<uint8_t> encode_frame(const CanFrame &f) {
  std::vector<uint8_t> p;
  const uint8_t dlc = f.dlc > 8 ? 8 : f.dlc;
  p.reserve(6 + dlc);
  espp::stream_frame::put_u32(p, f.id);
  uint8_t flags = 0;
  if (f.extended)
    flags |= kFlagExtended;
  if (f.rtr)
    flags |= kFlagRtr;
  p.push_back(flags);
  p.push_back(dlc);
  // RTR frames carry no data; otherwise append dlc data bytes.
  if (!f.rtr)
    p.insert(p.end(), f.data.begin(), f.data.begin() + dlc);
  return p;
}

/// Decode a CAN-frame wire payload. Returns false if malformed (too short, dlc
/// out of range, or fewer data bytes than dlc for a non-RTR frame).
inline bool decode_frame(std::span<const uint8_t> payload, CanFrame &out) {
  if (payload.size() < 6)
    return false;
  out.id = espp::stream_frame::get_u32(payload);
  const uint8_t flags = payload[4];
  const uint8_t dlc = payload[5];
  out.extended = (flags & kFlagExtended) != 0;
  out.rtr = (flags & kFlagRtr) != 0;
  if (dlc > 8)
    return false;
  out.dlc = dlc;
  out.data = {};
  if (!out.rtr) {
    if (payload.size() < static_cast<size_t>(6) + dlc)
      return false;
    for (uint8_t i = 0; i < dlc; ++i)
      out.data[i] = payload[6 + i];
  }
  return true;
}

// Status reply payload layout (kStatus):
//   [baudrate u32][mode u8][running u8][rx_count u32][tx_count u32][err_count u32]

} // namespace can_bridge

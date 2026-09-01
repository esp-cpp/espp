#pragma once

// espp::Dispatcher — multiplex several independent framed protocols over a
// single byte stream, routing each frame to a handler by its "module id".
//
// A USB vendor / CDC / socket link often needs to carry more than one protocol
// at once: firmware update (OTA), crash-dump inspection, a CAN bridge, an
// application's own control channel, ... They all ride the same
// espp::stream_frame framing (magic / type / len / crc). Rather than run a
// separate StreamParser per protocol over the same bytes (each re-buffering the
// whole stream and needing its own reset-on-overflow bookkeeping), a Dispatcher
// parses the stream ONCE and routes each complete frame to the handler
// registered for its module id.
//
// Module id convention
// --------------------
// The module id is the high nibble of the message-type byte: `type >> 4`.
// espp built-in protocols place their request opcodes so each protocol occupies
// one high nibble, and use bit 7 to mark device->host replies:
//
//   module 0  OTA       requests 0x0X   replies 0x8X
//   module 4  crash dump requests 0x4X  replies 0xCX
//   module 5  CAN bridge requests 0x5X  replies 0xDX   (example)
//
// A DEVICE-side dispatcher registers the request modules (0, 4, 5, ...); the
// reply-typed frames (high nibble 8..15) it never receives, and if one does
// arrive it lands on an unregistered module and is ignored — so requests and
// replies of the same protocol can never be confused. A HOST-side dispatcher
// (if used) would instead register the reply high nibbles. Application code is
// free to assign any unused module id to its own protocol; nothing here is
// hard-wired to a specific service.
//
// Header-only and dependency-free (only espp::stream_frame + the standard
// library), so it builds and unit-tests on a host.

#include <array>
#include <cstdint>
#include <functional>
#include <span>

#include "stream_frame.hpp"

namespace espp {

/// @brief Routes framed messages from one byte stream to per-module handlers.
class Dispatcher {
public:
  /// Number of routable modules (one per value of the type byte's high nibble).
  static constexpr uint8_t kNumModules = 16;

  /// @brief Handler invoked for every frame whose module id was registered.
  /// @param type The full message-type byte (module id in the high nibble).
  /// @param payload The frame payload bytes (valid only for the call).
  using handler_fn = std::function<void(uint8_t type, std::span<const uint8_t> payload)>;

  /// @brief The module id of a message-type byte (its high nibble).
  static constexpr uint8_t module_of(uint8_t type) { return static_cast<uint8_t>(type >> 4); }

  /// @brief Register (or replace) the handler for a module id (0..kNumModules-1).
  /// @param module_id High-nibble module id to route to @p handler.
  /// @param handler Callback for frames with this module id (null unregisters).
  void register_module(uint8_t module_id, handler_fn handler) {
    if (module_id < kNumModules)
      handlers_[module_id] = std::move(handler);
  }

  /// @brief Remove the handler for a module id (frames for it become ignored).
  void unregister_module(uint8_t module_id) {
    if (module_id < kNumModules)
      handlers_[module_id] = nullptr;
  }

  /// @brief Whether a handler is registered for a module id.
  bool has_module(uint8_t module_id) const {
    return module_id < kNumModules && static_cast<bool>(handlers_[module_id]);
  }

  /// @brief Feed raw received bytes: parse and route each complete frame to its
  ///        module's handler. Frames whose module has no handler are ignored
  ///        (so unrelated protocols on the same stream are harmless).
  /// @param data Any number of received bytes (frames may be split or batched).
  void feed(std::span<const uint8_t> data) {
    for (const auto &frame : parser_.feed(data))
      dispatch(frame);
  }

  /// @brief Route an already-parsed frame (for callers running their own parser).
  void dispatch(const stream_frame::Frame &frame) {
    const uint8_t module_id = module_of(frame.type);
    if (module_id < kNumModules && handlers_[module_id])
      handlers_[module_id](frame.type, frame.payload);
  }

  /// @brief Discard any partially-buffered frame bytes (transport reconnect or
  ///        RX overflow) so a frame straddling the gap resynchronizes at once.
  void reset() { parser_.reset(); }

  /// @brief Bytes buffered awaiting frame completion.
  size_t buffered() const { return parser_.buffered(); }

  /// @brief Total bytes discarded while resynchronizing (diagnostics).
  size_t dropped_bytes() const { return parser_.dropped_bytes(); }

private:
  stream_frame::StreamParser parser_;
  std::array<handler_fn, kNumModules> handlers_{};
};

} // namespace espp

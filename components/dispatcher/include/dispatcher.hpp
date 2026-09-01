#pragma once

// espp::Dispatcher — multiplex several independent framed protocols over a
// single byte stream, routing each frame to a handler by its `module` id.
//
// A USB vendor / CDC / socket / UART link often needs to carry more than one
// protocol at once: firmware update (OTA), crash-dump inspection, a CAN bridge,
// an application's own control channel, ... They all ride the same
// espp::stream_frame framing. Rather than run a separate StreamParser per
// protocol over the same bytes (each re-buffering the whole stream and needing
// its own reset-on-overflow bookkeeping), a Dispatcher parses the stream ONCE
// and routes each complete frame to the handler registered for its module id.
//
// The frame's `module` byte (0..255) is the routing key — a full byte, so up to
// 256 protocols can coexist. The message/transaction type and the
// request/reply direction travel in the frame's `type` and `flags` fields and
// are handed to the module's handler untouched; the Dispatcher does not
// interpret them. A device-side dispatcher typically registers the modules it
// serves and ignores everything else (including its own replies echoed back).
//
// Header-only and dependency-free (only espp::stream_frame + the standard
// library), so it builds and unit-tests on a host.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <span>
#include <utility>
#include <vector>

#include "stream_frame.hpp"

namespace espp {

/// @brief Routes framed messages from one byte stream to per-module handlers.
class Dispatcher {
public:
  /// @brief Handler invoked for every frame whose module was registered.
  /// @param frame The decoded frame (module, type, flags/reply, payload).
  using handler_fn = std::function<void(const stream_frame::Frame &frame)>;

  /// @brief The module id a frame will route to (its `module` byte).
  static constexpr uint8_t module_of(const stream_frame::Frame &frame) { return frame.module; }

  /// @brief Register (or replace) the handler for a module id.
  /// @param module_id Module id (0..255) to route to @p handler.
  /// @param handler Callback for frames with this module id. A null handler
  ///        unregisters the module.
  void register_module(uint8_t module_id, handler_fn handler) {
    const auto it = std::find_if(handlers_.begin(), handlers_.end(),
                                 [module_id](const auto &e) { return e.first == module_id; });
    if (it != handlers_.end()) {
      if (handler)
        it->second = std::move(handler);
      else
        handlers_.erase(it);
    } else if (handler) {
      handlers_.emplace_back(module_id, std::move(handler));
    }
  }

  /// @brief Remove the handler for a module id (frames for it become ignored).
  void unregister_module(uint8_t module_id) {
    std::erase_if(handlers_, [module_id](const auto &e) { return e.first == module_id; });
  }

  /// @brief Whether a handler is registered for a module id.
  bool has_module(uint8_t module_id) const {
    return std::any_of(handlers_.begin(), handlers_.end(),
                       [module_id](const auto &e) { return e.first == module_id; });
  }

  /// @brief Feed raw received bytes: parse and route each complete frame to its
  ///        module's handler. Frames whose module has no handler are ignored
  ///        (so unrelated protocols on the same stream are harmless).
  void feed(std::span<const uint8_t> data) {
    for (const auto &frame : parser_.feed(data))
      dispatch(frame);
  }

  /// @brief Route an already-parsed frame (for callers running their own parser).
  void dispatch(const stream_frame::Frame &frame) const {
    // Copy the handler out before invoking it: a handler that (re-entrantly)
    // calls register_module() / unregister_module() can reallocate or erase
    // handlers_, which would destroy the std::function being executed.
    handler_fn handler;
    {
      const auto it = std::find_if(handlers_.begin(), handlers_.end(),
                                   [&frame](const auto &e) { return e.first == frame.module; });
      if (it == handlers_.end())
        return;
      handler = it->second;
    }
    handler(frame);
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
  // Small set of (module id -> handler); linear scan is fine for the handful of
  // protocols a stream carries, and it costs memory only per registered module
  // (vs a 256-entry table).
  std::vector<std::pair<uint8_t, handler_fn>> handlers_;
};

} // namespace espp

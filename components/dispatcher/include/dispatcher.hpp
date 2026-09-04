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
// Capability discovery: a module can be registered with a ModuleInfo (name, web
// app, description). A connected peer (e.g. a browser hub) can then ask the
// device WHICH modules it runs — over the reserved discovery module id 0xFF —
// and render/link each one. describe() serializes the registered modules for
// callers that own their transmit path; serve_discovery() is a one-liner that
// auto-answers the discovery request. The Dispatcher stays a pure router: it
// only ever sends when you opt in by giving serve_discovery() a reply function.
//
// Header-only and dependency-free (only espp::stream_frame + the standard
// library), so it builds and unit-tests on a host.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <span>
#include <string>
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

  /// @brief Transmit callback for serve_discovery(): sends one already-encoded
  ///        stream_frame back to the peer over the application's transport.
  using reply_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// @brief Optional human/browser-facing metadata advertised for a module.
  /// @details All fields are optional; a module with an empty name is not
  ///          advertised by describe(). Kept short — each string is serialized
  ///          with a one-byte length, so anything past 255 bytes is truncated.
  struct ModuleInfo {
    std::string name;        ///< Human-readable module name, e.g. "MCP266 Console".
    std::string app;         ///< Hosted web-app filename, e.g. "mcp266_console.html" (optional).
    std::string description; ///< One-line description (optional).
  };

  /// @brief Reserved module id for capability discovery. A peer sends a
  ///        Discovery::ListModules request here; the device answers with the
  ///        serialized module list (see describe() / serve_discovery()). Module
  ///        ids 0xF0..0xFF are reserved for dispatcher / meta use.
  static constexpr uint8_t kDiscoveryModule = 0xFF;

  /// @brief `type` values within the discovery module (kDiscoveryModule).
  enum class Discovery : uint8_t {
    ListModules = 0x00, ///< request: list the device's modules; reply payload = describe().
  };

  /// @brief Version byte at the start of a describe() payload, so the wire format
  ///        can evolve without a framing change.
  static constexpr uint8_t kDiscoveryVersion = 1;

  /// @brief The module id a frame will route to (its `module` byte).
  static constexpr uint8_t module_of(const stream_frame::Frame &frame) { return frame.module; }

  /// @brief Register (or replace) the handler for a module id.
  /// @param module_id Module id (0..255) to route to @p handler.
  /// @param handler Callback for frames with this module id. A null handler
  ///        unregisters the module.
  void register_module(uint8_t module_id, handler_fn handler) {
    register_module(module_id, std::move(handler), ModuleInfo{});
  }

  /// @brief Register (or replace) a module's handler AND its discovery metadata.
  /// @param module_id Module id (0..255) to route to @p handler.
  /// @param handler Callback for frames with this module id (a null handler
  ///        unregisters the module; @p info is then ignored).
  /// @param info Metadata advertised to a discovery peer (see describe()).
  /// @note Registration fully replaces any previous entry, so re-registering a
  ///       module with the 2-argument overload clears its metadata.
  void register_module(uint8_t module_id, handler_fn handler, ModuleInfo info) {
    // Mutating handlers_ while a handler runs could reallocate it or destroy the
    // running handler (use-after-free). If called from inside a dispatch (a
    // handler registering/unregistering), defer the change until dispatch
    // unwinds; otherwise apply it immediately.
    Entry entry{module_id, std::move(handler), std::move(info)};
    if (dispatch_depth_ > 0)
      pending_.push_back(std::move(entry));
    else
      apply_register(std::move(entry));
  }

  /// @brief Remove the handler for a module id (frames for it become ignored).
  void unregister_module(uint8_t module_id) { register_module(module_id, nullptr); }

  /// @brief Whether a handler is registered for a module id (reflects applied
  ///        registrations; changes made during a dispatch apply after it ends).
  bool has_module(uint8_t module_id) const {
    return std::any_of(handlers_.begin(), handlers_.end(),
                       [module_id](const Entry &e) { return e.id == module_id; });
  }

  /// @brief Set the device-level info advertised at the head of describe()
  ///        (so a peer can show "Connected to <name> <firmware>").
  void set_device_info(std::string name, std::string firmware = "") {
    device_name_ = std::move(name);
    device_firmware_ = std::move(firmware);
  }

  /// @brief Serialize the device info + every registered module that carries a
  ///        (non-empty) name into the binary discovery payload.
  ///
  /// Layout (all lengths are one byte; strings are [len][bytes], truncated at
  /// 255): [version u8][reserved u8][device_name str][device_fw str]
  /// [module_count u8] then per module [id u8][name str][app str][desc str].
  /// The reserved discovery module (0xFF) is never listed.
  /// @return The payload bytes (to be sent as the ListModules reply).
  std::vector<uint8_t> describe() const {
    std::vector<uint8_t> out;
    out.push_back(kDiscoveryVersion);
    out.push_back(0); // reserved flags
    append_string(out, device_name_);
    append_string(out, device_firmware_);
    // A module is advertised if it carries a name and is not the discovery module.
    const auto advertised = [](const Entry &e) {
      return e.id != kDiscoveryModule && !e.info.name.empty();
    };
    // module_count is a single wire byte, so cap the count (and the number of
    // records emitted below) at 255 rather than overflowing it.
    const auto total = std::count_if(handlers_.begin(), handlers_.end(), advertised);
    const uint8_t count = static_cast<uint8_t>(std::min<std::ptrdiff_t>(total, 255));
    out.push_back(count);
    uint8_t emitted = 0;
    for (const Entry &e : handlers_) {
      if (emitted == count || !advertised(e))
        continue;
      ++emitted;
      out.push_back(e.id);
      append_string(out, e.info.name);
      append_string(out, e.info.app);
      append_string(out, e.info.description);
    }
    return out;
  }

  /// @brief Opt in to auto-answering capability discovery.
  /// @details Registers a handler on kDiscoveryModule that, on a
  ///          Discovery::ListModules request, encodes describe() into a reply
  ///          frame (echoing the request's correlation id, if any) and hands it
  ///          to @p reply for transmission. This is the only path by which a
  ///          Dispatcher ever sends — the app supplies the transport.
  /// @param reply Transmit callback (sends the encoded reply frame).
  void serve_discovery(reply_fn reply) {
    register_module(
        kDiscoveryModule, [this, reply = std::move(reply)](const stream_frame::Frame &f) {
          // Only answer requests (ignore our own replies echoed back) of the right type.
          if (f.is_reply() || f.type != static_cast<uint8_t>(Discovery::ListModules))
            return;
          const auto payload = describe();
          const auto frame = stream_frame::build_frame(true, kDiscoveryModule,
                                                       static_cast<uint8_t>(Discovery::ListModules),
                                                       payload, f.correlation);
          // build_frame yields empty only if the payload exceeds the frame cap
          // (far more modules than any real device); drop rather than send garbage.
          if (reply && !frame.empty())
            reply(frame);
        });
  }

  /// @brief Feed raw received bytes: parse and route each complete frame to its
  ///        module's handler. Frames whose module has no handler are ignored
  ///        (so unrelated protocols on the same stream are harmless).
  void feed(std::span<const uint8_t> data) {
    for (const auto &frame : parser_.feed(data))
      dispatch(frame);
  }

  /// @brief Route an already-parsed frame (for callers running their own parser).
  void dispatch(const stream_frame::Frame &frame) {
    // No per-frame handler copy: register_module()/unregister_module() called
    // from within a handler are deferred (see register_module), so handlers_ is
    // neither reallocated nor is the running handler destroyed while it executes.
    // The depth counter defers until the OUTERMOST dispatch unwinds (a handler
    // may itself feed()/dispatch()).
    //
    // The depth MUST be restored even if the handler throws (e.g. a Python
    // callback raising) — otherwise dispatch_depth_ would stay nonzero forever
    // and every later registration would be deferred and never applied. An RAII
    // guard decrements it on every path (its destructor only touches an int, so
    // it cannot throw during unwinding).
    struct DepthGuard {
      int &depth;
      ~DepthGuard() { --depth; }
    };
    ++dispatch_depth_;
    DepthGuard guard{dispatch_depth_};
    const auto it = std::find_if(handlers_.begin(), handlers_.end(),
                                 [&frame](const Entry &e) { return e.id == frame.module; });
    if (it != handlers_.end())
      it->handler(frame); // may throw; guard still restores the depth
    // Flush deferred registrations only on the normal path of the OUTERMOST
    // dispatch (depth is still 1 here; the guard makes it 0 on scope exit). If a
    // handler threw, pending ops stay queued and are applied on the next
    // dispatch — never lost, and the dispatcher is never wedged.
    if (dispatch_depth_ == 1 && !pending_.empty()) {
      // swap (not move) so pending_ is left in a defined empty state; applying an
      // op never re-enters dispatch, so no new pending ops accrue here.
      std::vector<Entry> ops;
      ops.swap(pending_);
      for (auto &op : ops)
        apply_register(std::move(op));
    }
  }

  /// @brief Discard any partially-buffered frame bytes (transport reconnect or
  ///        RX overflow) so a frame straddling the gap resynchronizes at once.
  void reset() { parser_.reset(); }

  /// @brief Bytes buffered awaiting frame completion.
  size_t buffered() const { return parser_.buffered(); }

  /// @brief Total bytes discarded while resynchronizing (diagnostics).
  size_t dropped_bytes() const { return parser_.dropped_bytes(); }

private:
  /// A registered module: routing id, its handler, and its discovery metadata.
  struct Entry {
    uint8_t id;
    handler_fn handler;
    ModuleInfo info;
  };

  /// Append a length-prefixed string ([len u8][bytes]), truncated at 255 bytes.
  static void append_string(std::vector<uint8_t> &out, const std::string &s) {
    const uint8_t len = static_cast<uint8_t>(std::min<size_t>(s.size(), 255));
    out.push_back(len);
    out.insert(out.end(), s.begin(), s.begin() + len);
  }

  /// Add / replace / (null handler) remove a module's entry in handlers_.
  /// Must not run while a handler is on the stack (see register_module()).
  void apply_register(Entry entry) {
    const auto it = std::find_if(handlers_.begin(), handlers_.end(),
                                 [&entry](const Entry &e) { return e.id == entry.id; });
    if (it != handlers_.end()) {
      if (entry.handler) {
        it->handler = std::move(entry.handler);
        it->info = std::move(entry.info);
      } else {
        handlers_.erase(it);
      }
    } else if (entry.handler) {
      handlers_.push_back(std::move(entry));
    }
  }

  stream_frame::StreamParser parser_;
  // Small set of registered modules; linear scan is fine for the handful of
  // protocols a stream carries, and it costs memory only per registered module
  // (vs a 256-entry table).
  std::vector<Entry> handlers_;
  // Registrations deferred while dispatching (applied when dispatch unwinds).
  std::vector<Entry> pending_;
  // Device-level info advertised at the head of describe().
  std::string device_name_;
  std::string device_firmware_;
  // >0 while a handler is executing (supports nested dispatch).
  int dispatch_depth_{0};
};

} // namespace espp

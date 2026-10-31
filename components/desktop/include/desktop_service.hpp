#pragma once

// espp::DesktopService -- one espp::Desktop as a transport-agnostic dispatcher
// module (detail/desktop_protocol.hpp is the wire spec). Same contract as the
// other espp services: frames for other modules / reply-flagged frames are
// ignored so the service shares a stream, and the `send` callback never runs
// with the service's mutex held.
//
// The service only DECODES and VALIDATES: a malformed request is answered
// with ERROR(EINVAL) right away (under the service's send mutex); every
// well-formed request is handed to the Desktop (Desktop::submit) and handled
// -- replied to, and its events broadcast -- on the desktop task, through the
// sink this service registered with the Desktop. One instance per transport:
//
//   espp::Desktop desktop({...});
//   espp::DesktopService desktop_service(desktop, {.send = [&](auto f) { usb.write_vendor(f); }});
//   dispatcher.register_module(desktop_service);   // module 9 + discovery metadata

#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <system_error>
#include <vector>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

#include "base_component.hpp"
#include "desktop.hpp"
#include "detail/desktop_protocol.hpp"

namespace espp {

/**
 * @brief Serves an espp::Desktop over any framed byte stream (dispatcher
 *        module 9 by default; see Config::module).
 *
 * GET_DESKTOP answers with the DESKTOP snapshot (apps, settings, open
 * windows) followed by the full tree of every open window and every open
 * dialog, and marks this transport attached: from then on the Desktop
 * broadcasts its changes (WINDOW_OPEN / WIDGET_SET / ... ) to it until
 * detach() (e.g. on USB unmount) or until the next GET_DESKTOP after a
 * reconnect. A frame the transport refuses pauses the broadcasts (the
 * transport stays attached, needs_resync() is true) until the Desktop's own
 * retried snapshot gets through or the host sends GET_DESKTOP. LAUNCH_APP /
 * CLOSE_WINDOW are acknowledged with OK / ERROR; the events (WINDOW_EVENT /
 * WIDGET_EVENT / DIALOG_RESULT) are not.
 *
 * **Threading**: an internal mutex covers the parser; the only frame this
 * object sends itself is the ERROR for a malformed request, serialized on a
 * send mutex that is also held while the Desktop's task sends through this
 * transport, so frames never interleave. `send` must not re-enter this object.
 *
 * \section desktop_service_ex1 DesktopService Example
 * \snippet desktop_example.cpp desktop_example
 */
class DesktopService : public BaseComponent {
public:
  using Stream = espp::stream_frame::StreamParser;
  using Type = espp::detail::desktop_protocol::Type;

  /// Default dispatcher module id (9). A routing key only: Config::module
  /// serves on any id, and hosts find it through discovery (by kProtocol).
  static constexpr uint8_t kModule = espp::detail::desktop_protocol::kModule;
  /// Stable protocol identifier + version advertised through discovery.
  static constexpr const char *kProtocol = espp::detail::desktop_protocol::kProtocol;
  static constexpr uint16_t kProtocolVersion = espp::detail::desktop_protocol::kProtocolVersion;

  /// Transmits one encoded frame to the host, all-or-nothing, and returns
  /// whether it was queued. Unlike the other espp services' `send`, which is
  /// void, this one must report a refused frame: the desktop streams state,
  /// so it then pauses streaming to this transport and re-sends the full
  /// snapshot by itself once frames go through again, see needs_resync().
  /// UsbDevice::write_vendor / write_cdc have exactly this contract (bounded
  /// wait for FIFO room, never a partial frame).
  using send_fn = std::function<bool(std::span<const uint8_t> frame)>;

  /// Configuration for the DesktopService.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded frame, all-or-nothing (required).
    /// Dispatcher module id this instance answers on (and stamps on every
    /// frame it sends). A routing key only (0x00..0xEF).
    uint8_t module{kModule};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the service and register its transport with the desktop.
  explicit DesktopService(Desktop &desktop, const Config &config)
      : BaseComponent("DesktopService", config.log_level)
      , desktop_(desktop)
      , config_(config) {
    sink_ = desktop_.add_sink([this](std::span<const uint8_t> frame) { return send_raw(frame); },
                              config.module);
  }

  ~DesktopService() { desktop_.remove_sink(sink_); }

  /// @brief The dispatcher module id this service answers on (Config::module).
  uint8_t module_id() const { return config_.module; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "Desktop",
            .app = "desktop.html",
            .description = "Windowed desktop: apps, windows and widgets drawn by the browser",
            .protocol = kProtocol,
            .protocol_version = kProtocolVersion};
  }

  /// @brief The Desktop sink id of this transport.
  Desktop::SinkId sink() const { return sink_; }

  /// @brief Stop broadcasting to this transport (call when it disconnects,
  ///        e.g. on USB unmount); the next GET_DESKTOP re-attaches it.
  void detach() { desktop_.set_sink_active(sink_, false); }

  /// @brief Whether a host asked for the desktop on this transport (and it
  ///        was not detached since).
  bool attached() const { return desktop_.sink_active(sink_); }

  /// @brief Whether a frame to this transport was dropped (send returned
  ///        false) and the host's mirror is still incomplete: streaming is
  ///        paused (the transport stays attached()) until the desktop's own
  ///        snapshot gets through (retried with back-off) or the host sends
  ///        GET_DESKTOP.
  bool needs_resync() const { return desktop_.sink_needs_resync(sink_); }

  /**
   * @brief Dispatcher entry point: handle one routed frame. Frames for other
   *        modules and reply-flagged frames are ignored, so this can be
   *        registered directly: `dispatcher.register_module(service)`.
   */
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_frame(frame.type, frame.payload, frame.correlation);
  }

  /// @brief Feed received transport bytes (standalone use, without a Dispatcher).
  void feed(std::span<const uint8_t> data) {
    std::vector<espp::stream_frame::Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames = parser_.feed(data);
    }
    for (const auto &frame : frames)
      handle(frame);
  }

  /// @brief Discard any partially-buffered frame bytes (standalone feed() use).
  void reset_parser() {
    std::lock_guard<std::mutex> lock(mutex_);
    parser_.reset();
  }

  /**
   * @brief Handle one already-parsed request frame.
   * @return true if the type belongs to the desktop protocol (it was queued
   *         for the desktop task, or answered with ERROR), false if ignored.
   */
  /// @param correlation The request frame's correlation id, if it carried one;
  ///        the reply (DESKTOP / OK / ERROR) echoes it.
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload,
                    std::optional<uint16_t> correlation = std::nullopt) {
    namespace proto = espp::detail::desktop_protocol;
    Desktop::Command cmd{.sink = sink_, .correlation = correlation};
    switch (static_cast<Type>(type)) {
    case Type::GetDesktop:
      if (!payload.empty()) {
        send_error(type, "GET_DESKTOP takes no payload", correlation);
        return true;
      }
      cmd.request = Desktop::GetDesktop{};
      break;
    case Type::LaunchApp: {
      const auto r = proto::decode_launch_app(payload);
      if (!r) {
        send_error(type, "malformed LAUNCH_APP (expected u8 app)", correlation);
        return true;
      }
      cmd.request = *r;
      break;
    }
    case Type::CloseWindow: {
      const auto r = proto::decode_close_window(payload);
      if (!r) {
        send_error(type, "malformed CLOSE_WINDOW (expected u16 window)", correlation);
        return true;
      }
      cmd.request = *r;
      break;
    }
    case Type::WindowEvent: {
      const auto r = proto::decode_window_event(payload);
      if (!r) {
        send_error(type,
                   "malformed WINDOW_EVENT (expected u16 window, u8 event, i16 x, i16 y, "
                   "u16 w, u16 h)",
                   correlation);
        return true;
      }
      cmd.request = *r;
      break;
    }
    case Type::WidgetEvent: {
      const auto r = proto::decode_widget_event(payload);
      if (!r) {
        send_error(type, "malformed WIDGET_EVENT (unknown event kind or wrong value size)",
                   correlation);
        return true;
      }
      cmd.request = *r;
      break;
    }
    case Type::DialogResult: {
      const auto r = proto::decode_dialog_result(payload);
      if (!r) {
        send_error(type, "malformed DIALOG_RESULT (expected u16 dialog, u8 button, text)",
                   correlation);
        return true;
      }
      cmd.request = *r;
      break;
    }
    default:
      return false; // not a desktop request: ignore so the service can share a stream
    }
    desktop_.submit(std::move(cmd));
    return true;
  }

protected:
  /// Transmit a frame the Desktop built (its sink callback). Serialized with
  /// the service's own ERROR frames on send_mutex_. Returns whether it was queued.
  bool send_raw(std::span<const uint8_t> frame) {
    if (frame.empty())
      return true;
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    if (!config_.send) {
      logger_.warn_rate_limited("no send function configured; dropping a {}-byte frame",
                                frame.size());
      return false;
    }
    return config_.send(frame);
  }

  void send_error(uint8_t request_type, std::string_view message,
                  std::optional<uint16_t> correlation) {
    namespace proto = espp::detail::desktop_protocol;
    logger_.warn("{} (type 0x{:02x})", message, request_type);
    const auto frame = proto::build_frame(
        Type::Error,
        proto::encode_error(
            request_type,
            static_cast<uint32_t>(std::make_error_code(std::errc::invalid_argument).value()),
            message),
        module_id(), correlation);
    send_raw(frame);
  }

private:
  Desktop &desktop_;
  Config config_;
  Desktop::SinkId sink_{0};
  mutable std::mutex mutex_;      ///< guards the parser
  mutable std::mutex send_mutex_; ///< serializes every outbound frame across `send`
  Stream parser_;
};

// Compile-time check that the service keeps satisfying the dispatcher's module
// contract (module_id() / module_info() / handle(frame)).
static_assert(DispatcherModuleConcept<DesktopService>);

} // namespace espp

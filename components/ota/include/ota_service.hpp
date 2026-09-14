#pragma once

// espp::OtaService — the OTA stream protocol (detail/ota_stream_protocol.hpp)
// as a transport-agnostic *service*: hand it an espp::Ota engine and a `send`
// function, register it on an espp::Dispatcher (or feed it a byte stream), and
// the whole request/reply state machine — BEGIN/DATA/FINISH/ABORT, session
// ownership, error replies, rollback status/confirmation, the post-update
// restart — lives here instead of being re-implemented by every application.
//
// It is the OTA counterpart of espp::CoreDumpService and follows the same
// contract: requests are handled (and the reply built) under an internal
// mutex, the `send` callback is always invoked AFTER that mutex is released,
// and frames for other modules / reply-flagged frames are ignored so the
// service coexists with other protocols on one stream.
//
// Typical wiring (one line per service):
//
//   espp::Ota ota({...});
//   espp::OtaService ota_service(ota, {.send = [&](auto f) { usb.write_vendor(f); }});
//   dispatcher.register_module(ota_service);   // module 0 + discovery metadata
//
// Multi-transport: create one OtaService per byte stream (they may share one
// espp::Ota). Each instance only ever appends to / finishes / aborts a session
// IT began, so a DATA frame arriving on another transport cannot corrupt an
// in-progress update — the same guarantee the ota example used to implement
// by hand with an `owns_session` flag.

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include "dispatcher.hpp"
#include "stream_frame.hpp"

#include "base_component.hpp"
#include "detail/ota_stream_protocol.hpp"
#include "ota.hpp"

namespace espp {

/**
 * @brief Transport-agnostic service exposing an espp::Ota engine over any
 *        framed byte stream (dispatcher module 0).
 *
 * See detail/ota_stream_protocol.hpp for the wire protocol. Flow control is
 * one request in flight: the host waits for OK / ERROR before sending the
 * next DATA frame, so a well-behaved host never queues more than ~one frame.
 *
 * **Restart after an update**: with `Config::auto_restart` (the default) a
 * successful FINISH replies OK and then restarts the device after
 * `Config::restart_delay` (from a detached thread, so the reply has left the
 * transport). Set `auto_restart = false` to decide yourself: the
 * `on_update_finished` callback fires (outside the lock) and the app calls
 * `Ota::restart()` when convenient.
 *
 * **Rollback is host-driven**: after an update the new image boots
 * PENDING_VERIFY; the host confirms it with MARK_VALID (or rejects it with
 * MARK_INVALID) after checking the device is healthy. This service never
 * marks the running image valid on its own.
 *
 * **Threading**: an internal mutex covers the parser, the session ownership
 * flag and each engine call, and the `send` callback always runs after it is
 * released (so a re-entrant transport cannot deadlock). Requests are handled
 * one at a time, but the mutex is released between the frames of one feed()
 * call and the OTA protocol is order-sensitive (BEGIN, DATA..., FINISH), so
 * drive one instance from ONE context per byte stream — a Dispatcher /
 * DispatcherWorker feeding it, or a single task calling feed(). Engine calls
 * block (a BEGIN erases the target partition, which can take seconds), so
 * that context should be a worker task rather than a transport's receive
 * callback (see espp::DispatcherWorker).
 *
 * \section ota_service_ex1 OtaService Example
 * \snippet ota_example.cpp ota_example
 */
class OtaService : public BaseComponent {
public:
  /// Frame-stream parser type (from the shared stream_frame codec).
  using Stream = espp::stream_frame::StreamParser;
  /// The OTA wire protocol (message types, frame builders).
  using MessageType = espp::detail::ota_stream::MessageType;

  /// Dispatcher module id owned by the OTA protocol.
  static constexpr uint8_t kModule = espp::detail::ota_stream::kModule;

  /// Transmits one encoded reply frame to the host.
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;
  /// Notified (outside the lock) after a successful FINISH has been acknowledged.
  using finished_fn = std::function<void()>;

  /// Configuration for the OtaService.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded reply frame (required).
    /// Restart the device after a successful FINISH (after the OK reply).
    bool auto_restart{true};
    /// Delay between the OK reply and the restart, so the reply reaches the host.
    std::chrono::milliseconds restart_delay{750};
    /// Called after a successful FINISH was acknowledged (before the restart,
    /// if any). With `auto_restart = false` this is where the app schedules
    /// its own `Ota::restart()`.
    finished_fn on_update_finished{nullptr};
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /**
   * @brief Construct the service.
   * @param ota The OTA engine to drive (may be shared between several service
   *        instances / transports; must outlive the service).
   * @param config Configuration parameters (the reply `send` function, ...).
   */
  explicit OtaService(Ota &ota, const Config &config)
      : BaseComponent("OtaService", config.log_level)
      , ota_(ota)
      , config_(config) {}

  /// @brief The dispatcher module id this service answers on (kModule: the
  ///        OTA protocol's fixed id, which the OTA console / CLI expect).
  uint8_t module_id() const { return kModule; }

  /// @brief Discovery metadata for registering this service on a Dispatcher.
  Dispatcher::ModuleInfo module_info() const {
    return {.name = "OTA",
            .app = "ota_console.html",
            .description = "Firmware update over the framed stream"};
  }

  /// @brief Whether an update session begun through THIS service is in progress.
  bool owns_session() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return owns_session_;
  }

  /**
   * @brief Dispatcher entry point: handle one routed frame.
   *
   * Frames for other modules and reply-flagged frames (e.g. an echo) are
   * ignored, so this can be registered directly:
   * `dispatcher.register_module(service)`.
   */
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.module != module_id() || frame.is_reply())
      return;
    handle_frame(frame.type, frame.payload);
  }

  /**
   * @brief Feed received transport bytes (standalone use, without a Dispatcher).
   *
   * Runs the internal incremental frame parser and handles every complete
   * request frame of this module, one reply at a time.
   */
  void feed(std::span<const uint8_t> data) {
    std::vector<espp::stream_frame::Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames = parser_.feed(data);
    }
    for (const auto &frame : frames)
      handle(frame);
  }

  /**
   * @brief Handle one already-parsed request frame.
   * @param type The frame type byte (MessageType).
   * @param payload The frame payload bytes.
   * @return true if the type belongs to the OTA protocol (a reply was
   *         produced and sent), false if it was ignored.
   * @note The reply `send` callback is invoked after the internal mutex has
   *       been released.
   */
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload) {
    std::vector<uint8_t> reply;
    bool handled;
    bool finished = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handled = handle_frame_locked(type, payload, reply, finished);
    }
    // send outside the lock so a re-entrant transport cannot deadlock
    send(reply);
    if (finished)
      on_finished();
    return handled;
  }

  /// @brief Discard any partially-buffered frame bytes (standalone feed() use).
  void reset_parser() {
    std::lock_guard<std::mutex> lock(mutex_);
    parser_.reset();
  }

  /**
   * @brief Tell the service that received bytes were dropped (transport RX
   *        overflow). An image being transferred through this service is now
   *        unusable: its session is aborted and the host is told (ERROR) to
   *        restart the update. Also resets the standalone parser.
   * @return true if an update session owned by this service was aborted (an
   *         ERROR reply was sent); false if no transfer was in progress here
   *         (nothing is sent -- the dropped bytes belonged to another module,
   *         which may want to reply on its own).
   */
  bool on_rx_overflow() {
    std::vector<uint8_t> reply;
    bool aborted = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      parser_.reset();
      if (owns_session_) {
        std::error_code ec;
        ota_.abort(ec);
        owns_session_ = false;
        aborted = true;
        reply = build_error(std::errc::no_buffer_space,
                            "RX overflow: frames dropped; transfer aborted -- wait for OK "
                            "replies between frames and restart the update");
      }
    }
    send(reply);
    return aborted;
  }

protected:
  /// Handle one frame with the mutex held: drive the engine and BUILD the
  /// reply into @p reply (not sent here). Sets @p finished when a FINISH
  /// succeeded so the caller can run the post-update hooks outside the lock.
  bool handle_frame_locked(uint8_t type, std::span<const uint8_t> payload,
                           std::vector<uint8_t> &reply, bool &finished) {
    namespace proto = espp::detail::ota_stream;
    std::error_code ec;
    switch (static_cast<MessageType>(type)) {
    case MessageType::Begin: {
      if (payload.size() != 4) {
        reply = build_error(std::errc::invalid_argument, "malformed BEGIN (expected u32 size)");
        return true;
      }
      const uint32_t image_size = espp::stream_frame::get_u32(payload);
      if (ota_.begin(image_size, ec)) {
        owns_session_ = true;
        logger_.info("BEGIN: update session started ({} bytes expected)", image_size);
        reply = proto::make_ok(0);
      } else {
        // busy = another transport's session; ownership stays false
        reply = build_error(ec, "begin failed");
      }
      return true;
    }
    case MessageType::Data:
      if (!owns_session_) {
        reply = build_error(std::errc::operation_not_permitted,
                            "no update session on this transport (send BEGIN first)");
        return true;
      }
      if (ota_.write(payload, ec)) {
        reply = proto::make_ok(static_cast<uint32_t>(ota_.bytes_written()));
      } else {
        owns_session_ = false; // write() aborted the session on failure
        reply = build_error(ec, "write failed");
      }
      return true;
    case MessageType::Finish: {
      if (!owns_session_) {
        reply = build_error(std::errc::operation_not_permitted,
                            "no update session on this transport (send BEGIN first)");
        return true;
      }
      const auto written = static_cast<uint32_t>(ota_.bytes_written());
      owns_session_ = false; // finish() ends the session in all outcomes
      if (ota_.finish(ec)) {
        logger_.info("FINISH: image validated and activated ({} bytes)", written);
        reply = proto::make_ok(written);
        finished = true;
      } else {
        reply = build_error(ec, "finish (validate/activate) failed");
      }
      return true;
    }
    case MessageType::Abort: {
      if (!owns_session_) {
        reply = build_error(std::errc::operation_not_permitted,
                            "no update session on this transport to abort");
        return true;
      }
      const auto written = static_cast<uint32_t>(ota_.bytes_written());
      owns_session_ = false; // session over either way
      if (ota_.abort(ec)) {
        logger_.info("ABORT: session discarded after {} bytes", written);
        reply = proto::make_ok(written);
      } else {
        reply = build_error(ec, "abort failed");
      }
      return true;
    }
    case MessageType::GetStatus: {
      // Rollback status + the running firmware (so the host can show what is
      // now running before confirming it). Session-independent.
      uint8_t flags = 0;
#if defined(CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE)
      flags |= proto::kStatusRollbackSupported;
      if (ota_.is_pending_verify())
        flags |= proto::kStatusPendingVerify;
#endif
      const auto desc = ota_.running_app_description();
      reply = proto::make_status(flags, desc.version, desc.project_name);
      return true;
    }
    case MessageType::MarkValid:
      // The HOST confirms the running image after its own health checks.
      if (ota_.mark_app_valid(ec)) {
        logger_.info("MARK_VALID: running image confirmed");
        reply = proto::make_ok(0);
      } else {
        reply = build_error(ec, "mark valid failed");
      }
      return true;
    case MessageType::MarkInvalid:
      // Does NOT return on success (the device reboots into the previous
      // image): the reboot / disconnect IS the success signal to the host.
      // Only a failure (e.g. nothing to roll back to) produces a reply.
      ota_.mark_app_invalid_and_rollback(ec);
      reply = build_error(ec, "rollback failed");
      return true;
    default:
      // Not an OTA request: ignore so the service can share a stream.
      return false;
    }
  }

  /// Post-FINISH hooks, run WITHOUT the mutex held and after the OK reply.
  void on_finished() {
    if (config_.on_update_finished)
      config_.on_update_finished();
    if (!config_.auto_restart)
      return;
    // reply first, then restart into the new image from a detached thread so
    // the caller's task (typically the transport worker) is never blocked
    logger_.info("restarting in {} ms", config_.restart_delay.count());
    std::thread([delay = config_.restart_delay, &ota = ota_]() {
      std::this_thread::sleep_for(delay);
      ota.restart();
    }).detach();
  }

  /// Transmit an encoded reply frame. Must be called WITHOUT the mutex held.
  void send(const std::vector<uint8_t> &frame) {
    if (frame.empty())
      return;
    if (!config_.send) {
      logger_.warn("no send function configured; dropping a {}-byte reply", frame.size());
      return;
    }
    config_.send(frame);
  }

  /// Build an ERROR reply (u32 code + "context: message"); the code is the
  /// std::errc-normalized value (informational; the message is authoritative).
  std::vector<uint8_t> build_error(const std::error_code &ec, std::string_view context) const {
    logger_.error("{}: {}", context, ec.message());
    int code = ec.value();
    if (ec.category() != std::generic_category()) {
      const std::error_condition cond = ec.default_error_condition();
      code = (cond.category() == std::generic_category()) ? cond.value()
                                                          : static_cast<int>(std::errc::io_error);
    }
    return espp::detail::ota_stream::make_error(static_cast<uint32_t>(code),
                                                std::string(context) + ": " + ec.message());
  }

  std::vector<uint8_t> build_error(std::errc errc, std::string_view context) const {
    return build_error(std::make_error_code(errc), context);
  }

private:
  Ota &ota_;
  Config config_;
  mutable std::mutex mutex_;
  Stream parser_;
  bool owns_session_{false}; // set by a successful BEGIN here; cleared on every terminal path
};

} // namespace espp

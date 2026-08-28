#pragma once

// espp core-dump stream service — message ids + a transport-agnostic request
// handler layered on the espp `ota_stream` framing (magic "OT" + type u8 +
// len u32 + payload + CRC-32, all little-endian; see
// components/ota/include/detail/ota_stream_protocol.hpp for the authoritative
// framing spec). Reusing the framing means the parser's resynchronization
// works unchanged, and — because message TYPES live in a dedicated range —
// the core-dump service can share one byte stream with other espp protocols
// (the OTA protocol, an application protocol, or free-form console text)
// without ambiguity: each protocol handler simply ignores frame types outside
// its own range.
//
// Message types & payloads (host -> device), range 0x40..0x4F:
//   0x40 GET_SUMMARY — no payload. Reply: SUMMARY.
//   0x41 GET_SIZE    — no payload. Reply: SIZE.
//   0x42 READ        — payload: u32 offset + u16 length. Reply: DATA / ERROR.
//   0x43 ERASE       — no payload. Reply: OK / ERROR.
//
// Message types & payloads (device -> host), range 0xC0..0xCF:
//   0xC0 SUMMARY — payload: UTF-8 crash report text (espp::CoreDump::
//                  format_report()); EMPTY payload = clean boot history.
//   0xC1 SIZE    — payload: u32 total core-dump image size in bytes (0 = no
//                  core dump present).
//   0xC2 DATA    — payload: u32 offset + the requested image bytes.
//   0xC3 OK      — payload: u32 context-dependent value (ERASE reply).
//   0xC4 ERROR   — payload: u32 code (always a std::errc value; error codes
//                  from other categories are normalized before sending) +
//                  UTF-8 message.
//
// Flow control: the host serializes transactions — one request in flight,
// wait for its reply. READ length is capped so the DATA reply (4-byte offset
// + data) fits the framing's 4096-byte payload cap.

#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "detail/ota_stream_protocol.hpp"

#include "base_component.hpp"
#include "coredump.hpp"

namespace espp {

/**
 * @brief Transport-agnostic service exposing the flash core dump (see
 *        espp::CoreDump) over any framed byte stream.
 *
 * The service answers the core-dump protocol requests (GET_SUMMARY /
 * GET_SIZE / READ / ERASE — see `coredump_service.hpp`'s header comment for
 * the wire spec) with replies encoded by the espp `ota_stream` framing. It is
 * constructed with a `send` function that transmits an encoded reply frame,
 * so mounting it on a transport takes a few lines:
 *
 * - **USB vendor / WebUSB**: `send` = `usb.write_vendor(frame)`, and call
 *   `feed(data)` from the vendor receive callback.
 * - **USB CDC / Web Serial**: `send` = `usb.write_cdc(frame)`, and call
 *   `feed(data)` from the CDC receive callback. The framing parser
 *   resynchronizes on the frame magic, so the SAME stream can also carry
 *   console text (the espp core-dump web console renders the text and the
 *   frames side by side).
 * - **Sockets / UART / ...**: same pattern with the transport's send / receive.
 *
 * `feed()` runs an internal incremental frame parser and dispatches every
 * complete, CRC-verified frame to `handle_frame()`; a transport that already
 * parses frames itself (e.g. one shared parser for several protocols) can
 * call `handle_frame(type, payload)` directly. Frame types outside the
 * core-dump range are IGNORED (`handle_frame()` returns false, nothing is
 * sent), so the service coexists with other protocols on one stream.
 *
 * Each instance owns one parser, so create one instance per byte stream (they
 * can all share the same espp::CoreDump).
 *
 * **Threading & the `send` callback contract**: `feed()` / `handle_frame()` /
 * `reset_parser()` are serialized against each other by an internal mutex,
 * which covers the parser state, the flash access, and building the reply
 * frame — but the `send` callback is always invoked AFTER that mutex is
 * released. `send` may therefore freely call back into the service (e.g. a
 * loopback transport, or an error path that calls `reset_parser()`) without
 * deadlocking. The flip side: the service does NOT serialize `send` itself —
 * with the recommended one-instance-per-stream design each instance's `send`
 * is only ever called from that stream's single receive context, but if you
 * do feed one instance from multiple tasks, `send` can be invoked
 * concurrently and must be thread-safe.
 *
 * @note `handle_frame()` runs the flash access (and the reply `send`) in the
 *       caller's context. Reads are fast, but ERASE can take tens of
 *       milliseconds — when feeding from a latency-sensitive context (e.g.
 *       the TinyUSB task), queue the received bytes and `feed()` from a
 *       worker task (see the example).
 *
 * \section coredump_service_ex1 CoreDumpService Example
 * \snippet coredump_example.cpp coredump_example
 */
class CoreDumpService : public BaseComponent {
public:
  /// Frame-stream helpers shared with the espp `ota` component.
  using Stream = espp::detail::ota_stream::StreamParser;

  /// Core-dump protocol message types (carried in the ota_stream frame `type`
  /// byte; see the header comment for the payload spec).
  enum class Msg : uint8_t {
    // host -> device
    GetSummary = 0x40, ///< request the crash report text
    GetSize = 0x41,    ///< request the core-dump image size
    Read = 0x42,       ///< read image bytes (u32 offset + u16 length)
    Erase = 0x43,      ///< erase the stored core dump
    // device -> host
    Summary = 0xC0, ///< UTF-8 crash report (empty = clean boot history)
    Size = 0xC1,    ///< u32 image size (0 = no core dump)
    Data = 0xC2,    ///< u32 offset + image bytes
    Ok = 0xC3,      ///< u32 context-dependent success value
    Error = 0xC4,   ///< u32 code (std::errc) + UTF-8 message
  };

  /// Maximum image bytes per READ request / DATA reply (the DATA payload is
  /// a 4-byte offset plus the data, capped by the framing's payload limit).
  static constexpr size_t kMaxReadLength = espp::detail::ota_stream::kMaxPayloadSize - 4;

  /**
   * @brief Function used to transmit one encoded reply frame to the host.
   * @param frame The complete encoded frame bytes (header + payload + CRC).
   */
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// Configuration for the CoreDumpService.
  struct Config {
    send_fn send{nullptr}; ///< Transmits an encoded reply frame (required).
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /**
   * @brief Construct the service.
   * @param core_dump The core-dump accessor to serve (may be shared between
   *        several service instances / transports; must outlive the service).
   * @param config Configuration parameters (the reply `send` function).
   */
  explicit CoreDumpService(CoreDump &core_dump, const Config &config)
      : BaseComponent("CoreDumpService", config.log_level)
      , core_dump_(core_dump)
      , send_(config.send) {}

  /**
   * @brief Feed received transport bytes to the service.
   *
   * Runs the internal incremental frame parser (arbitrary chunking, CRC
   * verification, resynchronization past non-frame bytes such as console
   * text) and dispatches every complete frame to handle_frame().
   *
   * @param data Any number of received bytes.
   *
   * @note The reply `send` callback is invoked after the internal mutex has
   *       been released (see the class-level threading notes).
   */
  void feed(std::span<const uint8_t> data) {
    std::vector<std::vector<uint8_t>> replies;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &frame : parser_.feed(data)) {
        std::vector<uint8_t> reply;
        if (handle_frame_locked(static_cast<uint8_t>(frame.type), frame.payload, reply) &&
            !reply.empty())
          replies.push_back(std::move(reply));
      }
    }
    // send outside the lock so a re-entrant transport cannot deadlock
    for (const auto &reply : replies)
      send(reply);
  }

  /**
   * @brief Handle one already-parsed frame.
   * @param type The frame type byte.
   * @param payload The frame payload bytes.
   * @return true if the frame type belongs to the core-dump protocol (a reply
   *         was sent), false if it was ignored (another protocol's frame —
   *         nothing is sent, so multiple services can share one stream).
   *
   * @note The reply `send` callback is invoked after the internal mutex has
   *       been released (see the class-level threading notes).
   */
  bool handle_frame(uint8_t type, std::span<const uint8_t> payload) {
    std::vector<uint8_t> reply;
    bool handled;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      handled = handle_frame_locked(type, payload, reply);
    }
    // send outside the lock so a re-entrant transport cannot deadlock
    if (!reply.empty())
      send(reply);
    return handled;
  }

  /// @brief Discard any partially-buffered frame bytes (e.g. on transport
  ///        reconnect or after an RX overflow).
  void reset_parser() {
    std::lock_guard<std::mutex> lock(mutex_);
    parser_.reset();
  }

protected:
  /// Handle one frame with the mutex held: perform the flash access and BUILD
  /// the encoded reply frame into @p reply — but do NOT send it. The caller
  /// (feed() / handle_frame()) transmits @p reply after releasing the mutex,
  /// so the user `send` callback never runs under the internal lock. See
  /// handle_frame().
  bool handle_frame_locked(uint8_t type, std::span<const uint8_t> payload,
                           std::vector<uint8_t> &reply) {
    namespace stream = espp::detail::ota_stream;
    switch (static_cast<Msg>(type)) {
    case Msg::GetSummary: {
      const std::string report = core_dump_.format_report();
      logger_.info("GET_SUMMARY -> {} bytes", report.size());
      // truncate to the frame payload cap (reports are far smaller in practice)
      const size_t count = std::min(report.size(), stream::kMaxPayloadSize);
      reply =
          build(Msg::Summary,
                std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(report.data()), count));
      return true;
    }
    case Msg::GetSize: {
      const auto size = static_cast<uint32_t>(core_dump_.image_size());
      logger_.info("GET_SIZE -> {} bytes", size);
      std::vector<uint8_t> reply_payload;
      stream::put_u32(reply_payload, size);
      reply = build(Msg::Size, reply_payload);
      return true;
    }
    case Msg::Read: {
      if (payload.size() != 6) {
        reply = build_error(std::errc::invalid_argument, "READ needs u32 offset + u16 length");
        return true;
      }
      const uint32_t offset = stream::get_u32(payload);
      const uint16_t length =
          static_cast<uint16_t>(payload[4]) | (static_cast<uint16_t>(payload[5]) << 8);
      if (length > kMaxReadLength) {
        reply =
            build_error(std::errc::invalid_argument, "READ length exceeds the per-frame maximum");
        return true;
      }
      std::vector<uint8_t> reply_payload;
      reply_payload.reserve(4 + length);
      stream::put_u32(reply_payload, offset);
      reply_payload.resize(4 + length);
      std::error_code ec;
      if (!core_dump_.read_image(offset, std::span<uint8_t>(reply_payload.data() + 4, length),
                                 ec)) {
        reply = build_error(ec, "READ failed");
        return true;
      }
      logger_.debug("READ offset {} length {}", offset, length);
      reply = build(Msg::Data, reply_payload);
      return true;
    }
    case Msg::Erase: {
      std::error_code ec;
      // cppcheck-suppress knownConditionTrueFalse // erase() is a constant
      // only in the coredump-disabled configuration cppcheck analyzes
      if (!core_dump_.erase(ec)) {
        reply = build_error(ec, "ERASE failed");
        return true;
      }
      logger_.info("ERASE ok");
      std::vector<uint8_t> reply_payload;
      stream::put_u32(reply_payload, 0);
      reply = build(Msg::Ok, reply_payload);
      return true;
    }
    default:
      // Not a core-dump protocol frame: IGNORE it (no error reply), so this
      // service can share a stream with other espp protocols.
      return false;
    }
  }

  /// Build an encoded frame for a core-dump protocol message type.
  static std::vector<uint8_t> build(Msg type, std::span<const uint8_t> payload = {}) {
    namespace stream = espp::detail::ota_stream;
    return stream::build_frame(static_cast<stream::MessageType>(type), payload);
  }

  /// Transmit an encoded reply frame via the configured send function. Must
  /// be called WITHOUT the internal mutex held (user callbacks never run
  /// under the lock).
  void send(const std::vector<uint8_t> &frame) {
    if (frame.empty())
      return;
    if (!send_) {
      logger_.warn("no send function configured; dropping a {}-byte reply", frame.size());
      return;
    }
    send_(frame);
  }

  /// Build an ERROR reply frame (u32 std::errc code + UTF-8 context message).
  /// Codes from categories other than the generic category are normalized to
  /// the equivalent std::errc via default_error_condition() — falling back to
  /// std::errc::io_error when there is no generic equivalent — so the on-wire
  /// code is always a std::errc value and stable for the host to interpret
  /// (the message text still carries the original category's description).
  std::vector<uint8_t> build_error(const std::error_code &ec, std::string_view context) {
    namespace stream = espp::detail::ota_stream;
    logger_.error("{}: {}", context, ec.message());
    int code = ec.value();
    if (ec.category() != std::generic_category()) {
      const std::error_condition cond = ec.default_error_condition();
      code = (cond.category() == std::generic_category()) ? cond.value()
                                                          : static_cast<int>(std::errc::io_error);
    }
    std::vector<uint8_t> payload;
    stream::put_u32(payload, static_cast<uint32_t>(code));
    const std::string message = std::string(context) + ": " + ec.message();
    const size_t count = std::min(message.size(), stream::kMaxPayloadSize - payload.size());
    payload.insert(payload.end(), message.begin(), message.begin() + count);
    return build(Msg::Error, payload);
  }

  /// Overload taking a std::errc directly.
  std::vector<uint8_t> build_error(std::errc errc, std::string_view context) {
    return build_error(std::make_error_code(errc), context);
  }

private:
  CoreDump &core_dump_;
  send_fn send_;
  std::mutex mutex_;
  Stream parser_;
};

} // namespace espp

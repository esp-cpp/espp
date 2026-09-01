#pragma once

// espp core-dump stream service — message ids + a transport-agnostic request
// handler layered on the espp `stream_frame` codec (magic "OT" + flags u8 +
// module u8 + type u8 + len u32 + payload + CRC-32, all little-endian; see
// components/stream_frame/include/stream_frame.hpp for the authoritative
// framing spec). The core-dump protocol owns dispatcher MODULE 4, so it can
// share one byte stream with other espp protocols (OTA on module 0, an
// application protocol, or free-form console text): frames for other modules
// are simply ignored (route with espp::Dispatcher, or call handle_frame()
// after routing by module).
//
// Message types & payloads (host -> device), module 4:
//   0x40 GET_SUMMARY — no payload. Reply: SUMMARY.
//   0x41 GET_SIZE    — no payload. Reply: SIZE.
//   0x42 READ        — payload: u32 offset + u16 length. Reply: DATA / ERROR.
//   0x43 ERASE       — no payload. Reply: OK / ERROR.
//
// Message types & payloads (device -> host), module 4, reply flag set:
//   0xC0 SUMMARY — payload: UTF-8 crash report text (espp::CoreDump::
//                  format_report()); EMPTY payload = clean boot history.
//   0xC1 SIZE    — payload: u32 total core-dump image size in bytes (0 = no
//                  core dump present).
//   0xC2 DATA    — payload: u32 offset + the requested image bytes.
//   0xC3 OK      — payload: u32 context-dependent value (ERASE reply).
//   0xC4 ERROR   — payload: u32 code + UTF-8 message. The MESSAGE is the
//                  authoritative, self-sufficient description of the error;
//                  the code is informational / best-effort (a std::errc value
//                  as numbered by the device's C++ stdlib — stable for the
//                  espp/ESP-IDF newlib toolchain, but std::errc numbering is
//                  not standardized across stdlibs, so hosts should display
//                  code + message verbatim and MUST NOT branch on specific
//                  code values).
//
// Flow control: the host serializes transactions — one request in flight,
// wait for its reply. READ length is capped so the DATA reply (4-byte offset
// + data) fits the framing's 4096-byte payload cap.

#include <cstdint>
#include <functional>
#include <limits>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "stream_frame.hpp"

#include "base_component.hpp"
#include "coredump.hpp"

namespace espp {

/**
 * @brief Transport-agnostic service exposing the flash core dump (see
 *        espp::CoreDump) over any framed byte stream.
 *
 * The service answers the core-dump protocol requests (GET_SUMMARY /
 * GET_SIZE / READ / ERASE — see `coredump_service.hpp`'s header comment for
 * the wire spec) with replies encoded by the espp `stream_frame` codec. It is
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
 * call `handle_frame(type, payload)` directly (after routing by module).
 * feed() ignores frames for other modules, and handle_frame() returns false
 * for types outside the core-dump protocol, so the service coexists with
 * other protocols on one stream.
 *
 * Each instance owns one parser, so create one instance per byte stream (they
 * can all share the same espp::CoreDump: it serializes its flash-touching
 * methods with its own internal mutex, so e.g. a READ arriving on one
 * transport cannot interleave with an ERASE arriving on another).
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
  /// Frame-stream parser type (from the shared stream_frame codec).
  using Stream = espp::stream_frame::StreamParser;

  /// Core-dump protocol message types (the stream_frame `type` byte within
  /// module 4; see the header comment for the payload spec).
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
    Error = 0xC4,   ///< u32 informational code + authoritative UTF-8 message
  };

  /// Dispatcher module id owned by the core-dump protocol (the frame `module`
  /// byte). Reply Msg values keep the high bit set, which build() maps to the
  /// frame reply flag.
  static constexpr uint8_t kModule = 4;

  /// Maximum image bytes per READ request / DATA reply (the DATA payload is
  /// a 4-byte offset plus the data, capped by the framing's payload limit).
  static constexpr size_t kMaxReadLength = espp::stream_frame::kMaxPayloadSize - 4;

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
   * text) and processes every complete frame (see handle_frame()).
   *
   * @param data Any number of received bytes.
   *
   * @note The reply `send` callback is invoked after the internal mutex has
   *       been released (see the class-level threading notes). Frames are
   *       processed and their replies sent ONE AT A TIME, so reply memory is
   *       bounded at a single frame regardless of how many requests one input
   *       chunk carries (a max-length READ request is only 17 bytes on the
   *       wire while its DATA reply can be ~4 KiB, so accumulating all the
   *       replies first would let a single 4 KiB receive chunk materialize
   *       close to 1 MiB).
   */
  void feed(std::span<const uint8_t> data) {
    // Parse ALL of the received bytes under the lock ONCE (the parsed frames
    // are owned copies, so their memory is bounded by the input size and the
    // parser state stays consistent), then drain the frame queue: re-take the
    // lock per frame to build its single reply, release it, send, repeat —
    // only one reply frame is ever alive.
    std::vector<espp::stream_frame::Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames = parser_.feed(data);
    }
    for (const auto &frame : frames) {
      // Only handle this protocol's REQUESTS: ignore frames for other modules
      // and reply-flagged frames (the service answers requests; a reply-typed
      // frame — e.g. an echo/loopback — is never a host request).
      if (frame.module != kModule || frame.is_reply())
        continue;
      std::vector<uint8_t> reply;
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!handle_frame_locked(frame.type, frame.payload, reply))
          continue;
      }
      // send outside the lock so a re-entrant transport cannot deadlock
      if (!reply.empty())
        send(reply);
    }
  }

  /**
   * @brief Handle one already-parsed frame.
   * @param type The frame type byte.
   * @param payload The frame payload bytes.
   * @return true if the frame type belongs to the core-dump protocol and the
   *         frame was processed (a reply frame is produced; it is delivered
   *         only when a `send` callback is configured, and dropped with a
   *         warning otherwise), false if it was ignored (another protocol's
   *         frame — nothing is produced, so multiple services can share one
   *         stream).
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
    namespace stream = espp::stream_frame;
    switch (static_cast<Msg>(type)) {
    case Msg::GetSummary: {
      const std::string report = core_dump_.format_report();
      logger_.info("GET_SUMMARY -> {} bytes", report.size());
      // truncate to the frame payload cap (reports are far smaller in practice),
      // backing off to a UTF-8 codepoint boundary so a split multi-byte
      // sequence cannot produce invalid UTF-8 in the SUMMARY payload
      // (continuation bytes are 0b10xxxxxx = 0x80..0xBF).
      size_t count = std::min(report.size(), stream::kMaxPayloadSize);
      while (count > 0 && count < report.size() &&
             (static_cast<uint8_t>(report[count]) & 0xC0) == 0x80) {
        --count;
      }
      reply =
          build(Msg::Summary,
                std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(report.data()), count));
      return true;
    }
    case Msg::GetSize: {
      // The wire SIZE field is u32. Core dumps are far smaller, but clamp
      // defensively so an oversized/misreported image_size() cannot silently
      // wrap when narrowed.
      const size_t raw_size = core_dump_.image_size();
      const auto size =
          static_cast<uint32_t>(std::min<size_t>(raw_size, std::numeric_limits<uint32_t>::max()));
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
    namespace stream = espp::stream_frame;
    // Reply message types (Summary/Size/Data/Ok/Error) carry the high bit; map
    // it to the frame reply flag so requests and replies are distinguishable
    // independent of the type value.
    const bool reply = (static_cast<uint8_t>(type) & 0x80) != 0;
    return stream::build_frame(reply, kModule, static_cast<uint8_t>(type), payload);
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

  /// Build an ERROR reply frame (u32 code + UTF-8 context message). Codes
  /// from categories other than the generic category are normalized to the
  /// equivalent std::errc via default_error_condition() — falling back to
  /// std::errc::io_error when there is no generic equivalent — so the on-wire
  /// code is always a std::errc value (the message text still carries the
  /// original category's description). Note that the code is informational /
  /// best-effort: std::errc numbering is not standardized across C++ stdlibs
  /// (it is stable for espp's newlib/ESP-IDF toolchain), so the message is
  /// the authoritative description and hosts should display code + message
  /// without interpreting specific code values (see the wire spec above).
  std::vector<uint8_t> build_error(const std::error_code &ec, std::string_view context) const {
    namespace stream = espp::stream_frame;
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
    // Truncate to the remaining payload cap, backing off to a UTF-8 codepoint
    // boundary (as GET_SUMMARY does) so a split multi-byte sequence cannot
    // produce invalid UTF-8 on the wire (continuation bytes are 0b10xxxxxx =
    // 0x80..0xBF).
    size_t count = std::min(message.size(), stream::kMaxPayloadSize - payload.size());
    while (count > 0 && count < message.size() &&
           (static_cast<uint8_t>(message[count]) & 0xC0) == 0x80) {
      --count;
    }
    payload.insert(payload.end(), message.begin(), message.begin() + count);
    return build(Msg::Error, payload);
  }

  /// Overload taking a std::errc directly.
  std::vector<uint8_t> build_error(std::errc errc, std::string_view context) const {
    return build_error(std::make_error_code(errc), context);
  }

private:
  CoreDump &core_dump_;
  send_fn send_;
  std::mutex mutex_;
  Stream parser_;
};

} // namespace espp

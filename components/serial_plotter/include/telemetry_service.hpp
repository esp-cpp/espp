#pragma once

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "base_component.hpp"
#include "stream_frame.hpp"

namespace espp {
/// @brief Binary telemetry emitter for the Serial Plotter web app.
///
/// A tiny device->host protocol carried over the espp `stream_frame` framing
/// (so it can share one USB vendor / CDC stream with other modules via
/// `espp::Dispatcher`). Firmware declares a fixed set of named float channels
/// (the SCHEMA) and pushes SAMPLE frames — a device timestamp plus one float
/// per channel — which the hosted `serial_plotter.html` web app decodes and
/// plots, exactly like the columnar Web-Serial path but binary, higher rate,
/// and with device-accurate timestamps.
///
/// This is the purpose-built counterpart to the app's text (CSV-style)
/// Web-Serial transport: instead of parsing printed columns, the device sends
/// typed samples directly.
///
/// ## Wire protocol (dispatcher module id 3)
///
/// Every message is a `stream_frame` frame with `module == kModule`. The frame
/// `type` byte's high bit distinguishes direction: host->device requests are
/// `0x0X`, device->host frames are `0x8X` (which sets the frame reply flag).
///
/// - **GET_SCHEMA** (host->device, no payload): request the current SCHEMA.
/// - **SET_STREAM** (host->device, `[enabled u8][period_ms u16]`): enable or
///   disable streaming and request a sample period (informational — the
///   firmware's emit cadence is authoritative; the requested period is exposed
///   via period_ms() so an app can honor it). Answered with OK.
/// - **SCHEMA** (device->host): `[version u8][flags u8][nchannels u8]` then, per
///   channel, `[type u8][name_len u8][name bytes]`. `version == kSchemaVersion`,
///   `flags == 0` (reserved), channel `type == 0` (f32). Sent on GET_SCHEMA, on
///   set_channels(), and on demand via send_schema().
/// - **SAMPLE** (device->host): one or more packed records, each
///   `[timestamp u32 microseconds][f32 × nchannels]` little-endian. A frame may
///   batch several records (payload size is an exact multiple of the record
///   size) for higher throughput.
/// - **OK** (device->host, `[request_type u8]`): acknowledges a request.
/// - **ERROR** (device->host, `[request_type u8][code u32][utf8 message]`).
///
/// ## Threading
///
/// emit() is typically called from a producer task while requests are handled
/// on a transport RX task; both are safe to call concurrently. Frames are built
/// under one mutex and transmitted under a separate send mutex, so the user
/// `send` callback is (a) never invoked while the build mutex is held — a
/// re-entrant transport cannot deadlock — and (b) never invoked concurrently, so
/// a `send` that is not itself thread-safe still cannot interleave the bytes of
/// two frames.
class Telemetry : public espp::BaseComponent {
public:
  /// Dispatcher module id owned by the telemetry protocol (the frame `module`
  /// byte). Device->host Type values keep the high bit set, which the framing
  /// maps to the reply flag.
  static constexpr uint8_t kModule = 3;

  /// Version byte at the head of a SCHEMA payload, so the wire format can evolve.
  static constexpr uint8_t kSchemaVersion = 1;

  /// Maximum channel count (the SCHEMA encodes it as a u8).
  static constexpr size_t kMaxChannels = 255;

  /// Frame `type` values within the telemetry module.
  enum class Type : uint8_t {
    // host -> device
    GetSchema = 0x01, ///< request the current SCHEMA
    SetStream = 0x02, ///< [enabled u8][period_ms u16]: enable/disable + rate
    // device -> host (high bit set)
    Schema = 0x81, ///< channel schema (see class docs)
    Sample = 0x82, ///< one or more [timestamp u32 us][f32 x nchannels] records
    Ok = 0x83,     ///< [request_type u8]: request acknowledged
    Error = 0x84,  ///< [request_type u8][code u32][utf8 message]
  };

  /// Channel value type (only 32-bit float today; reserved for future widening).
  enum class ChannelType : uint8_t { F32 = 0 };

  /// @brief Function used to transmit one encoded frame to the host.
  /// @param frame The complete encoded frame bytes (header + payload + CRC).
  using send_fn = std::function<void(std::span<const uint8_t> frame)>;

  /// Configuration for the Telemetry emitter.
  struct Config {
    std::vector<std::string> channels; ///< Channel names, in sample order (>= 1).
    send_fn send{nullptr};             ///< Transmits an encoded frame (may be set later).
    bool stream_on_start{true};        ///< Start with streaming enabled.
    uint16_t period_ms{20};            ///< Default requested sample period (informational).
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Logger verbosity.
  };

  /// @brief Construct the emitter.
  /// @param config Channel names, the (optional) send function, and defaults.
  explicit Telemetry(const Config &config)
      : BaseComponent("Telemetry", config.log_level)
      , channels_(config.channels)
      , send_(config.send)
      , streaming_(config.stream_on_start)
      , period_ms_(config.period_ms) {
    clamp_channels();
  }

  /// @brief Set (or replace) the transmit function, e.g. after USB init.
  void set_send(send_fn fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    send_ = std::move(fn);
  }

  /// @brief Push one sample with an explicit device timestamp.
  /// @param values One value per channel, in schema order (size must match the
  ///        channel count, else the sample is dropped with a rate-limited warning).
  /// @param timestamp_us Device timestamp in microseconds (u32; wraps ~71 min).
  /// @note No-op while streaming is disabled or no send function is configured.
  void emit(std::span<const float> values, uint32_t timestamp_us) {
    if (!streaming_.load())
      return;
    // Hold send_mutex_ across BOTH building this sample (which reads the current
    // channel count) and sending it. That makes the sample's width and its
    // delivery atomic with respect to set_channels()/send_schema() (which take
    // the same outer lock), so a new SCHEMA can never be interleaved between a
    // sample built from one channel set and its transmission - which would make
    // the host decode the sample at the wrong record width. Lock order is always
    // send_mutex_ (outer) -> mutex_ (inner); no sender ever takes them the other
    // way, so this cannot deadlock.
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    std::vector<uint8_t> frame;
    send_fn s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (values.size() != channels_.size()) {
        logger_.warn_rate_limited("emit(): {} values for {} channels; dropping", values.size(),
                                  channels_.size());
        return;
      }
      if (!send_)
        return;
      s = send_;
      std::vector<uint8_t> p;
      p.reserve(4 + 4 * values.size());
      espp::stream_frame::put_u32(p, timestamp_us);
      for (float v : values)
        put_f32(p, v);
      frame = build(Type::Sample, p);
    }
    s(std::span<const uint8_t>(frame)); // send while still holding send_mutex_
  }

  /// @brief Push one sample timestamped with the current device time.
  void emit(std::span<const float> values) { emit(values, now_us()); }

  /// @brief Redefine the channel set at runtime and send a fresh SCHEMA.
  /// @note The channel-set update and the outbound SCHEMA are performed together
  ///       under send_mutex_, so a concurrent emit() cannot deliver a sample
  ///       built from the new channels before (or the old channels after) the
  ///       matching SCHEMA reaches the host.
  void set_channels(std::vector<std::string> channels) {
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    std::vector<uint8_t> frame;
    send_fn s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      channels_ = std::move(channels);
      clamp_channels();
      if (!send_)
        return; // no transport yet; the next GetSchema will carry the new set
      s = send_;
      frame = build(Type::Schema, build_schema_payload_locked());
    }
    s(std::span<const uint8_t>(frame)); // send while still holding send_mutex_
  }

  /// @brief The current channel names (schema order).
  std::vector<std::string> channels() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return channels_;
  }

  /// @brief Whether streaming is currently enabled.
  bool streaming() const { return streaming_.load(); }

  /// @brief Enable or disable streaming (SAMPLE emission).
  void set_streaming(bool on) { streaming_.store(on); }

  /// @brief The host-requested sample period in milliseconds (informational).
  uint16_t period_ms() const { return period_ms_.load(); }

  /// @brief Send the current SCHEMA frame now (device->host).
  /// @note Uses the same send_mutex_-outer / mutex_-inner order as emit() so the
  ///       SCHEMA and any concurrent SAMPLE stay consistently ordered.
  void send_schema() const {
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    std::vector<uint8_t> frame;
    send_fn s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!send_)
        return;
      s = send_;
      frame = build(Type::Schema, build_schema_payload_locked());
    }
    s(std::span<const uint8_t>(frame)); // send while still holding send_mutex_
  }

  /// @brief Dispatcher handler: process one frame addressed to this module.
  ///
  /// Register with `dispatcher.register_module(Telemetry::kModule, ...)`. Ignores
  /// reply-flagged frames (device->host pushes are never host requests).
  void handle(const espp::stream_frame::Frame &frame) {
    if (frame.is_reply())
      return;
    handle_request(frame.type, frame.payload);
  }

  /// @brief Feed raw transport bytes through an internal frame parser (for use
  ///        without a Dispatcher). Processes every complete frame for this module.
  void feed(std::span<const uint8_t> data) {
    std::vector<espp::stream_frame::Frame> frames;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      frames = parser_.feed(data);
    }
    for (const auto &frame : frames) {
      if (frame.module != kModule || frame.is_reply())
        continue;
      handle_request(frame.type, frame.payload);
    }
  }

  /// @brief Discard any partially-buffered bytes in the internal parser (e.g.
  ///        on transport reconnect). Only relevant when using feed().
  void reset_parser() {
    std::lock_guard<std::mutex> lock(mutex_);
    parser_.reset();
  }

protected:
  /// Handle one request `type` + `payload`; builds any reply under the lock and
  /// transmits it with the lock released.
  void handle_request(uint8_t type, std::span<const uint8_t> payload) {
    switch (static_cast<Type>(type)) {
    case Type::GetSchema:
      send_schema();
      break;
    case Type::SetStream: {
      if (payload.size() < 3) {
        send_error(type, "SET_STREAM payload too short");
        return;
      }
      const bool enabled = payload[0] != 0;
      const uint16_t period =
          static_cast<uint16_t>(payload[1] | (static_cast<uint16_t>(payload[2]) << 8));
      if (period != 0)
        period_ms_.store(period);
      streaming_.store(enabled);
      logger_.debug("SET_STREAM enabled={} period_ms={}", enabled, period_ms_.load());
      send_ok(type);
      break;
    }
    default:
      // Unknown telemetry type: report it (the module id already matched).
      send_error(type, "unknown telemetry message");
      break;
    }
  }

  /// Serialize the SCHEMA payload. Caller must hold `mutex_`. The channel count
  /// is a u8; channels_ is capped at kMaxChannels (see clamp_channels), but the
  /// count and the serialized channels are derived from the same bound so the
  /// payload is always self-consistent.
  std::vector<uint8_t> build_schema_payload_locked() const {
    const size_t n = std::min<size_t>(channels_.size(), kMaxChannels);
    std::vector<uint8_t> p;
    p.push_back(kSchemaVersion);
    p.push_back(0); // flags (reserved)
    p.push_back(static_cast<uint8_t>(n));
    for (size_t i = 0; i < n; i++) {
      const auto &name = channels_[i];
      p.push_back(static_cast<uint8_t>(ChannelType::F32));
      const uint8_t len = static_cast<uint8_t>(std::min<size_t>(name.size(), 255));
      p.push_back(len);
      p.insert(p.end(), name.begin(), name.begin() + len);
    }
    return p;
  }

  /// Truncate channels_ to the u8 SCHEMA limit (caller holds mutex_, or is the
  /// constructor). Warns when channels are dropped.
  void clamp_channels() {
    if (channels_.size() > kMaxChannels) {
      logger_.warn("{} channels exceeds the {}-channel SCHEMA limit; truncating", channels_.size(),
                   kMaxChannels);
      channels_.resize(kMaxChannels);
    }
  }

  void send_ok(uint8_t request_type) const {
    const uint8_t p[] = {request_type};
    send_frame(build(Type::Ok, p));
  }

  void send_error(uint8_t request_type, std::string_view message) const {
    logger_.warn("{} (type 0x{:02x})", message, request_type);
    std::vector<uint8_t> p;
    p.push_back(request_type);
    espp::stream_frame::put_u32(p, 0); // reserved code
    p.insert(p.end(), message.begin(), message.end());
    send_frame(build(Type::Error, p));
  }

  /// Transmit an already-built frame via the configured send function. Takes
  /// send_mutex_ (outer) then mutex_ (inner) - the same order as emit() and
  /// send_schema() - so frames' bytes never interleave and no lock-order
  /// inversion is possible. Never called while mutex_ is already held.
  void send_frame(const std::vector<uint8_t> &frame) const {
    std::lock_guard<std::mutex> send_lock(send_mutex_);
    send_fn s;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      s = send_;
    }
    if (s)
      s(std::span<const uint8_t>(frame)); // send while still holding send_mutex_
  }

  /// Build an encoded frame for a telemetry message type. Device->host types
  /// (high bit set) map to the frame reply flag.
  static std::vector<uint8_t> build(Type type, std::span<const uint8_t> payload = {}) {
    const bool reply = (static_cast<uint8_t>(type) & 0x80) != 0;
    return espp::stream_frame::build_frame(reply, kModule, static_cast<uint8_t>(type), payload);
  }

  /// Append a little-endian IEEE-754 float32 to a byte buffer.
  static void put_f32(std::vector<uint8_t> &out, float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    espp::stream_frame::put_u32(out, bits);
  }

  /// Current device time in microseconds (truncated to u32; wraps ~71 min).
  static uint32_t now_us() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
  }

  mutable std::mutex mutex_;      ///< guards channels_, send_, and the parser
  mutable std::mutex send_mutex_; ///< outer lock: serializes send callbacks and
                                  ///< keeps each frame's build+send atomic (order:
                                  ///< send_mutex_ then mutex_)
  std::vector<std::string> channels_;
  send_fn send_;
  std::atomic<bool> streaming_;
  std::atomic<uint16_t> period_ms_;
  espp::stream_frame::StreamParser parser_;
};
} // namespace espp

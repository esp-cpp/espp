#pragma once

// Wheelchair Digital Interface (WDI) — the **host** role.
//
// WdiHost is the wheelchair side of the interface: it receives Control reports
// from the app / accessory and sends Feedback back, and it owns the host-side
// keepalive **watchdog** from the spec (if the app stops sending, the host
// disconnects and drive-disables). It is the mirror image of WdiDevice.
//
// Like WdiDevice it is transport-agnostic and depends only on the C++20 standard
// library and the WDI protocol core (detail/wdi_protocol.hpp): you give it a
// `send` callback that puts an OUTPUT report on the wire (USB HID SET_REPORT or a
// BLE write) and feed it the app's INPUT reports via handle_input(). It does NOT
// own a timer — call poll() periodically and it fires the disconnect callback
// when the app has gone quiet for too long. Time is read through a
// caller-supplied clock (default: a steady ms clock) so it is host-testable.

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <optional>
#include <span>
#include <utility>

#include "detail/wdi_protocol.hpp"

namespace espp {

/// @brief The WDI **host** role (the wheelchair receiving Control, sending Feedback).
class WdiHost {
public:
  /// @brief Transmit an OUTPUT report to the app. `id` is the report id;
  ///        `payload` is the report body (no report-id byte). Return true if it
  ///        was sent. The transport binding maps this to a USB HID Output report
  ///        (SET_REPORT) or a BLE write.
  using send_fn = std::function<bool(wdi::ReportId id, std::span<const uint8_t> payload)>;
  /// @brief Invoked when a Control (0x01) report arrives from the app. The
  ///        wheelchair should act on it (or, on a release / disconnect, stop).
  using control_fn = std::function<void(const wdi::ControlReport &)>;
  /// @brief Supplies the current Feedback to send (on Request-Feedback or
  ///        send_feedback()). If unset, the last value from set_feedback() is used.
  using feedback_provider_fn = std::function<wdi::FeedbackReport()>;
  /// @brief Link state change (connected when the app is talking; disconnected
  ///        when the keepalive watchdog expires).
  using link_fn = std::function<void()>;
  /// @brief Monotonic clock in milliseconds.
  using clock_fn = std::function<uint32_t()>;

  struct Config {
    send_fn send;                           ///< REQUIRED: put an OUTPUT report on the wire
    control_fn on_control{nullptr};         ///< called with each Control report
    feedback_provider_fn feedback{nullptr}; ///< current Feedback to report (optional)
    link_fn on_connected{nullptr};          ///< the app started talking
    link_fn on_disconnected{nullptr};       ///< the watchdog expired (drive-disable!)
    /// @brief The host's 128-bit identity, returned in Keepalive Responses. Set at
    ///        least the manufacturer id (see make_host_uuid()).
    wdi::HostUuid host_uuid{};
    /// @brief Per-window timeout (ms). The app sends every ~233 ms; the host's
    ///        window is 257 ms.
    uint32_t keepalive_window_ms{wdi::kHostKeepaliveWindowMs};
    /// @brief Consecutive missed windows before disconnect + drive-disable (spec: 3).
    uint32_t missed_windows_to_disconnect{wdi::kHostMissedWindowsToDisconnect};
    /// @brief Monotonic ms clock; defaults to std::chrono::steady_clock. Inject a
    ///        fake clock in tests.
    clock_fn now_ms{nullptr};
  };

  explicit WdiHost(Config config)
      : config_(std::move(config)) {
    if (!config_.now_ms)
      config_.now_ms = default_clock;
    last_rx_ms_ = config_.now_ms();
  }

  /// @brief Build a Host UUID from a manufacturer id and 14 random bytes (the
  ///        spec's RFC-4122 v4 layout). The manufacturer id is stored big-endian
  ///        in bytes 0..1; the version / variant nibbles are set on the random
  ///        part. Pass your own randomness (e.g. esp_fill_random / a PRNG).
  static wdi::HostUuid make_host_uuid(uint16_t manufacturer_id, std::span<const uint8_t> random14) {
    wdi::HostUuid u;
    u.bytes[0] = static_cast<uint8_t>((manufacturer_id >> 8) & 0xFF);
    u.bytes[1] = static_cast<uint8_t>(manufacturer_id & 0xFF);
    for (size_t i = 0; i < 14 && i < random14.size(); ++i)
      u.bytes[2 + i] = random14[i];
    // RFC 4122 v4: version nibble in byte 6 (spec's byte index 6), variant in byte 8.
    u.bytes[6] = static_cast<uint8_t>((u.bytes[6] & 0x0F) | 0x40);
    u.bytes[8] = static_cast<uint8_t>((u.bytes[8] & 0x3F) | 0x80);
    return u;
  }

  // --- app -> host (received INPUT reports) ----------------------------------

  /// @brief Feed a received INPUT report (app→host): Control (0x01),
  ///        Request-Feedback (0x03) or Keepalive (0x04). Any of them refreshes
  ///        the watchdog and marks the link connected. Request-Feedback triggers
  ///        a Feedback reply; Keepalive triggers a Keepalive-Response reply.
  void handle_input(wdi::ReportId id, std::span<const uint8_t> payload) {
    switch (id) {
    case wdi::ReportId::Control:
      if (auto c = wdi::ControlReport::parse(payload)) {
        {
          std::lock_guard<std::mutex> lk(state_mutex_);
          last_control_ = *c;
        }
        mark_activity();
        if (config_.on_control)
          config_.on_control(*c);
      }
      break;
    case wdi::ReportId::RequestFeedback:
      mark_activity();
      send_feedback();
      break;
    case wdi::ReportId::Keepalive:
      mark_activity();
      send_keepalive_response();
      break;
    default:
      break; // not an app→host report; ignore
    }
  }

  // --- host -> app (feedback + identity) -------------------------------------

  /// @brief Update the Feedback the host reports (used when no feedback provider
  ///        is configured, and as the value sent by send_feedback()).
  void set_feedback(const wdi::FeedbackReport &fb) {
    std::lock_guard<std::mutex> lk(state_mutex_);
    feedback_ = fb;
  }

  /// @brief Send a Feedback report now (host→app). Returns true if sent.
  bool send_feedback() {
    wdi::FeedbackReport fb;
    if (config_.feedback) {
      fb = config_.feedback();
    } else {
      std::lock_guard<std::mutex> lk(state_mutex_);
      fb = feedback_;
    }
    const auto bytes = fb.serialize();
    return transmit(wdi::ReportId::Feedback, bytes);
  }

  /// @brief Send a Keepalive Response (the host's UUID) now. Returns true if sent.
  bool send_keepalive_response() {
    const auto bytes = config_.host_uuid.serialize();
    return transmit(wdi::ReportId::KeepaliveResponse, bytes);
  }

  // --- watchdog --------------------------------------------------------------

  /// @brief Check the keepalive watchdog; call periodically. If the app has been
  ///        quiet for `missed_windows_to_disconnect` windows, the link is marked
  ///        disconnected (fire on_disconnected — the caller must drive-disable).
  ///        Returns true if a disconnect transition happened this call.
  bool poll() {
    if (!connected_.load())
      return false;
    const uint32_t now = config_.now_ms();
    const uint32_t timeout = config_.keepalive_window_ms * config_.missed_windows_to_disconnect;
    if (now - last_rx_ms_.load() >= timeout) {
      connected_.store(false);
      if (config_.on_disconnected)
        config_.on_disconnected();
      return true;
    }
    return false;
  }

  /// @brief Whether the app is currently considered connected (talking).
  bool is_connected() const { return connected_.load(); }
  /// @brief Milliseconds until the watchdog expires (0 if already expired / down).
  uint32_t ms_until_timeout() const {
    if (!connected_.load())
      return 0;
    const uint32_t timeout = config_.keepalive_window_ms * config_.missed_windows_to_disconnect;
    const uint32_t elapsed = config_.now_ms() - last_rx_ms_.load();
    return elapsed >= timeout ? 0 : timeout - elapsed;
  }
  /// @brief The most recently received Control report, if any.
  std::optional<wdi::ControlReport> last_control() const {
    std::lock_guard<std::mutex> lk(state_mutex_);
    return last_control_;
  }

private:
  static uint32_t default_clock() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
  }

  void mark_activity() {
    last_rx_ms_.store(config_.now_ms());
    bool was = false;
    if (connected_.compare_exchange_strong(was, true)) {
      if (config_.on_connected)
        config_.on_connected();
    }
  }

  bool transmit(wdi::ReportId id, std::span<const uint8_t> payload) {
    if (!config_.send)
      return false;
    return config_.send(id, payload);
  }

  Config config_;
  // last_rx_ms_ / connected_ are written by handle_input() (transport RX task)
  // and read by poll() (watchdog task); atomic so the two are race-free.
  // feedback_ / last_control_ are guarded by state_mutex_ (written on one task,
  // read on another).
  std::atomic<uint32_t> last_rx_ms_{0};
  std::atomic<bool> connected_{false};
  mutable std::mutex state_mutex_;
  wdi::FeedbackReport feedback_{};
  std::optional<wdi::ControlReport> last_control_{};
};

} // namespace espp

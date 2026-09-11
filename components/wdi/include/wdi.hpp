#pragma once

// Wheelchair Digital Interface (WDI) — the **device** role.
//
// WdiDevice is the app / accessory side of the interface (the thing that drives
// the wheelchair): special switches, an alternative joystick, a phone app, a
// companion MCU. It is transport-agnostic — you give it a `send` callback that
// puts a report on the wire (USB HID or BLE), and you feed it the reports the
// host sends back via handle_output(). It owns the keepalive state machine from
// the spec.
//
// The class depends only on the C++20 standard library and the WDI protocol core
// (detail/wdi_protocol.hpp), so it is unit-testable on a host. It does NOT own a
// timer: call poll() periodically (from an espp::Timer / Task on device, or a
// test loop) and it emits a keepalive when one is due. Time is read through a
// caller-supplied clock (defaulting to a steady millisecond clock) so tests can
// drive it deterministically.

#include <chrono>
#include <cstdint>
#include <functional>
#include <optional>
#include <span>
#include <utility>

#include "detail/wdi_protocol.hpp"

namespace espp {

/// @brief The WDI **device** role (app / accessory driving the wheelchair).
class WdiDevice {
public:
  /// @brief Transmit a report to the host. `id` is the report id; `payload` is the
  ///        report body (no report-id byte). Return true if it was sent. The
  ///        transport binding maps this to a USB HID Input report or a BLE notify.
  using send_fn = std::function<bool(wdi::ReportId id, std::span<const uint8_t> payload)>;
  /// @brief Invoked when a Feedback (0x02) report arrives from the host.
  using feedback_fn = std::function<void(const wdi::FeedbackReport &)>;
  /// @brief Invoked when a Keepalive Response (0x05) arrives (the host's UUID).
  using host_uuid_fn = std::function<void(const wdi::HostUuid &)>;
  /// @brief Monotonic clock in milliseconds.
  using clock_fn = std::function<uint32_t()>;

  struct Config {
    send_fn send;                                ///< REQUIRED: put a report on the wire
    feedback_fn on_feedback{nullptr};            ///< called with each Feedback report
    host_uuid_fn on_keepalive_response{nullptr}; ///< called with each Keepalive Response
    /// @brief Keepalive send interval (ms). The spec's app sends every ~233 ms
    ///        (24 ms margin before the host's 257 ms window); sending Control or
    ///        Request-Feedback also resets the timer.
    uint32_t keepalive_interval_ms{wdi::kAppKeepaliveIntervalMs};
    /// @brief Monotonic ms clock; defaults to std::chrono::steady_clock (portable,
    ///        works on device and host). Inject a fake clock in tests.
    clock_fn now_ms{nullptr};
  };

  explicit WdiDevice(Config config)
      : config_(std::move(config)) {
    if (!config_.now_ms)
      config_.now_ms = default_clock;
    // Initialize so the first poll() emits a keepalive promptly (kickstart).
    last_tx_ms_ = config_.now_ms() - config_.keepalive_interval_ms;
  }

  // --- app -> host (the accessory's controls) --------------------------------

  /// @brief Send a Control report (joystick + flags). Resets the keepalive timer.
  bool send_control(const wdi::ControlReport &control) {
    const auto bytes = control.serialize();
    return transmit(wdi::ReportId::Control, bytes);
  }

  /// @brief Send an all-zero "release" Control report (neutral joystick, no flags).
  bool send_release() { return send_control(wdi::ControlReport{}); }

  /// @brief Ask the host to send a Feedback report. Resets the keepalive timer.
  bool request_feedback() {
    const uint8_t b = wdi::kTriggerValue;
    return transmit(wdi::ReportId::RequestFeedback, {&b, 1});
  }

  /// @brief Send a Keepalive heartbeat (normally emitted automatically by poll()).
  bool send_keepalive() {
    const uint8_t b = wdi::kTriggerValue;
    return transmit(wdi::ReportId::Keepalive, {&b, 1});
  }

  /// @brief Emit a keepalive if the interval has elapsed since the last transmit.
  ///        Call this periodically (e.g. from an espp::Timer or Task). Returns
  ///        true if a keepalive was actually sent this call.
  bool poll() {
    const uint32_t now = config_.now_ms();
    // Unsigned subtraction is correct across wraparound for intervals < 2^31 ms.
    if (now - last_tx_ms_ >= config_.keepalive_interval_ms)
      return send_keepalive();
    return false;
  }

  /// @brief Milliseconds until the next keepalive is due (0 if due now).
  uint32_t ms_until_keepalive() const {
    const uint32_t elapsed = config_.now_ms() - last_tx_ms_;
    return elapsed >= config_.keepalive_interval_ms ? 0 : config_.keepalive_interval_ms - elapsed;
  }

  // --- host -> device (feedback + identity) ----------------------------------

  /// @brief Feed a received OUTPUT report (host→device): Feedback (0x02) or
  ///        Keepalive Response (0x05). Other ids are ignored. The transport
  ///        binding calls this from its HID SET_REPORT / BLE write handler.
  void handle_output(wdi::ReportId id, std::span<const uint8_t> payload) {
    switch (id) {
    case wdi::ReportId::Feedback:
      if (auto fb = wdi::FeedbackReport::parse(payload)) {
        last_feedback_ = *fb;
        if (config_.on_feedback)
          config_.on_feedback(*fb);
      }
      break;
    case wdi::ReportId::KeepaliveResponse:
      if (auto uuid = wdi::HostUuid::parse(payload)) {
        host_uuid_ = *uuid;
        if (config_.on_keepalive_response)
          config_.on_keepalive_response(*uuid);
      }
      break;
    default:
      break; // not a host→device report; ignore
    }
  }

  /// @brief The host's identity from the most recent Keepalive Response, if any.
  std::optional<wdi::HostUuid> host_uuid() const { return host_uuid_; }
  /// @brief The most recently received Feedback report, if any.
  std::optional<wdi::FeedbackReport> last_feedback() const { return last_feedback_; }

private:
  static uint32_t default_clock() {
    using namespace std::chrono;
    return static_cast<uint32_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
  }

  bool transmit(wdi::ReportId id, std::span<const uint8_t> payload) {
    if (!config_.send)
      return false;
    const bool ok = config_.send(id, payload);
    // Per spec, Control / Request-Feedback / Keepalive all reset the app's
    // keepalive timer -- every transmit path routes through here, so reset on any
    // successful send.
    if (ok)
      last_tx_ms_ = config_.now_ms();
    return ok;
  }

  Config config_;
  uint32_t last_tx_ms_{0};
  std::optional<wdi::HostUuid> host_uuid_{};
  std::optional<wdi::FeedbackReport> last_feedback_{};
};

} // namespace espp

#pragma once

// WDI (Wheelchair Digital Interface) USB **host** — the wheelchair role over USB.
// Wraps the transport-agnostic espp::WdiHost with an espp::UsbHost (USB Host HID):
// it enumerates an attached WDI HID device (an accessory / app running e.g.
// espp::WdiUsbPeripheral) and speaks the host side of the protocol to it.
//
// The app's Control / Request-Feedback / Keepalive are HID **Input** reports
// (device->host, delivered by UsbHost's per-device input callback); the host's
// Feedback / Keepalive-Response are HID **Output** reports (host->device, sent
// with HidDevice::send_output_report()). The report logic + keepalive watchdog
// live in WdiHost (host-tested).
//
// Threading: UsbHost delivers its callbacks on its own dispatch task, so the
// Output-report replies WdiHost makes from inside the input path are ordinary
// control transfers that complete normally. The wrapper's mutex only guards its
// pointers; the WdiHost core (itself thread-safe) is always invoked with the
// mutex released, so user callbacks (on_control / on_disconnected / ...) may
// freely call back into this object.
//
// Only one WDI device is tracked at a time (a wheelchair has one active
// accessory link). Usage: construct, initialize(), set_feedback() as the chair's
// status changes, and call poll() periodically so the watchdog can drive-disable
// if the accessory goes quiet. on_disconnected may fire twice for one link loss
// (watchdog, then the USB detach); it is idempotent for its purpose.

#include <algorithm>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <system_error>

#include "base_component.hpp"
#include "usb_host.hpp"

#include "wdi_hid.hpp"
#include "wdi_host.hpp"

namespace espp {

/// @brief The WDI host role over USB (a USB host talking to a WDI HID device).
class WdiUsbHost : public BaseComponent {
public:
  struct Config {
    WdiHost::control_fn on_control{nullptr};         ///< a Control report arrived
    WdiHost::feedback_provider_fn feedback{nullptr}; ///< current Feedback to report
    WdiHost::link_fn on_connected{nullptr};          ///< a WDI accessory link came up
    WdiHost::link_fn on_disconnected{nullptr};       ///< the link dropped / watchdog fired
    wdi::HostUuid host_uuid{}; ///< the host's identity (see WdiHost::make_host_uuid)
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  explicit WdiUsbHost(const Config &config)
      : BaseComponent("WdiUsbHost", config.log_level)
      , config_(config)
      , usb_(make_usb_config(config)) {}

  /// @brief Install the USB host stack and start looking for a WDI device.
  bool initialize(std::error_code &ec) { return usb_.initialize(ec); }

  /// @brief Update the Feedback reported to the accessory (host->device).
  void set_feedback(const wdi::FeedbackReport &fb) {
    std::shared_ptr<WdiHost> h;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      feedback_ = fb;
      h = host_;
    }
    if (h)
      h->set_feedback(fb);
  }

  /// @brief Send a Feedback report now (if a device is connected).
  // Not const: it has side effects through the WdiHost core (sends a report /
  // fires user callbacks) even though it only reads this wrapper.
  // cppcheck-suppress functionConst
  bool send_feedback() {
    auto h = host();
    return h ? h->send_feedback() : false;
  }

  /// @brief Run the keepalive watchdog; call periodically (e.g. from a Timer).
  ///        Fires on_disconnected if the accessory has gone quiet too long.
  // Not const: it has side effects through the WdiHost core (sends a report /
  // fires user callbacks) even though it only reads this wrapper.
  // cppcheck-suppress functionConst
  bool poll() {
    auto h = host();
    return h ? h->poll() : false;
  }

  /// @brief Whether a WDI accessory is currently connected and talking.
  bool is_connected() const {
    auto h = host();
    return h && h->is_connected();
  }

  /// @brief The most recent Control report, if any.
  std::optional<wdi::ControlReport> last_control() const {
    auto h = host();
    return h ? h->last_control() : std::nullopt;
  }

  /// @brief Access the underlying USB host (e.g. to enumerate all HID devices).
  UsbHost &usb() { return usb_; }

  /// @brief Does a HID report descriptor describe a WDI device? Exact match
  ///        against the descriptor this component emits, or -- for another
  ///        implementation of the spec -- an application collection on the WDI
  ///        vendor usage page (0xFF00) with usage 0x01 that declares report ids
  ///        1..5. Walks the descriptor's short items rather than byte-scanning,
  ///        so item *data* (e.g. a Logical Maximum of 0x00FF0006) cannot
  ///        masquerade as a Usage Page item.
  static bool looks_like_wdi(std::span<const uint8_t> d) {
    if (d.size() == wdi::kReportDescriptor.size() &&
        std::equal(d.begin(), d.end(), wdi::kReportDescriptor.begin()))
      return true;
    bool vendor_page = false; // saw Usage Page 0xFF00 immediately followed by Usage 0x01
    uint8_t report_ids = 0;   // bit i-1 set when Report ID i (1..5) was seen
    bool prev_was_wdi_page = false;
    for (size_t i = 0; i < d.size();) {
      const uint8_t prefix = d[i];
      if (prefix == 0xFE) // long item: skip (bDataSize in the next byte)
        return false;     // not something a WDI descriptor contains
      const uint8_t size_code = prefix & 0x03;
      const size_t size = size_code == 3 ? 4 : size_code;
      if (i + 1 + size > d.size())
        return false; // malformed
      const uint8_t tag_type = prefix & 0xFC;
      const uint8_t *data = &d[i + 1];
      if (tag_type == 0x04 && size == 2 && data[0] == 0x00 && data[1] == 0xFF) {
        prev_was_wdi_page = true; // Global: Usage Page 0xFF00
      } else {
        if (tag_type == 0x08 && size == 1 && data[0] == 0x01 && prev_was_wdi_page)
          vendor_page = true; // Local: Usage 0x01 (Wheelchair Control Device)
        prev_was_wdi_page = false;
      }
      if (tag_type == 0x84 && size == 1 && data[0] >= 1 && data[0] <= 5) // Global: Report ID
        report_ids |= static_cast<uint8_t>(1u << (data[0] - 1));
      i += 1 + size;
    }
    return vendor_page && report_ids == 0x1F;
  }

private:
  std::shared_ptr<WdiHost> host() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_;
  }

  UsbHost::Config make_usb_config(const Config &c) {
    UsbHost::Config uc;
    uc.log_level = c.log_level;
    uc.auto_start = true;
    // The filter sees only info/params (not the descriptor), so accept all here
    // and confirm via the descriptor on connect.
    uc.on_device_connected = [this](const std::shared_ptr<UsbHost::HidDevice> &dev) {
      on_device_connected(dev);
    };
    uc.on_device_disconnected = [this](const std::shared_ptr<UsbHost::HidDevice> &dev) {
      on_device_disconnected(dev);
    };
    return uc;
  }

  // Runs on UsbHost's dispatch task, before the device is started (so the input
  // callback installed here sees the very first report).
  void on_device_connected(const std::shared_ptr<UsbHost::HidDevice> &dev) {
    if (!looks_like_wdi(dev->report_descriptor())) {
      logger_.debug("ignoring non-WDI HID device {:#06x}:{:#06x}", dev->info().vid,
                    dev->info().pid);
      return;
    }
    std::shared_ptr<WdiHost> h;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (device_) {
        logger_.warn("a WDI device is already connected; ignoring the new one");
        return;
      }
      device_ = dev;

      WdiHost::Config hc;
      hc.host_uuid = config_.host_uuid;
      hc.on_control = config_.on_control;
      hc.feedback = config_.feedback;
      hc.on_connected = config_.on_connected;
      hc.on_disconnected = config_.on_disconnected;
      // WdiHost sends an OUTPUT report -> HID SET_REPORT (report id + payload).
      hc.send = [this](wdi::ReportId id, std::span<const uint8_t> payload) {
        std::shared_ptr<UsbHost::HidDevice> d;
        {
          std::lock_guard<std::mutex> lk(mutex_);
          d = device_;
        }
        std::error_code ec;
        return d && d->send_output_report(static_cast<uint8_t>(id), payload, ec);
      };
      h = std::make_shared<WdiHost>(hc);
      if (feedback_)
        h->set_feedback(*feedback_);
      host_ = h;
    }

    // Route the device's INPUT reports (report id in byte 0) into the host core.
    // Invoked with the wrapper mutex released, so on_control etc. may re-enter.
    dev->set_input_callback([this](std::span<const uint8_t> data) {
      if (data.empty())
        return;
      if (auto hh = host())
        hh->handle_input(static_cast<wdi::ReportId>(data[0]), data.subspan(1));
    });
    logger_.info("WDI accessory connected ({:#06x}:{:#06x})", dev->info().vid, dev->info().pid);
  }

  void on_device_disconnected(const std::shared_ptr<UsbHost::HidDevice> &dev) {
    bool was_ours = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (device_ && device_.get() == dev.get()) {
        was_ours = true;
        host_.reset();
        device_.reset();
      }
    }
    if (was_ours) {
      logger_.info("WDI accessory disconnected");
      // The USB link is gone; the app is no longer driving. Notify the caller so
      // it can drive-disable (mirrors the watchdog's on_disconnected).
      if (config_.on_disconnected)
        config_.on_disconnected();
    }
  }

  Config config_;
  UsbHost usb_;
  mutable std::mutex mutex_;
  std::shared_ptr<UsbHost::HidDevice> device_{};
  std::shared_ptr<WdiHost> host_{};
  std::optional<wdi::FeedbackReport> feedback_{};
};

} // namespace espp

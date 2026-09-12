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
// Only one WDI device is tracked at a time (a wheelchair has one active
// accessory link). Usage: construct, initialize(), set_feedback() as the chair's
// status changes, and call poll() periodically so the watchdog can drive-disable
// if the accessory goes quiet.

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
    std::lock_guard<std::mutex> lk(mutex_);
    feedback_ = fb;
    if (host_)
      host_->set_feedback(fb);
  }

  /// @brief Send a Feedback report now (if a device is connected).
  bool send_feedback() {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->send_feedback() : false;
  }

  /// @brief Run the keepalive watchdog; call periodically (e.g. from a Timer).
  ///        Fires on_disconnected if the accessory has gone quiet too long.
  bool poll() {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->poll() : false;
  }

  /// @brief Whether a WDI accessory is currently connected and talking.
  bool is_connected() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ && host_->is_connected();
  }

  /// @brief The most recent Control report, if any.
  std::optional<wdi::ControlReport> last_control() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->last_control() : std::nullopt;
  }

  /// @brief Access the underlying USB host (e.g. to enumerate all HID devices).
  UsbHost &usb() { return usb_; }

  /// @brief Heuristic: does a HID report descriptor look like a WDI device? (It
  ///        declares the WDI vendor usage page 0xFF00: the bytes 06 00 FF.)
  static bool looks_like_wdi(std::span<const uint8_t> descriptor) {
    for (size_t i = 0; i + 2 < descriptor.size(); ++i) {
      if (descriptor[i] == 0x06 && descriptor[i + 1] == 0x00 && descriptor[i + 2] == 0xFF)
        return true;
    }
    return false;
  }

private:
  UsbHost::Config make_usb_config(const Config &c) {
    UsbHost::Config uc;
    uc.log_level = c.log_level;
    uc.auto_start = true;
    // Only open HID devices that advertise the WDI vendor usage page. The filter
    // sees only info/params (not the descriptor), so accept all here and confirm
    // via the descriptor on connect.
    uc.on_device_connected = [this](const std::shared_ptr<UsbHost::HidDevice> &dev) {
      on_device_connected(dev);
    };
    uc.on_device_disconnected = [this](const std::shared_ptr<UsbHost::HidDevice> &dev) {
      on_device_disconnected(dev);
    };
    return uc;
  }

  void on_device_connected(const std::shared_ptr<UsbHost::HidDevice> &dev) {
    if (!looks_like_wdi(dev->report_descriptor())) {
      logger_.debug("ignoring non-WDI HID device {:#06x}:{:#06x}", dev->info().vid,
                    dev->info().pid);
      return;
    }
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
      std::error_code ec;
      auto d = device_; // captured; valid while connected
      return d && d->send_output_report(static_cast<uint8_t>(id), payload, ec);
    };
    host_ = std::make_unique<WdiHost>(hc);
    if (feedback_)
      host_->set_feedback(*feedback_);

    // Route the device's INPUT reports (report id in byte 0) into the host core.
    dev->set_input_callback([this](std::span<const uint8_t> data) {
      if (data.empty())
        return;
      std::lock_guard<std::mutex> lk(mutex_);
      if (host_)
        host_->handle_input(static_cast<wdi::ReportId>(data[0]), data.subspan(1));
    });
    logger_.info("WDI accessory connected ({:#06x}:{:#06x})", dev->info().vid, dev->info().pid);
  }

  void on_device_disconnected(const std::shared_ptr<UsbHost::HidDevice> &dev) {
    std::unique_ptr<WdiHost> dead;
    bool was_ours = false;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (device_ && device_->handle() == dev->handle()) {
        was_ours = true;
        dead = std::move(host_);
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
  std::unique_ptr<WdiHost> host_{};
  std::optional<wdi::FeedbackReport> feedback_{};
};

} // namespace espp

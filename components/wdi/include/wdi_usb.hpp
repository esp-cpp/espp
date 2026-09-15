#pragma once

// WDI (Wheelchair Digital Interface) USB peripheral — the **device** role over
// USB. Wraps the transport-agnostic espp::WdiDevice with an espp::UsbDevice HID
// interface using the WDI report descriptor (wdi_hid.hpp): the accessory / app
// enumerates as a WDI HID device and drives a wheelchair (the USB host).
//
// Control / Request-Feedback / Keepalive are HID **Input** reports (device->host,
// sent with write_hid_report()); Feedback / Keepalive-Response are HID **Output**
// reports (host->device, delivered via the HID receive callback -- which needs
// UsbDevice's HidFunction::on_receive + has_out_endpoint). Device-only (TinyUSB);
// the report logic + keepalive state machine live in WdiDevice (host-tested).
//
// Usage: construct, initialize(), then call poll() periodically (from an
// espp::Timer / Task) so keepalives are sent, and send_control() to drive.

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <system_error>

#include "base_component.hpp"
#include "usb_device.hpp"

#include "wdi.hpp"
#include "wdi_hid.hpp"

namespace espp {

/// @brief The WDI device role over USB (a HID device).
class WdiUsbPeripheral : public BaseComponent {
public:
  struct Config {
    WdiDevice::feedback_fn on_feedback{nullptr};            ///< called with each Feedback report
    WdiDevice::host_uuid_fn on_keepalive_response{nullptr}; ///< called with the host's UUID
    uint32_t keepalive_interval_ms{wdi::kAppKeepaliveIntervalMs}; ///< keepalive send interval
    uint16_t vid{0x1209};              ///< USB VID (default: pid.codes); set your own
    uint16_t pid{0x0d32};              ///< USB PID
    std::string manufacturer{"espp"};  ///< USB manufacturer string
    std::string product{"espp WDI"};   ///< USB product string
    std::string interface_name{"WDI"}; ///< HID interface string
    uint8_t poll_interval_ms{10};      ///< HID interrupt IN polling interval
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  explicit WdiUsbPeripheral(const Config &config)
      : BaseComponent("WdiUsbPeripheral", config.log_level)
      , device_(make_device_config(config))
      , usb_(make_usb_config(config)) {}

  /// @brief Install the TinyUSB driver + WDI HID interface.
  bool initialize(std::error_code &ec) { return usb_.initialize(ec); }

  // --- app API (forwards to the internal WdiDevice) --------------------------
  bool send_control(const wdi::ControlReport &c) { return device_.send_control(c); }
  bool send_release() { return device_.send_release(); }
  bool request_feedback() { return device_.request_feedback(); }
  bool send_keepalive() { return device_.send_keepalive(); }
  /// @brief Emit a keepalive if due; call periodically (e.g. from an espp::Timer).
  bool poll() { return device_.poll(); }
  std::optional<wdi::HostUuid> host_uuid() const { return device_.host_uuid(); }
  std::optional<wdi::FeedbackReport> last_feedback() const { return device_.last_feedback(); }

  /// @brief Access the underlying USB device (e.g. to check is_hid_ready()).
  UsbDevice &usb() { return usb_; }

private:
  WdiDevice::Config make_device_config(const Config &c) {
    WdiDevice::Config dc;
    dc.on_feedback = c.on_feedback;
    dc.on_keepalive_response = c.on_keepalive_response;
    dc.keepalive_interval_ms = c.keepalive_interval_ms;
    // WdiDevice sends a report -> a HID Input report (report id + payload, no
    // report-id byte in the span; write_hid_report supplies the id separately).
    dc.send = [this](wdi::ReportId id, std::span<const uint8_t> p) {
      return usb_.write_hid_report(static_cast<uint8_t>(id), p);
    };
    return dc;
  }

  UsbDevice::Config make_usb_config(const Config &c) {
    UsbDevice::Config uc;
    uc.vid = c.vid;
    uc.pid = c.pid;
    uc.manufacturer = c.manufacturer;
    uc.product = c.product;
    uc.log_level = c.log_level;
    UsbDevice::HidFunction hid;
    hid.interface_name = c.interface_name;
    hid.report_descriptor = {wdi::kReportDescriptor.begin(), wdi::kReportDescriptor.end()};
    hid.has_out_endpoint = true; // receive host OUTPUT reports (Feedback / KA response)
    hid.poll_interval_ms = c.poll_interval_ms;
    hid.on_receive = [this](std::span<const uint8_t> data) { on_hid_out(data); };
    uc.hid = hid;
    return uc;
  }

  // HID OUTPUT report (host->device): byte 0 is the report id, the rest is the
  // report payload. Route it into the WdiDevice.
  void on_hid_out(std::span<const uint8_t> data) {
    if (data.empty())
      return;
    device_.handle_output(static_cast<wdi::ReportId>(data[0]), data.subspan(1));
  }

  WdiDevice device_;
  UsbDevice usb_;
};

} // namespace espp

#pragma once

// WDI (Wheelchair Digital Interface) BLE peripheral — the **device** role over
// Bluetooth LE. Wraps the transport-agnostic espp::WdiDevice with the WDI GATT
// service (service 10A50001-…, characteristics 10A5000{6..A}) built on
// esp-nimble-cpp, so an accessory/app advertises as a WDI device and drives a
// wheelchair (BLE central) over the standard characteristics.
//
// This is device-only (NimBLE); the report logic + keepalive state machine live
// in WdiDevice (host-tested). Usage: create it, then after BleGattServer::init()
// call make_service(server.server()), start() it, advertise service_uuid(), and
// call poll() periodically (from an espp::Timer / Task) so keepalives are sent.

#include <cstdint>
#include <optional>
#include <span>

#include "NimBLEDevice.h"

#include "base_component.hpp"

#include "wdi.hpp"
#include "wdi_hid.hpp" // the HID report descriptor served by the Report Map characteristic

namespace espp {

/// @brief The WDI device role over BLE (a GATT peripheral).
class WdiBlePeripheral : public BaseComponent {
public:
  // 128-bit WDI UUIDs (base 10A5xxxx-C4EA-4B47-AE30-A7D9577FC3F9).
  static constexpr const char *kServiceUuid = "10A50001-C4EA-4B47-AE30-A7D9577FC3F9";
  // HID-over-GATT characteristics (per the WDI spec, mirroring HOGP):
  static constexpr const char *kReportMapUuid = "10A50002-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kHidInformationUuid = "10A50003-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kHidControlPointUuid = "10A50004-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kProtocolModeUuid = "10A50005-C4EA-4B47-AE30-A7D9577FC3F9";
  // Report characteristics:
  static constexpr const char *kControlUuid = "10A50006-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kFeedbackUuid = "10A50007-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kRequestFeedbackUuid = "10A50008-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kKeepaliveUuid = "10A50009-C4EA-4B47-AE30-A7D9577FC3F9";
  static constexpr const char *kKeepaliveResponseUuid = "10A5000A-C4EA-4B47-AE30-A7D9577FC3F9";

  struct Config {
    WdiDevice::feedback_fn on_feedback{nullptr};            ///< called with each Feedback report
    WdiDevice::host_uuid_fn on_keepalive_response{nullptr}; ///< called with the host's UUID
    uint32_t keepalive_interval_ms{wdi::kAppKeepaliveIntervalMs}; ///< keepalive send interval
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  explicit WdiBlePeripheral(const Config &config)
      : BaseComponent("WdiBlePeripheral", config.log_level)
      , device_(make_device_config(config)) {}

  /// @brief The WDI GATT service UUID (advertise this so a wheelchair finds it).
  static NimBLEUUID service_uuid() { return NimBLEUUID(kServiceUuid); }

  /// @brief Create the WDI service + characteristics on `server`. Call after
  ///        BleGattServer::init() (which creates the NimBLEServer) and before
  ///        start().
  void make_service(NimBLEServer *server) {
    if (server == nullptr) {
      logger_.error("null server");
      return;
    }
    service_ = server->createService(NimBLEUUID(kServiceUuid));
    if (service_ == nullptr) {
      logger_.error("failed to create WDI service");
      return;
    }

    // HID-over-GATT descriptor characteristics (WDI spec 0x02..0x05). The Report
    // Map serves the *same* HID report descriptor as the USB transport so a
    // central can introspect the report layout.
    auto *report_map =
        service_->createCharacteristic(NimBLEUUID(kReportMapUuid), NIMBLE_PROPERTY::READ);
    // HID Information: bcdHID 0x0111 (LE), bCountryCode 0, Flags 0x02 (normally
    // connectable).
    auto *hid_info =
        service_->createCharacteristic(NimBLEUUID(kHidInformationUuid), NIMBLE_PROPERTY::READ);
    // HID Control Point: write-without-response suspend/resume command (accepted
    // and ignored by this emulator).
    service_->createCharacteristic(NimBLEUUID(kHidControlPointUuid), NIMBLE_PROPERTY::WRITE_NR);
    // Protocol Mode: default Report Protocol (0x01).
    auto *protocol_mode = service_->createCharacteristic(
        NimBLEUUID(kProtocolModeUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);

    // app -> host (device sends): READ | NOTIFY.
    control_ = service_->createCharacteristic(NimBLEUUID(kControlUuid),
                                              NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    request_feedback_ = service_->createCharacteristic(
        NimBLEUUID(kRequestFeedbackUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    keepalive_ = service_->createCharacteristic(NimBLEUUID(kKeepaliveUuid),
                                                NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    // host -> app (device receives): READ | WRITE_NR (write without response).
    feedback_ = service_->createCharacteristic(NimBLEUUID(kFeedbackUuid),
                                               NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
    keepalive_resp_ = service_->createCharacteristic(
        NimBLEUUID(kKeepaliveResponseUuid), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);

    // createCharacteristic() can return nullptr (e.g. out of memory); bail before
    // dereferencing any of them.
    if (!report_map || !hid_info || !protocol_mode || !control_ || !request_feedback_ ||
        !keepalive_ || !feedback_ || !keepalive_resp_) {
      logger_.error("failed to create one or more WDI characteristics");
      return;
    }

    report_map->setValue(wdi::kReportDescriptor.data(), wdi::kReportDescriptor.size());
    static const uint8_t kHidInfo[4] = {0x11, 0x01, 0x00, 0x02};
    hid_info->setValue(kHidInfo, sizeof(kHidInfo));
    static const uint8_t kReportProtocol = 0x01;
    protocol_mode->setValue(&kReportProtocol, 1);
    feedback_->setCallbacks(&feedback_cb_);
    keepalive_resp_->setCallbacks(&keepalive_resp_cb_);
  }

  /// @brief Start the WDI service (after make_service()).
  void start() {
    if (service_)
      service_->start();
  }

  NimBLEService *get_service() { return service_; }

  // --- app API (forwards to the internal WdiDevice) --------------------------
  bool send_control(const wdi::ControlReport &c) { return device_.send_control(c); }
  bool send_release() { return device_.send_release(); }
  bool request_feedback() { return device_.request_feedback(); }
  bool send_keepalive() { return device_.send_keepalive(); }
  /// @brief Emit a keepalive if due; call periodically (e.g. from an espp::Timer).
  bool poll() { return device_.poll(); }
  std::optional<wdi::HostUuid> host_uuid() const { return device_.host_uuid(); }
  std::optional<wdi::FeedbackReport> last_feedback() const { return device_.last_feedback(); }

private:
  WdiDevice::Config make_device_config(const Config &c) const {
    WdiDevice::Config dc;
    dc.on_feedback = c.on_feedback;
    dc.on_keepalive_response = c.on_keepalive_response;
    dc.keepalive_interval_ms = c.keepalive_interval_ms;
    dc.send = [this](wdi::ReportId id, std::span<const uint8_t> p) { return notify_report(id, p); };
    return dc;
  }

  // WdiDevice send: notify the characteristic for an app->host report.
  bool notify_report(wdi::ReportId id, std::span<const uint8_t> payload) const {
    NimBLECharacteristic *ch = nullptr;
    switch (id) {
    case wdi::ReportId::Control:
      ch = control_;
      break;
    case wdi::ReportId::RequestFeedback:
      ch = request_feedback_;
      break;
    case wdi::ReportId::Keepalive:
      ch = keepalive_;
      break;
    default:
      return false; // host->device reports are not sent by the device
    }
    if (ch == nullptr)
      return false;                               // make_service() not called yet
    ch->setValue(payload.data(), payload.size()); // update the readable value too
    return ch->notify();
  }

  void on_write_report(wdi::ReportId id, std::span<const uint8_t> data) {
    device_.handle_output(id, data);
  }

  // NimBLE write callback for a host->device characteristic; routes the written
  // bytes into the WdiDevice as the given report id.
  class WriteCb : public NimBLECharacteristicCallbacks {
  public:
    WriteCb(WdiBlePeripheral *owner, wdi::ReportId id)
        : owner_(owner)
        , id_(id) {}
    void onWrite(NimBLECharacteristic *ch, NimBLEConnInfo &) override {
      const NimBLEAttValue v = ch->getValue();
      owner_->on_write_report(id_, std::span<const uint8_t>(v.data(), v.length()));
    }

  private:
    WdiBlePeripheral *owner_;
    wdi::ReportId id_;
  };

  WdiDevice device_;
  NimBLEService *service_{nullptr};
  NimBLECharacteristic *control_{nullptr};          // 0x01 notify
  NimBLECharacteristic *feedback_{nullptr};         // 0x02 write
  NimBLECharacteristic *request_feedback_{nullptr}; // 0x03 notify
  NimBLECharacteristic *keepalive_{nullptr};        // 0x04 notify
  NimBLECharacteristic *keepalive_resp_{nullptr};   // 0x05 write
  WriteCb feedback_cb_{this, wdi::ReportId::Feedback};
  WriteCb keepalive_resp_cb_{this, wdi::ReportId::KeepaliveResponse};
};

} // namespace espp

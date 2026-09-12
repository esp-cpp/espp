#pragma once

// WDI (Wheelchair Digital Interface) BLE **central** — the wheelchair role over
// Bluetooth LE. Wraps the transport-agnostic espp::WdiHost with a NimBLE central
// (client): it scans for / connects to a WDI peripheral (an accessory / app
// running e.g. espp::WdiBlePeripheral) and speaks the host side of the protocol.
//
// The WDI GATT characteristics keep their device-role direction: Control (0x06),
// Request-Feedback (0x08) and Keepalive (0x09) are **Notify** (peripheral ->
// central, i.e. app -> host), so the central subscribes to them and routes each
// into WdiHost::handle_input(); Feedback (0x07) and Keepalive-Response (0x0A) are
// **Write** (central -> peripheral, i.e. host -> app), so WdiHost's send callback
// writes them. The report logic + keepalive watchdog live in WdiHost (host-tested).
//
// Requires NimBLEDevice::init() to have been called first (see the example).
// Usage: construct, scan_and_connect() (or connect(address)), set_feedback() as
// the chair's status changes, and call poll() periodically so the watchdog can
// drive-disable if the accessory goes quiet.

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <system_error>

#include "NimBLEDevice.h"

#include "base_component.hpp"

#include "wdi_ble.hpp" // reuse the WDI GATT UUID constants (WdiBlePeripheral::k*Uuid)
#include "wdi_host.hpp"

namespace espp {

/// @brief The WDI host role over BLE (a GATT central talking to a WDI peripheral).
class WdiBleCentral : public BaseComponent {
public:
  struct Config {
    WdiHost::control_fn on_control{nullptr};         ///< a Control report arrived
    WdiHost::feedback_provider_fn feedback{nullptr}; ///< current Feedback to report
    WdiHost::link_fn on_connected{nullptr};          ///< the WDI link came up
    WdiHost::link_fn on_disconnected{nullptr};       ///< the link dropped / watchdog fired
    wdi::HostUuid host_uuid{};                       ///< the host's identity
    Logger::Verbosity log_level{Logger::Verbosity::WARN};
  };

  explicit WdiBleCentral(const Config &config)
      : BaseComponent("WdiBleCentral", config.log_level)
      , config_(config) {}

  ~WdiBleCentral() { disconnect(); }

  /// @brief The WDI service UUID (scan for peripherals advertising this).
  static NimBLEUUID service_uuid() { return NimBLEUUID(WdiBlePeripheral::kServiceUuid); }

  /// @brief Scan for a peripheral advertising the WDI service and connect to the
  ///        first one found. Blocks up to `scan_ms`.
  bool scan_and_connect(uint32_t scan_ms, std::error_code &ec) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (!scan) {
      ec = std::make_error_code(std::errc::not_connected);
      return false;
    }
    scan->setActiveScan(true);
    NimBLEScanResults results = scan->getResults(scan_ms, false);
    const NimBLEUUID svc = service_uuid();
    for (int i = 0; i < results.getCount(); ++i) {
      const NimBLEAdvertisedDevice *dev = results.getDevice(i);
      if (dev && dev->isAdvertisingService(svc)) {
        logger_.info("found WDI peripheral {}", dev->getAddress().toString());
        scan->clearResults();
        return connect(dev->getAddress(), ec);
      }
    }
    scan->clearResults();
    logger_.warn("no WDI peripheral found");
    ec = std::make_error_code(std::errc::no_such_device);
    return false;
  }

  /// @brief Connect to a specific peripheral address, discover the WDI service,
  ///        subscribe to its notify characteristics, and start the host role.
  bool connect(const NimBLEAddress &address, std::error_code &ec) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (client_) {
        ec = std::make_error_code(std::errc::already_connected);
        return false;
      }
    }
    NimBLEClient *client = NimBLEDevice::createClient();
    if (!client) {
      ec = std::make_error_code(std::errc::not_enough_memory);
      return false;
    }
    callbacks_.owner = this;
    client->setClientCallbacks(&callbacks_, false);
    {
      std::lock_guard<std::mutex> lk(mutex_);
      client_ = client; // publish so on_ble_disconnect() can tear it down
    }

    // The blocking connect + GATT discovery + subscribe below are done WITHOUT
    // holding mutex_: NimBLE runs its host on a separate task and invokes our
    // callbacks (onDisconnect / notify) from it, so holding the lock across these
    // calls would deadlock (the host task would block on mutex_ and never signal
    // completion).
    if (!client->connect(address)) {
      logger_.error("connect failed");
      teardown_client(client);
      ec = std::make_error_code(std::errc::connection_refused);
      return false;
    }
    NimBLERemoteService *service = client->getService(service_uuid());
    if (!service) {
      logger_.error("WDI service not found on peer");
      teardown_client(client);
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    auto *control = service->getCharacteristic(NimBLEUUID(WdiBlePeripheral::kControlUuid));
    auto *request_feedback =
        service->getCharacteristic(NimBLEUUID(WdiBlePeripheral::kRequestFeedbackUuid));
    auto *keepalive = service->getCharacteristic(NimBLEUUID(WdiBlePeripheral::kKeepaliveUuid));
    auto *feedback = service->getCharacteristic(NimBLEUUID(WdiBlePeripheral::kFeedbackUuid));
    auto *keepalive_response =
        service->getCharacteristic(NimBLEUUID(WdiBlePeripheral::kKeepaliveResponseUuid));
    if (!control || !request_feedback || !keepalive || !feedback || !keepalive_response) {
      logger_.error("WDI characteristics incomplete");
      teardown_client(client);
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }

    // Publish the characteristics + build the host core under the lock, before
    // subscribing, so an early notification finds a live host_.
    {
      std::lock_guard<std::mutex> lk(mutex_);
      control_ = control;
      request_feedback_ = request_feedback;
      keepalive_ = keepalive;
      feedback_ = feedback;
      keepalive_response_ = keepalive_response;
      WdiHost::Config hc;
      hc.host_uuid = config_.host_uuid;
      hc.on_control = config_.on_control;
      hc.feedback = config_.feedback;
      hc.on_connected = config_.on_connected;
      hc.on_disconnected = config_.on_disconnected;
      hc.send = [this](wdi::ReportId id, std::span<const uint8_t> payload) {
        NimBLERemoteCharacteristic *chr = (id == wdi::ReportId::Feedback) ? feedback_
                                          : (id == wdi::ReportId::KeepaliveResponse)
                                              ? keepalive_response_
                                              : nullptr;
        if (!chr)
          return false;
        return chr->writeValue(payload.data(), payload.size(), /*response=*/false);
      };
      host_ = std::make_unique<WdiHost>(hc);
      if (feedback_value_)
        host_->set_feedback(*feedback_value_);
    }

    // Subscribe to the app's INPUT reports (Notify): Control / Request-Feedback /
    // Keepalive. A failed subscription means those notifications never arrive (the
    // watchdog would trip), so fail the connect rather than report success.
    auto cb = [this](NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len, bool) {
      on_notify(chr, data, len);
    };
    if (!control->subscribe(true, cb) || !request_feedback->subscribe(true, cb) ||
        !keepalive->subscribe(true, cb)) {
      logger_.error("failed to subscribe to WDI notifications");
      teardown_client(client);
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    logger_.info("WDI peripheral connected");
    ec.clear();
    return true;
  }

  /// @brief Disconnect and tear down.
  void disconnect() {
    std::unique_ptr<WdiHost> dead;
    NimBLEClient *client = nullptr;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      dead = std::move(host_);
      client = client_;
      client_ = nullptr;
      control_ = request_feedback_ = keepalive_ = feedback_ = keepalive_response_ = nullptr;
    }
    if (client) {
      if (client->isConnected())
        client->disconnect();
      NimBLEDevice::deleteClient(client);
    }
  }

  /// @brief Update the Feedback reported to the accessory (host->app).
  void set_feedback(const wdi::FeedbackReport &fb) {
    std::lock_guard<std::mutex> lk(mutex_);
    feedback_value_ = fb;
    if (host_)
      host_->set_feedback(fb);
  }

  /// @brief Send a Feedback report now (if connected).
  bool send_feedback() {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->send_feedback() : false;
  }

  /// @brief Run the keepalive watchdog; call periodically.
  bool poll() {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->poll() : false;
  }

  /// @brief Whether a WDI accessory is connected and talking.
  bool is_connected() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ && host_->is_connected();
  }

  /// @brief The most recent Control report, if any.
  std::optional<wdi::ControlReport> last_control() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_ ? host_->last_control() : std::nullopt;
  }

private:
  // Drop any published state referring to `client`, then disconnect + delete it.
  // Called from connect()'s failure paths; does not hold mutex_ across the BLE
  // calls.
  void teardown_client(NimBLEClient *client) {
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (client_ == client)
        client_ = nullptr;
      host_.reset();
      control_ = request_feedback_ = keepalive_ = feedback_ = keepalive_response_ = nullptr;
    }
    if (client->isConnected())
      client->disconnect();
    NimBLEDevice::deleteClient(client);
  }

  // Route a notification to the host core by which characteristic delivered it.
  void on_notify(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len) {
    wdi::ReportId id;
    if (chr == control_)
      id = wdi::ReportId::Control;
    else if (chr == request_feedback_)
      id = wdi::ReportId::RequestFeedback;
    else if (chr == keepalive_)
      id = wdi::ReportId::Keepalive;
    else
      return;
    std::lock_guard<std::mutex> lk(mutex_);
    if (host_)
      host_->handle_input(id, std::span<const uint8_t>(data, len));
  }

  void on_ble_disconnect() {
    std::unique_ptr<WdiHost> dead;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      dead = std::move(host_);
      control_ = request_feedback_ = keepalive_ = feedback_ = keepalive_response_ = nullptr;
      // client_ is deleted by NimBLE after the callback; drop our pointer.
      client_ = nullptr;
    }
    logger_.info("WDI peripheral disconnected");
    if (config_.on_disconnected)
      config_.on_disconnected();
  }

  struct Callbacks : public NimBLEClientCallbacks {
    WdiBleCentral *owner{nullptr};
    void onDisconnect(NimBLEClient * /*client*/, int /*reason*/) override {
      if (owner)
        owner->on_ble_disconnect();
    }
  };

  Config config_;
  mutable std::mutex mutex_;
  Callbacks callbacks_{};
  NimBLEClient *client_{nullptr};
  NimBLERemoteCharacteristic *control_{nullptr};
  NimBLERemoteCharacteristic *request_feedback_{nullptr};
  NimBLERemoteCharacteristic *keepalive_{nullptr};
  NimBLERemoteCharacteristic *feedback_{nullptr};
  NimBLERemoteCharacteristic *keepalive_response_{nullptr};
  std::unique_ptr<WdiHost> host_{};
  std::optional<wdi::FeedbackReport> feedback_value_{};
};

} // namespace espp

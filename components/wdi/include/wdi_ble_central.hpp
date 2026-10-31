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
// Threading: NimBLE invokes our notify / disconnect callbacks on its host task.
// The wrapper's mutex only guards its pointers and is never held across a
// blocking NimBLE call or a user callback -- the WdiHost core (itself
// thread-safe) is always invoked with the mutex released, so on_control /
// on_disconnected / ... may call back into this object. The Output writes made
// from the notify path are write-without-response (non-blocking) and, per the
// spec's report sizes, always fit the minimum ATT MTU (see the static_assert).
//
// One NimBLEClient is created on the first connect and reused for the object's
// lifetime (NimBLE clients are reconnectable); it is deleted only in the
// destructor. Requires NimBLEDevice::init() to have been called first (see the
// example). Usage: construct, scan_and_connect() (or connect(address)),
// set_feedback() as the chair's status changes, and call poll() periodically so
// the watchdog can drive-disable if the accessory goes quiet. on_disconnected
// may fire twice for one link loss (watchdog, then the BLE drop); it is
// idempotent for its purpose. connect()/disconnect() are not re-entrant with
// each other.

#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <system_error>
#include <thread>

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

  /// @brief Disconnects and releases the NimBLE client. Detaches our callbacks
  ///        before doing so, so a disconnect event that lands after this object
  ///        is gone cannot call into it.
  ~WdiBleCentral() {
    disconnect();
    NimBLEClient *client = nullptr;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      client = client_;
      client_ = nullptr;
    }
    if (client) {
      client->setClientCallbacks(nullptr, false);
      // disconnect() is asynchronous; give the link a moment to actually drop
      // so the client can be deleted immediately rather than deferred.
      for (int i = 0; i < 50 && client->isConnected(); ++i)
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      NimBLEDevice::deleteClient(client);
    }
  }

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
    std::optional<NimBLEAddress> found;
    for (int i = 0; i < results.getCount(); ++i) {
      const NimBLEAdvertisedDevice *dev = results.getDevice(i);
      if (dev && dev->isAdvertisingService(svc)) {
        found = dev->getAddress(); // copy: clearResults() deletes the entries
        break;
      }
    }
    scan->clearResults();
    if (!found) {
      logger_.warn("no WDI peripheral found");
      ec = std::make_error_code(std::errc::no_such_device);
      return false;
    }
    logger_.info("found WDI peripheral {}", found->toString());
    return connect(*found, ec);
  }

  /// @brief Connect to a specific peripheral address, discover the WDI service,
  ///        subscribe to its notify characteristics, and start the host role.
  bool connect(const NimBLEAddress &address, std::error_code &ec) {
    NimBLEClient *client = nullptr;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (host_) {
        ec = std::make_error_code(std::errc::already_connected);
        return false;
      }
      if (!client_) {
        client_ = NimBLEDevice::createClient();
        if (!client_) {
          ec = std::make_error_code(std::errc::not_enough_memory);
          return false;
        }
        callbacks_.owner = this;
        client_->setClientCallbacks(&callbacks_, false);
      }
      client = client_;
    }

    // The blocking connect + GATT discovery + subscribe below are done WITHOUT
    // holding mutex_: NimBLE runs its host on a separate task and invokes our
    // callbacks (onDisconnect / notify) from it, so holding the lock across these
    // calls would deadlock (the host task would block on mutex_ and never signal
    // completion).
    if (!client->connect(address)) {
      logger_.error("connect failed");
      ec = std::make_error_code(std::errc::connection_refused);
      return false;
    }
    NimBLERemoteService *service = client->getService(service_uuid());
    if (!service) {
      logger_.error("WDI service not found on peer");
      client->disconnect();
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
      client->disconnect();
      ec = std::make_error_code(std::errc::protocol_error);
      return false;
    }

    // Publish the characteristics + build the host core under the lock, before
    // subscribing, so an early notification finds a live host.
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
        NimBLERemoteCharacteristic *chr = nullptr;
        {
          std::lock_guard<std::mutex> lk2(mutex_);
          chr = (id == wdi::ReportId::Feedback)            ? feedback_
                : (id == wdi::ReportId::KeepaliveResponse) ? keepalive_response_
                                                           : nullptr;
        }
        if (!chr)
          return false;
        return chr->writeValue(payload.data(), payload.size(), /*response=*/false);
      };
      auto h = std::make_shared<WdiHost>(hc);
      if (feedback_value_)
        h->set_feedback(*feedback_value_);
      host_ = h;
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
      clear_link();
      client->disconnect();
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }

    logger_.info("WDI peripheral connected");
    ec.clear();
    return true;
  }

  /// @brief Drop the WDI link (the client is kept for a later connect()).
  void disconnect() {
    clear_link();
    NimBLEClient *client = nullptr;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      client = client_;
    }
    if (client && client->isConnected())
      client->disconnect();
  }

  /// @brief Update the Feedback reported to the accessory (host->app).
  void set_feedback(const wdi::FeedbackReport &fb) {
    std::shared_ptr<WdiHost> h;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      feedback_value_ = fb;
      h = host_;
    }
    if (h)
      h->set_feedback(fb);
  }

  /// @brief Send a Feedback report now (if connected).
  // Not const: it has side effects through the WdiHost core (sends a report /
  // fires user callbacks) even though it only reads this wrapper.
  // cppcheck-suppress functionConst
  bool send_feedback() {
    auto h = host();
    return h ? h->send_feedback() : false;
  }

  /// @brief Run the keepalive watchdog; call periodically.
  // Not const: it has side effects through the WdiHost core (sends a report /
  // fires user callbacks) even though it only reads this wrapper.
  // cppcheck-suppress functionConst
  bool poll() {
    auto h = host();
    return h ? h->poll() : false;
  }

  /// @brief Whether a WDI accessory is connected and talking.
  bool is_connected() const {
    auto h = host();
    return h && h->is_connected();
  }

  /// @brief The most recent Control report, if any.
  std::optional<wdi::ControlReport> last_control() const {
    auto h = host();
    return h ? h->last_control() : std::nullopt;
  }

private:
  // The Output writes are issued from the notify path with write-without-
  // response, which NimBLE only performs non-blocking when the value fits in
  // (ATT MTU - 3); larger writes take a blocking path that would deadlock the
  // host task. Both WDI Output reports fit the minimum MTU (23 - 3 = 20).
  static_assert(wdi::kFeedbackSize <= 20 && wdi::kKeepaliveResponseSize <= 20,
                "WDI Output reports must fit the minimum ATT MTU for non-blocking writes");

  std::shared_ptr<WdiHost> host() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return host_;
  }

  // Drop the host core + characteristic pointers (the link-level state).
  // Returns whether a live link existed.
  bool clear_link() {
    std::lock_guard<std::mutex> lk(mutex_);
    const bool had_link = static_cast<bool>(host_);
    host_.reset();
    control_ = request_feedback_ = keepalive_ = feedback_ = keepalive_response_ = nullptr;
    return had_link;
  }

  // Route a notification to the host core by which characteristic delivered it.
  // Runs on the NimBLE host task; the core is invoked with mutex_ released.
  void on_notify(NimBLERemoteCharacteristic *chr, uint8_t *data, size_t len) {
    wdi::ReportId id;
    std::shared_ptr<WdiHost> h;
    {
      std::lock_guard<std::mutex> lk(mutex_);
      if (chr == control_)
        id = wdi::ReportId::Control;
      else if (chr == request_feedback_)
        id = wdi::ReportId::RequestFeedback;
      else if (chr == keepalive_)
        id = wdi::ReportId::Keepalive;
      else
        return;
      h = host_;
    }
    if (h)
      h->handle_input(id, std::span<const uint8_t>(data, len));
  }

  // NimBLE reports the link dropped (peer went away, or our own disconnect()).
  // Only report a disconnect to the application if a WDI link was actually up
  // (not for a failed connect attempt or an intentional disconnect()).
  void on_ble_disconnect() {
    const bool had_link = clear_link();
    if (had_link) {
      logger_.info("WDI peripheral disconnected");
      if (config_.on_disconnected)
        config_.on_disconnected();
    }
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
  NimBLEClient *client_{nullptr}; // created on first connect(), reused, deleted in dtor
  NimBLERemoteCharacteristic *control_{nullptr};
  NimBLERemoteCharacteristic *request_feedback_{nullptr};
  NimBLERemoteCharacteristic *keepalive_{nullptr};
  NimBLERemoteCharacteristic *feedback_{nullptr};
  NimBLERemoteCharacteristic *keepalive_response_{nullptr};
  std::shared_ptr<WdiHost> host_{};
  std::optional<wdi::FeedbackReport> feedback_value_{};
};

} // namespace espp

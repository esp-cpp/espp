#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "NimBLEDevice.h"
#include "ble_gatt_server.hpp"

#include "base_component.hpp"

#include "switch2_pro_pairing.hpp"
#include "switch2_pro_protocol.hpp"
#include "switch2_pro_report.hpp"

namespace espp {

/// @brief Emulates a Nintendo Switch 2 Pro Controller as a BLE peripheral.
///
/// The Switch 2 uses a proprietary BLE GATT interface (not HID-over-GATT) with
/// a custom pairing handshake (not BLE SMP). This class stands up that GATT
/// tree on top of espp::BleGattServer, advertises with Nintendo manufacturer
/// data, and answers the console's command channel — including the reverse-
/// engineered pairing handshake so a real console will bond with it.
///
/// Milestone status: GATT + pairing skeleton. Advertising, the custom service
/// tree, and the 0x15 pairing handshake are wired; the full init/calibration
/// sequence and input-report streaming are staged in follow-up work (see
/// DESIGN.md). Emulating the console's 5 ms connection interval additionally
/// requires the opt-in NimBLE patch (tools/patch_nimble_5ms.py).
///
/// \section switch2_pro_ex1 Example
/// \snippet switch2_pro_example.cpp switch2_pro example
class Switch2Pro : public BaseComponent {
public:
  /// Configuration for the controller.
  struct Config {
    std::string device_name{"Pro Controller"}; ///< BLE advertised name.
    Logger::Verbosity log_level{Logger::Verbosity::INFO};
  };

  explicit Switch2Pro(const Config &config)
      : BaseComponent("Switch2Pro", config.log_level)
      , device_name_(config.device_name)
      , ble_gatt_server_({.callbacks = {}, .log_level = Logger::Verbosity::WARN}) {}

  /// Initialize NimBLE, build the custom GATT services, configure security so
  /// the console (not standard SMP) drives pairing, and start advertising.
  /// @return true on success.
  bool init();

  /// Whether the pairing handshake has completed with a console.
  bool is_paired() const { return paired_; }

  /// Latest controller state to report once input streaming is enabled.
  void set_input_report(const switch2::Pro2InputReport &report) { input_report_ = report; }

protected:
  // --- setup ---
  bool build_gatt();
  void configure_security();
  void configure_callbacks();
  void start_advertising(bool wake, const std::array<uint8_t, 6> &host_addr = {});

  /// Log a byte buffer as hex at debug level (command/response tracing).
  void log_hex(const char *prefix, const uint8_t *data, size_t len);

  // --- command channel ---
  /// Handle a write on a command characteristic. `via_vibration_command` is
  /// true for writes on 0x0016 (pairing/init) which reply on 0x001e, false for
  /// writes on 0x0014 which reply on 0x001a. Parses the 8-byte header and
  /// dispatches, notifying a response.
  void on_command_write(bool via_vibration_command, const uint8_t *data, size_t len);
  void handle_pairing(bool via_vibration_command, uint8_t transport, switch2::PairingSub sub,
                      const uint8_t *payload, size_t len);
  void handle_command(bool via_vibration_command, switch2::Command cmd, uint8_t transport,
                      uint8_t sub, const uint8_t *payload, size_t len);

  /// Build an 8-byte device->host response header + payload and notify it on
  /// the response characteristic matching the request source.
  void send_response(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub,
                     uint8_t byte4, uint8_t byte5, const uint8_t *payload, size_t payload_len);
  /// Header-only ACK (byte4=0x00, byte5=0xf8, payload = {0x01,0,0,0}).
  void send_ack(bool via_vibration_command, uint8_t cmd, uint8_t transport, uint8_t sub);
  /// Our own BT address (6 bytes) for the exchange-addresses reply.
  std::array<uint8_t, 6> local_bt_address() const;

  friend class ChannelCallbacks;

  std::string device_name_;
  BleGattServer ble_gatt_server_;

  // Proprietary GATT characteristics (owned by NimBLE once created).
  NimBLECharacteristic *common_input_{nullptr};
  NimBLECharacteristic *pro2_input_{nullptr};
  NimBLECharacteristic *command_{nullptr};
  NimBLECharacteristic *vibration_command_{nullptr};
  NimBLECharacteristic *command_response1_{nullptr};
  NimBLECharacteristic *command_response2_{nullptr};

  // Pairing state.
  bool paired_{false};
  std::array<uint8_t, 16> ltk_{};      ///< derived during key exchange
  std::array<uint8_t, 6> host_addr_{}; ///< console BD_ADDR (from exchange-addresses)
  uint8_t feature_mask_{switch2::PRO2_FEATURE_MASK};

  switch2::Pro2InputReport input_report_{};
};

} // namespace espp

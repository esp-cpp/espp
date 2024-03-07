#pragma once

#include "gfps.hpp"

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

namespace espp {
/// Callback base class for the Google Fast Pair Service
/// This class provides the base functionality for the Google Fast Pair Service
/// characteristic callbacks.
class GfpsCharacteristicCallback {
public:
  /// Called when a Google Fast Pair Service characteristic is read
  /// \param conn_info The connection information for the device
  /// \param characteristic The characteristic to read
  /// \return The value of the characteristic or an empty vector if the read failed
  std::vector<uint8_t> on_gfps_read(NimBLEConnInfo &conn_info,
                                    nearby_fp_Characteristic characteristic) {
    auto gfps_ble_interface = gfps::get_ble_interface();
    if (!gfps_ble_interface) {
      logger_.error("BLE interface not set");
      return {};
    }
    auto addr = conn_info.getAddress();
    auto addr_64 = uint64_t(addr);
    std::vector<uint8_t> output(32);
    size_t output_size = 32;
    logger_.info("on_gatt_read from address: {}", addr.toString());
    auto status =
        gfps_ble_interface->on_gatt_read(addr_64, characteristic, output.data(), &output_size);
    if (status != kNearbyStatusOK) {
      logger_.error("on_gatt_read returned status {} for {}", (int)status, characteristic);
      return {};
    }
    output.resize(output_size);
    return output;
  }

  /// Called when a Google Fast Pair Service characteristic is written
  /// \param conn_info The connection information for the device
  /// \param characteristic The characteristic to write
  ///
  void on_gfps_write(NimBLEConnInfo &conn_info, nearby_fp_Characteristic characteristic,
                     const uint8_t *value, size_t length) {
    auto gfps_ble_interface = gfps::get_ble_interface();
    if (!gfps_ble_interface) {
      logger_.error("BLE interface not set");
      return;
    }
    auto addr = conn_info.getAddress();
    auto addr_64 = uint64_t(addr);
    logger_.info("on_gatt_write from address: {}, length {}", addr.toString(), length);
    logger_.debug("  with value: {::#02x}", std::vector<uint8_t>(value, value + length));
    auto status = gfps_ble_interface->on_gatt_write(addr_64, characteristic, value, length);
    if (status != kNearbyStatusOK) {
      logger_.error("on_gatt_write returned status {} for {}", (int)status, characteristic);
    }
  }

  /// Set the log level for the class
  /// \param log_level The log level
  void set_log_level(espp::Logger::Verbosity log_level) { logger_.set_verbosity(log_level); }

protected:
  espp::Logger logger_ =
      espp::Logger({.tag = "GfpsCharacteristicCallback", .level = espp::gfps::LOG_LEVEL});
};

class GfpsModelIdCharacteristicCallbacks : public NimBLECharacteristicCallbacks,
                                           public GfpsCharacteristicCallback {
public:
  /// Called when the Model ID Characteristic is read
  /// \param characteristic The characteristic that is being read
  /// \param conn_info The ConnInfo object for the connection associated with the read
  void onRead(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("Model ID characteristic read");
    auto output = on_gfps_read(conn_info, kModelId);
    if (output.empty()) {
      logger_.error("on_gfps_read failed");
      return;
    }
    characteristic->setValue(output.data(), output.size());
  }
}; // class GfpsModelIdCharacteristicCallbacks

class GfpsKbPairingCharacteristicCallbacks : public NimBLECharacteristicCallbacks,
                                             public GfpsCharacteristicCallback {
public:
  /// Called when the Key-Based Pairing Characteristic is written
  /// \param characteristic The characteristic that is being written
  /// \param conn_info The ConnInfo object for the connection associated with the write
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("KB Pairing characteristic written");
    auto value = characteristic->getValue();
    // if the write length is 80, then this is the remote's public key
    // copy the characteristic value (starting at offset 16) to REMOTE_PUBLIC_KEY (64 bytes)
    if (value.size() == 80) {
      std::vector<uint8_t> remote_public_key(value.begin() + 16, value.end());
      logger_.info("Got remote public key of length {}", remote_public_key.size());
      gfps::set_remote_public_key(remote_public_key);
    }
    on_gfps_write(conn_info, kKeyBasedPairing, value.data(), value.size());
    // get the peer address from the connection info
    auto peer_address = uint64_t(conn_info.getAddress());
    auto gfps_bt_interface = gfps::get_bt_interface();
    gfps_bt_interface->on_pairing_request(peer_address);
  }
  /// Called when the Key-Based Pairing Characteristic is read
  /// \param characteristic The characteristic that is being read
  /// \param conn_info The ConnInfo object for the connection associated with the read
  void onRead(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("KB Pairing characteristic read");
    auto output = on_gfps_read(conn_info, kKeyBasedPairing);
    if (output.empty()) {
      logger_.error("on_gfps_read failed");
      return;
    }
    characteristic->setValue(output.data(), output.size());
  }
}; // class GfpsKbPairingCharacteristicCallbacks

class GfpsPasskeyCharacteristicCallbacks : public NimBLECharacteristicCallbacks,
                                           public GfpsCharacteristicCallback {
public:
  /// Called when the Passkey Pairing Characteristic is written
  /// \param characteristic The characteristic that is being written
  /// \param conn_info The ConnInfo object for the connection associated with the write
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("Passkey characteristic written");
    auto value = characteristic->getValue();
    on_gfps_write(conn_info, kPasskey, value.data(), value.size());
  }
  /// Called when the Passkey Characteristic is read
  /// \param characteristic The characteristic that is being read
  /// \param conn_info The ConnInfo object for the connection associated with the read
  void onRead(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("Passkey characteristic read");
    auto output = on_gfps_read(conn_info, kPasskey);
    if (output.empty()) {
      logger_.error("on_gfps_read failed");
      return;
    }
    characteristic->setValue(output.data(), output.size());
  }
}; // class GfpsPasskeyCharacteristicCallbacks

class GfpsAccountKeyCharacteristicCallbacks : public NimBLECharacteristicCallbacks,
                                              public GfpsCharacteristicCallback {
public:
  /// Called when the Account Key Characteristic is written
  /// \param characteristic The characteristic that is being written
  /// \param conn_info The ConnInfo object for the connection associated with the write
  void onWrite(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("Account Key characteristic written");
    auto value = characteristic->getValue();
    on_gfps_write(conn_info, kAccountKey, value.data(), value.size());
  }
  /// Called when the Account Key Characteristic is read
  /// \param characteristic The characteristic that is being read
  /// \param conn_info The ConnInfo object for the connection associated with the read
  void onRead(NimBLECharacteristic *characteristic, NimBLEConnInfo &conn_info) override {
    logger_.info("Account Key characteristic read");
    auto output = on_gfps_read(conn_info, kAccountKey);
    if (output.empty()) {
      logger_.error("on_gfps_read failed");
      return;
    }
    characteristic->setValue(output.data(), output.size());
  }
}; // class GfpsAccountKeyCharacteristicCallbacks
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

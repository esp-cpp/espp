#pragma once

#include <sdkconfig.h>

#include <atomic>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

#include "base_component.hpp"

#include "gfps.hpp"
#include "gfps_characteristic_callbacks.hpp"

namespace espp {
/// Google Fast Pair Service
/// This class is responsible for creating and managing the Google Fast Pair Service.
///
/// The service is created with the following characteristics:
/// - Model ID (read, unencrypted)
/// - KB Pairing (write and notify, unencrypted)
/// - Passkey (write and notify, unencrypted)
/// - Account Key (write, unencrypted)
///
/// Google fast pair service also requires the fast pair provider (device)
/// to support the Device Information Service (DIS, 0x180A) including the
/// Firmware Revision characteristic (0x2A26).
///
/// For more information see
/// https://developers.google.com/nearby/fast-pair/specifications/characteristics
///
/// If you need the DeviceInfoService and BatteryService, you can access
/// them through espp::BleGattServer.
///
/// @see BleGattServer
///
/// \section gfps_service_ex1 GFPS Service Example
/// \snippet gfps_service_example.cpp gfps service example
class GfpsService : public espp::BaseComponent {
public:
  /// Constructor
  /// \param log_level The log level for the service
  explicit GfpsService(espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("GfpsService", log_level) {
    set_service_data();
  }

  /// Constructor
  /// \param server The BLE server to attach the service to
  /// \param log_level The log level for the service
  /// \note This constructor will initialize the service with the given server
  /// \note This constructor will also initialize the nearby framework
  explicit GfpsService(NimBLEServer *server,
                       espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("GfpsService", log_level) {
    set_service_data();
    init(server);
  }

  /// Set the passkey
  /// \param passkey The passkey to set
  /// \note This function will set the passkey for the service
  /// \note This function should be called in the ESP_GAP_BLE_NC_REQ_EVT GAP
  ///       event handler
  void set_passkey(uint32_t passkey) {
    logger_.info("new passkey: {}", passkey);
    NimBLEDevice::setSecurityPasskey(passkey);
  }

  /// Get the passkey
  /// \return The passkey for the service
  uint32_t get_passkey() const { return NimBLEDevice::getSecurityPasskey(); }

  /// On pairing request
  /// This function is called when a pairing request is received
  /// \param conn_info The connection information for the device
  /// \note This function will call the GFPS/nearby on_pairing_request
  ///       function
  /// \note This function should be called in the ESP_GAP_BLE_SEC_REQ_EVT GAP
  ///       event handler
  void on_pairing_request(NimBLEConnInfo &conn_info) {
    auto gfps_bt_interface = gfps::get_bt_interface();
    if (!gfps_bt_interface) {
      logger_.error("BLE interface not set");
      return;
    }
    // get the peer address from the connection info
    auto peer_address = uint64_t(conn_info.getAddress());
    gfps_bt_interface->on_pairing_request(peer_address);
  }

  /// Get the Service
  /// \return The BLE service for the Google Fast Pair Service
  NimBLEService *service() const { return service_; }

  /// Get the Service UUID
  /// \return The UUID for the Google Fast Pair Service
  NimBLEUUID uuid() const {
    if (service_) {
      return service_->getUUID();
    }
    return NimBLEUUID(SERVICE_UUID);
  }

  /// Get the Service Data
  /// \return The service data for the Google Fast Pair Service
  /// \note The service data is the 3 byte model ID
  /// \note This is the service data that is advertised
  std::string get_service_data() const {
    return std::string(reinterpret_cast<const char *>(service_data_), sizeof(service_data_));
  }

  /// Notify the characteristic
  /// \param characteristic The characteristic to notify
  /// \param value The value to notify
  /// \param length The length of the value
  /// \return True if the notification was successful, false otherwise
  /// \note This function will notify the given characteristic with the given
  ///       value
  bool notify(nearby_fp_Characteristic characteristic, const uint8_t *value, size_t length) {
    if (!service_) {
      logger_.error("Service not created");
      return false;
    }
    if (characteristic == kKeyBasedPairing) {
      kb_pairing_->notify(value, length);
      return true;
    } else if (characteristic == kPasskey) {
      passkey_->notify(value, length);
      return true;
    } else if (characteristic == kAccountKey) {
      account_key_->notify(value, length);
      return true;
    } else {
      logger_.error("Unknown characteristic");
    }
    return false;
  }

  /// Initialize the service
  /// \param server The BLE server to attach the service to
  /// \note This function will initialize the service with the given server
  /// \note This function will also initialize the nearby framework
  void init(NimBLEServer *server) {
    // make the service
    make_service(server);
    // initialize gfps
    gfps::init({
        .notify =
            [this](nearby_fp_Characteristic characteristic, const uint8_t *value, size_t length) {
              // notify the characteristic
              return notify(characteristic, value, length);
            },
    });
    // register the gap event handler
    NimBLEDevice::setCustomGapHandler(gfps::ble_gap_event_handler);
  }

  /// Start the service
  /// \note This function will start the service
  /// \note This function will also set the advertisement mode for the nearby
  ///       framework
  void start() {
    if (!service_) {
      logger_.error("Service not created");
      return;
    }
    if (service_->start()) {
      logger_.info("Service started");
    } else {
      logger_.error("Failed to start service");
    }
    int advertisement_mode =
        NEARBY_FP_ADVERTISEMENT_DISCOVERABLE | NEARBY_FP_ADVERTISEMENT_PAIRING_UI_INDICATOR;
    logger_.info("Setting advertisement mode to {}", advertisement_mode);
    // set the advertisement mode
    nearby_fp_client_SetAdvertisement(advertisement_mode);
  }

  /// Deinitialize the service
  void deinit() { gfps::deinit(); }

protected:
  static constexpr uint16_t SERVICE_UUID = 0xFE2C;
  static constexpr char *MODEL_ID_UUID = "fe2c1233-8366-4814-8eb0-01de32100bea";
  static constexpr char *KB_PAIRING_UUID = "fe2c1234-8366-4814-8eb0-01de32100bea";
  static constexpr char *PASSKEY_UUID = "fe2c1235-8366-4814-8eb0-01de32100bea";
  static constexpr char *ACCOUNT_KEY_UUID = "fe2c1236-8366-4814-8eb0-01de32100bea";

  void make_service(NimBLEServer *server) {
    if (service_) {
      logger_.error("Service already created");
      return;
    }

    // make the service
    service_ = server->createService(SERVICE_UUID);
    if (!service_) {
      logger_.error("Failed to create service");
      return;
    }
    logger_.info("Service created");

    // now make the characteristics
    model_id_ = service_->createCharacteristic(MODEL_ID_UUID, NIMBLE_PROPERTY::READ);
    if (!model_id_) {
      logger_.error("Failed to create model id characteristic");
      return;
    }
    // model id is a uint32_t but we only want the lower 24 bits, so we
    auto model_id_value = gfps::get_model_id();
    model_id_->setValue(reinterpret_cast<uint8_t *>(&model_id_value), 3);

    kb_pairing_ = service_->createCharacteristic(KB_PAIRING_UUID,
                                                 NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    if (!kb_pairing_) {
      logger_.error("Failed to create kb pairing characteristic");
      return;
    }

    passkey_ = service_->createCharacteristic(PASSKEY_UUID,
                                              NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::NOTIFY);
    if (!passkey_) {
      logger_.error("Failed to create passkey characteristic");
      return;
    }

    account_key_ = service_->createCharacteristic(ACCOUNT_KEY_UUID, NIMBLE_PROPERTY::WRITE);
    if (!account_key_) {
      logger_.error("Failed to create account key characteristic");
      return;
    }

    // set the characteristic callbacks
    model_id_->setCallbacks(new GfpsModelIdCharacteristicCallbacks());
    kb_pairing_->setCallbacks(new GfpsKbPairingCharacteristicCallbacks());
    passkey_->setCallbacks(new GfpsPasskeyCharacteristicCallbacks());
    account_key_->setCallbacks(new GfpsAccountKeyCharacteristicCallbacks());

    logger_.info("Characteristics created");
  }

  void set_service_data() {
    uint32_t model_id = gfps::get_model_id();
    // 3 bytes of the model id (lower 24 bits, big endian)
    service_data_[0] = model_id >> 16;
    service_data_[1] = model_id >> 8;
    service_data_[2] = model_id & 0xFF;
  }

  NimBLEService *service_{nullptr};
  NimBLECharacteristic *model_id_{nullptr};
  NimBLECharacteristic *kb_pairing_{nullptr};
  NimBLECharacteristic *passkey_{nullptr};
  NimBLECharacteristic *account_key_{nullptr};

  uint8_t service_data_[3] = {0x00, 0x00, 0x00};
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

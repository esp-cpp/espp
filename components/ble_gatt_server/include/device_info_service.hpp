#pragma once

#include <sdkconfig.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <string>
#include <vector>

#if CONFIG_BT_NIMBLE_ENABLED

#include "NimBLEDevice.h"

#include "base_component.hpp"

namespace espp {
/// Device Information Service
/// This class is responsible for creating and managing the Device Information Service.
///
/// The service is created with the following characteristics:
/// - Manufacturer Name (read)
/// - Model Number (read)
/// - Serial Number (read)
/// - Software Version (read)
/// - Firmware Version (read)
/// - Hardware Version (read)
/// - PnP ID (read)
///
/// The Device Information Service is a standard Bluetooth service that
/// provides information about the device. This information can be used by a
/// client to identify the device and determine its capabilities. The Device
/// Information Service is defined by the Bluetooth SIG and is intended to be
/// used with any device.
class DeviceInfoService : public BaseComponent {
public:
  /// Constructor
  /// \param log_level The log level for the component
  explicit DeviceInfoService(espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("DeviceInfoService", log_level) {}

  /// Initialize the Device Information Service
  /// \param server The BLE server to add the service to
  void init(NimBLEServer *server) { make_service(server); }

  /// Start the service
  /// \note This must be called after the service has been initialized
  void start() {
    if (!service_) {
      logger_.error("Service not created");
      return;
    }
    if (!service_->start()) {
      logger_.error("Failed to start service");
      return;
    }
  }

  /// Get the service object
  /// \return The service object
  NimBLEService *get_service() { return service_; }

  /// Get the UUID of the service
  /// \return The service UUID
  NimBLEUUID uuid() {
    if (service_) {
      return service_->getUUID();
    }
    return NimBLEUUID(SERVICE_UUID);
  }

  /// Set the PnP ID
  /// \param vendor_id_source The vendor ID source (0x01 for Bluetooth SIG, 0x02 for USB)
  /// \param vendor_id The vendor ID
  /// \param product_id The product ID
  /// \param product_version The product version
  /// \note The PnP ID is a standard Bluetooth characteristic that provides
  ///       information about the device's Plug and Play ID. This information
  ///       can be used by a client to identify the device and determine its
  ///       capabilities.
  void set_pnp_id(uint8_t vendor_id_source, uint16_t vendor_id, uint16_t product_id,
                  uint16_t product_version) {
    if (!pnp_) {
      logger_.error("Characteristic not created");
      return;
    }
    // format of pnp:
    // 0xVVVVPPPPIIIISS
    // VVVV: product version
    // PPPP: product id
    // IIII: vendor id
    // SS: vendor id source
    uint8_t pnp_id[7];
    pnp_id[0] = vendor_id_source;
    pnp_id[1] = vendor_id & 0xFF;
    pnp_id[2] = vendor_id >> 8;
    pnp_id[3] = product_id & 0xFF;
    pnp_id[4] = product_id >> 8;
    pnp_id[5] = product_version & 0xFF;
    pnp_id[6] = product_version >> 8;
    pnp_->setValue(pnp_id, sizeof(pnp_id));
  }

  /// Set the manufacturer name
  /// \param name The manufacturer name
  void set_manufacturer_name(const std::string &name) {
    if (!manufacturer_name_) {
      logger_.error("Characteristic not created");
      return;
    }
    manufacturer_name_->setValue(reinterpret_cast<const uint8_t *>(name.data()), name.size());
  }

  /// Set the model number
  /// \param number The model number
  void set_model_number(const std::string &number) {
    if (!model_number_) {
      logger_.error("Characteristic not created");
      return;
    }
    model_number_->setValue(reinterpret_cast<const uint8_t *>(number.data()), number.size());
  }

  /// Set the serial number
  /// \param number The serial number
  void set_serial_number(const std::string &number) {
    if (!serial_number_) {
      logger_.error("Characteristic not created");
      return;
    }
    serial_number_->setValue(reinterpret_cast<const uint8_t *>(number.data()), number.size());
  }

  /// Set the software version
  /// \param version The software version
  void set_software_version(const std::string &version) {
    if (!software_version_) {
      logger_.error("Characteristic not created");
      return;
    }
    software_version_->setValue(reinterpret_cast<const uint8_t *>(version.data()), version.size());
  }

  /// Set the firmware version
  /// \param version The firmware version
  void set_firmware_version(const std::string &version) {
    if (!firmware_version_) {
      logger_.error("Characteristic not created");
      return;
    }
    firmware_version_->setValue(reinterpret_cast<const uint8_t *>(version.data()), version.size());
  }

  /// Set the hardware version
  /// \param version The hardware version
  void set_hardware_version(const std::string &version) {
    if (!hardware_version_) {
      logger_.error("Characteristic not created");
      return;
    }
    hardware_version_->setValue(reinterpret_cast<const uint8_t *>(version.data()), version.size());
  }

protected:
  static constexpr uint16_t SERVICE_UUID = 0x180A;
  static constexpr uint16_t PNP_CHAR_UUID = 0x2A50;
  static constexpr uint16_t MANUFACTURER_NAME_CHAR_UUID = 0x2A29;
  static constexpr uint16_t MODEL_NUMBER_CHAR_UUID = 0x2A24;
  static constexpr uint16_t SERIAL_NUMBER_CHAR_UUID = 0x2A25;
  static constexpr uint16_t SOFTWARE_VERSION_CHAR_UUID = 0x2A28;
  static constexpr uint16_t FIRMWARE_VERSION_CHAR_UUID = 0x2A26;
  static constexpr uint16_t HARDWARE_VERSION_CHAR_UUID = 0x2A27;

  void make_service(NimBLEServer *server) {
    service_ = server->createService(NimBLEUUID(SERVICE_UUID));
    if (!service_) {
      logger_.error("Failed to create service");
      return;
    }

    pnp_ = service_->createCharacteristic(NimBLEUUID(PNP_CHAR_UUID), NIMBLE_PROPERTY::READ);

    manufacturer_name_ = service_->createCharacteristic(NimBLEUUID(MANUFACTURER_NAME_CHAR_UUID),
                                                        NIMBLE_PROPERTY::READ);

    model_number_ =
        service_->createCharacteristic(NimBLEUUID(MODEL_NUMBER_CHAR_UUID), NIMBLE_PROPERTY::READ);

    serial_number_ =
        service_->createCharacteristic(NimBLEUUID(SERIAL_NUMBER_CHAR_UUID), NIMBLE_PROPERTY::READ);

    software_version_ = service_->createCharacteristic(NimBLEUUID(SOFTWARE_VERSION_CHAR_UUID),
                                                       NIMBLE_PROPERTY::READ);

    firmware_version_ = service_->createCharacteristic(NimBLEUUID(FIRMWARE_VERSION_CHAR_UUID),
                                                       NIMBLE_PROPERTY::READ);

    hardware_version_ = service_->createCharacteristic(NimBLEUUID(HARDWARE_VERSION_CHAR_UUID),
                                                       NIMBLE_PROPERTY::READ);
  }

  NimBLEService *service_{nullptr};
  NimBLECharacteristic *pnp_{nullptr};
  NimBLECharacteristic *manufacturer_name_{nullptr};
  NimBLECharacteristic *model_number_{nullptr};
  NimBLECharacteristic *serial_number_{nullptr};
  NimBLECharacteristic *software_version_{nullptr};
  NimBLECharacteristic *firmware_version_{nullptr};
  NimBLECharacteristic *hardware_version_{nullptr};
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED

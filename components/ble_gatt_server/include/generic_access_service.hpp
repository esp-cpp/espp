#pragma once

#include <sdkconfig.h>

#include <string>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

#include "base_component.hpp"

namespace espp {
/// Generic Access Service
/// This class is responsible for creating and managing the Generic Access Service.
///
/// The service is created with the following characteristics:
/// - Name (read)
/// - Appearance (read)
///
/// The Generic Access Service is a required Bluetooth service that provides
/// information about the device. This information can be used by a client to
/// identify the device and determine its capabilities. The Generic Access
/// Service is defined by the Bluetooth SIG and is intended to be used with any
/// device.
///
/// NOTE: as a developer, you should not need to actually use this service
/// directly as the NimBLE stack will automatically include it in the device
/// information. This class is provided for completeness and for developers who
/// want to customize the service. I'm not really sure why I created this file,
/// but I have so here we are.
class GenericAccessService : public BaseComponent {
public:
  static constexpr uint16_t SERVICE_UUID = 0x1800;
  static constexpr uint16_t NAME_CHAR_UUID = 0x2A00;
  static constexpr uint16_t APPEARANCE_CHAR_UUID = 0x2A01;

  /// Parse the appearance value from raw bytes
  /// \param bytes The characteristic value
  /// \return The appearance value
  static uint16_t parse_appearance(const std::vector<uint8_t> &bytes) {
    return parse_appearance(bytes.data(), bytes.size());
  }

  /// Parse the appearance value from raw bytes
  /// \param bytes The characteristic value
  /// \param size The size of the characteristic value
  /// \return The appearance value
  static uint16_t parse_appearance(const uint8_t *bytes, size_t size) {
    if (size != 2) {
      return 0;
    }
    return (bytes[1] << 8) | bytes[0];
  }

  /// Constructor
  /// \param log_level The log level for the component
  explicit GenericAccessService(espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("GenericAccessService", log_level) {}

  /// Initialize the Service
  /// \param server The BLE server to add the service to
  void init(NimBLEServer *server) { make_service(server); }

  /// Deinitialize the Service
  /// \note This should only be called after NimBLEDevice::deinit(true) has been
  ///       called, since that will free the memory used by the service
  void deinit() {
    service_ = nullptr;
    name_ = nullptr;
    appearance_ = nullptr;
  }

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

  /// Set the device name
  /// \param name The device name
  void set_name(const std::string &name) {
    if (!name_) {
      logger_.error("Characteristic not created");
      return;
    }
    name_->setValue(name);
  }

  /// Set the device appearance
  /// \param appearance The appearance value
  void set_appearance(uint16_t appearance) {
    if (!appearance_) {
      logger_.error("Characteristic not created");
      return;
    }
    appearance_->setValue(appearance);
  }

  /// Get the device name
  /// \return The device name
  std::string get_name() {
    if (!name_) {
      logger_.error("Characteristic not created");
      return "";
    }
    return name_->getValue();
  }

  /// Get the device appearance
  /// \return The appearance value
  uint16_t get_appearance() {
    if (!appearance_) {
      logger_.error("Characteristic not created");
      return 0;
    }
    return parse_appearance(appearance_->getValue());
  }

protected:
  void make_service(NimBLEServer *server) {
    service_ = server->createService(NimBLEUUID(SERVICE_UUID));
    if (!service_) {
      logger_.error("Failed to create service");
      return;
    }

    name_ = service_->createCharacteristic(NimBLEUUID(NAME_CHAR_UUID), NIMBLE_PROPERTY::READ);

    appearance_ =
        service_->createCharacteristic(NimBLEUUID(APPEARANCE_CHAR_UUID), NIMBLE_PROPERTY::READ);
  }

  NimBLEService *service_{nullptr};
  NimBLECharacteristic *name_{nullptr};
  NimBLECharacteristic *appearance_{nullptr};
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#pragma once

#include <sdkconfig.h>

#include <algorithm>
#include <ctime>
#include <functional>
#include <string>
#include <vector>

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include "NimBLEDevice.h"

#include "base_component.hpp"

namespace espp {
/// Battery Service
/// This class is responsible for creating and managing the Battery Service.
///
/// The service is created with the following characteristics:
/// - Battery Level (read, notify, unencrypted)
///
/// The Battery Level characteristic is a standard characteristic defined by
/// the Bluetooth SIG. It is used to report the current battery level of the
/// device.
class BatteryService : public BaseComponent {
public:
  static constexpr uint8_t BATTERY_LEVEL_MAX = 100;           ///< Maximum battery level
  static constexpr uint16_t BATTERY_LEVEL_UNIT = 0x27AD;      ///< Unit is percentage
  static constexpr uint16_t BATTERY_SERVICE_UUID = 0x180F;    ///< Battery Service UUID
  static constexpr uint16_t BATTERY_LEVEL_CHAR_UUID = 0x2A19; ///< Battery Level Characteristic UUID

  /// Constructor
  /// \param log_level The log level for the component
  explicit BatteryService(espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : BaseComponent("BatteryService", log_level) {}

  /// Initialize the Battery Service
  /// \param server The BLE server to add the service to
  void init(NimBLEServer *server) { make_service(server); }

  /// Deinitialize the Battery Service
  /// \note This should only be called after NimBLEDevice::deinit(true) has been
  ///       called, since that will free the memory used by the service
  void deinit() {
    service_ = nullptr;
    battery_level_ = nullptr;
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

  /// Get the service
  /// \return The Battery Service
  NimBLEService *get_service() { return service_; }

  /// Get the UUID of the service
  /// \return The UUID of the service
  NimBLEUUID uuid() {
    if (service_) {
      return service_->getUUID();
    }
    return NimBLEUUID(BATTERY_SERVICE_UUID);
  }

  /// Set the battery level
  /// \param level The battery level
  /// \note The level is clamped to the range [0, 100]
  /// \note This must be called after the service has been initialized
  void set_battery_level(uint8_t level) {
    if (!service_) {
      logger_.error("Service not created");
      return;
    }
    if (!battery_level_) {
      logger_.error("Battery level characteristic not created");
      return;
    }
    // ensure the level is within the valid range
    level = std::min(level, BATTERY_LEVEL_MAX);
    battery_level_->setValue(&level, 1);
    battery_level_->notify();
  }

  /// Get the battery level
  /// \return The battery level
  uint8_t get_battery_level() {
    if (!service_) {
      logger_.error("Service not created");
      return 0;
    }
    if (!battery_level_) {
      logger_.error("Battery level characteristic not created");
      return 0;
    }
    return battery_level_->getValue().getValue<uint8_t>();
  }

protected:
  void make_service(NimBLEServer *server) {
    service_ = server->createService(NimBLEUUID(BATTERY_SERVICE_UUID));
    if (!service_) {
      logger_.error("Failed to create service");
      return;
    }

    battery_level_ = service_->createCharacteristic(
        NimBLEUUID(BATTERY_LEVEL_CHAR_UUID), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    auto battery_level_desc =
        reinterpret_cast<NimBLE2904 *>(battery_level_->createDescriptor((uint16_t)0x2904));
    battery_level_desc->setFormat(NimBLE2904::FORMAT_UINT8);
    battery_level_desc->setNamespace(1);
    battery_level_desc->setUnit(BATTERY_LEVEL_UNIT);
  }

  NimBLEService *service_{nullptr};
  NimBLECharacteristic *battery_level_{nullptr};
};
} // namespace espp

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

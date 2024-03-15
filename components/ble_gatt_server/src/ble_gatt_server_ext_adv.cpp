#include "ble_gatt_server.hpp"
#include "ble_gatt_server_callbacks.hpp"

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#if CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

using namespace espp;

// EXTENDED ADV

void BleGattServer::set_advertisement_data(const AdvertisedData &advertising_data,
                                           uint8_t instance) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Setting ext advertising data for instance {}", instance);

  // set the advertising data
  auto success =
      advertising->setInstanceData(instance, const_cast<AdvertisedData &>(advertising_data));
  if (!success) {
    logger_.error("Failed to set advertising data");
  }
}

void BleGattServer::set_scan_response_data(const AdvertisedData &scan_response_data,
                                           uint8_t instance) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Setting ext scan response data for instance {}", instance);

  // set the advertising data
  auto success =
      advertising->setScanResponseData(instance, const_cast<AdvertisedData &>(scan_response_data));
  if (!success) {
    logger_.error("Failed to set scan response data");
  }
}

void BleGattServer::stop_advertising(uint8_t instance) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Stopping ext advertising for instance {}", instance);

  advertising->stop(instance);
}

void BleGattServer::start_advertising(uint32_t duration_ms, uint8_t instance) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Starting ext advertising for instance {} with duration {} ms", instance,
               duration_ms);

  // set the callbacks
  advertising->setCallbacks(new BleGattServerAdvertisingCallbacks(this));

  // start advertising
  auto success = advertising->start(instance, duration_ms);
  if (!success) {
    logger_.error("Failed to start advertising for instance {}", instance);
  }
}

#endif // CONFIG_BT_NIMBLE_EXT_ADV || defined(_DOXYGEN_)

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

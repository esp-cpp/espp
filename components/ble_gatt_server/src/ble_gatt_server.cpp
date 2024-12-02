#include "ble_gatt_server.hpp"
#include "ble_gatt_server_callbacks.hpp"

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#if !CONFIG_BT_NIMBLE_EXT_ADV

using namespace espp;

// LEGACY ADV

void BleGattServer::set_advertisement_data(const AdvertisedData &advertising_data) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Setting legacy advertising data");

  // set the advertising data
  advertising->setAdvertisementData(const_cast<AdvertisedData &>(advertising_data));
}

void BleGattServer::set_scan_response_data(const AdvertisedData &scan_response_data) {
  if (!server_) {
    logger_.error("Server not created");
    return;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return;
  }

  logger_.info("Setting legacy scan response data");

  // set the advertising data
  advertising->setScanResponseData(const_cast<AdvertisedData &>(scan_response_data));
}

bool BleGattServer::start_advertising(const AdvertisingParameters &advertising_params) {
  if (!server_) {
    logger_.error("Server not created");
    return false;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return false;
  }

  logger_.info("Starting legacy advertising");
  logger_.debug("Advertising parameters: {}", advertising_params);

  auto max_interval_units = interval_ms_to_units(advertising_params.max_interval_ms);
  auto min_interval_units = interval_ms_to_units(advertising_params.min_interval_ms);

  // check if the interval is a multiple of 0.625 ms
  if (advertising_params.max_interval_ms != interval_units_to_ms(max_interval_units)) {
    logger_.warn(
        "Max advertising interval is not a multiple of 0.625 ms, rounding to 0x{:04X} ({} ms)",
        max_interval_units, interval_units_to_ms(max_interval_units));
  }
  if (advertising_params.min_interval_ms != interval_units_to_ms(min_interval_units)) {
    logger_.warn(
        "Min advertising interval is not a multiple of 0.625 ms, rounding to 0x{:04X} ({} ms)",
        min_interval_units, interval_units_to_ms(min_interval_units));
  }

  // configure the advertising parameters
  advertising->setMinInterval(min_interval_units);
  advertising->setMaxInterval(max_interval_units);
  advertising->enableScanResponse(advertising_params.scan_response);

  if (advertising_params.include_tx_power) {
    advertising->addTxPower();
  }

  // set the whitelist configuration
  advertising->setScanFilter(advertising_params.scan_request_whitelist,
                             advertising_params.connect_whitelist);

  // set the advertisement type based on the connectable flag and the directed
  // address
  if (advertising_params.connectable) {
    if (advertising_params.directed_address) {
      // directed, connectable
      advertising->setConnectableMode(BLE_GAP_CONN_MODE_DIR);
    } else {
      // undirected, connectable
      advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
    }
  } else {
    // non-connectable
    advertising->setConnectableMode(BLE_GAP_CONN_MODE_NON);
  }

  // set the callback
  advertising->setAdvertisingCompleteCallback(callbacks_.advertisement_complete_callback);

  // now actually start advertising
  bool success =
      advertising->start(advertising_params.duration_ms, advertising_params.directed_address);
  if (!success) {
    logger_.error("Failed to start advertising");
  }
  return success;
}

bool BleGattServer::start_advertising(uint32_t duration_ms, NimBLEAddress *directed_address) {
  if (!server_) {
    logger_.error("Server not created");
    return false;
  }
  auto advertising = NimBLEDevice::getAdvertising();
  if (!advertising) {
    logger_.error("Advertising not created");
    return false;
  }
  logger_.info("Starting legacy advertising for {} ms", duration_ms);
  // assume connectable
  if (directed_address) {
    // directed, connectable
    advertising->setConnectableMode(BLE_GAP_CONN_MODE_DIR);
  } else {
    // undirected, connectable
    advertising->setConnectableMode(BLE_GAP_CONN_MODE_UND);
  }

  // set the callback
  advertising->setAdvertisingCompleteCallback(callbacks_.advertisement_complete_callback);

  // now actually start advertising
  auto success = advertising->start(duration_ms, directed_address);
  if (!success) {
    logger_.error("Failed to start advertising");
  }
  return success;
}

#endif // !CONFIG_BT_NIMBLE_EXT_ADV

#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

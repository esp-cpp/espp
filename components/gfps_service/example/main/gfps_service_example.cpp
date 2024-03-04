#include <chrono>
#include <vector>

#include "ble_gatt_server.hpp"
#include "gfps_service.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Gfps Service Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [gfps service example]

  // NOTE: esp-nimble-cpp already depends on nvs_flash and initializes
  //       nvs_flash in the NimBLEDevice::init(), so we don't have to do that
  //       to store bonding info

  // create the GATT server
  espp::BleGattServer ble_gatt_server;
  std::string device_name = "ESP++ GFPS";
  ble_gatt_server.set_log_level(espp::Logger::Verbosity::INFO);
  ble_gatt_server.set_callbacks({
      .connect_callback = [&](NimBLEConnInfo &conn_info) { logger.info("Device connected"); },
      .disconnect_callback = [&](NimBLEConnInfo &conn_info) { logger.info("Device disconnected"); },
      .authentication_complete_callback =
          [&](NimBLEConnInfo &conn_info) { logger.info("Device authenticated"); },
  });
  ble_gatt_server.init(device_name);
  ble_gatt_server.set_advertise_on_disconnect(true);

  // let's set some security
  bool bonding = true;
  bool mitm = false;
  bool secure_connections = true;
  ble_gatt_server.set_security(bonding, mitm, secure_connections);
  // and some i/o and key config
  ble_gatt_server.set_io_capabilities(BLE_HS_IO_NO_INPUT_OUTPUT);
  ble_gatt_server.set_init_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  ble_gatt_server.set_resp_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  // let's create a GFPS service
  espp::GfpsService gfps_service;
  gfps_service.init(ble_gatt_server.server());

  // now that we've made the input characteristic, we can start the service
  gfps_service.start();
  ble_gatt_server.start_services(); // starts the device info service and battery service
  // NOTE: we could also directly start them ourselves if we wanted to
  //      control the order of starting the services
  // e.g.:
  // ble_gatt_server.battery_service().start();
  // ble_gatt_server.device_info_service().start();

  // now start the gatt server
  ble_gatt_server.start();

  // let's set some of the service data
  auto &battery_service = ble_gatt_server.battery_service();
  battery_service.set_battery_level(99);

  auto &device_info_service = ble_gatt_server.device_info_service();
  uint8_t vendor_source = 0x02; // USB
  uint16_t vid = 0x045E;        // Microsoft
  uint16_t pid = 0x02FD;        // Xbox One Controller
  uint16_t product_version = 0x0100;
  device_info_service.set_pnp_id(vendor_source, vid, pid, product_version);
  device_info_service.set_manufacturer_name("ESP-CPP");
  // NOTE: this is NOT required to be the same as the GFPS SKU Name
  device_info_service.set_model_number("espp-gfps-01");
  device_info_service.set_serial_number("1234567890");
  device_info_service.set_software_version("1.0.0");
  device_info_service.set_firmware_version("1.0.0");
  device_info_service.set_hardware_version("1.0.0");

  // now lets start advertising
  espp::BleGattServer::AdvertisingData adv_data = {
      .name = device_name,
      .appearance = 0x03C4, // Gamepad
      .services = {},
      .service_data =
          {// these are the service data that we want to advertise
           {gfps_service.uuid(), gfps_service.get_service_data()}},
  };
  espp::BleGattServer::AdvertisingParameters adv_params = {
      .include_tx_power = true, // needed for gfps
  };
  ble_gatt_server.start_advertising(adv_data, adv_params);

  // now lets update the battery level every second
  uint8_t battery_level = 99;
  while (true) {
    auto start = std::chrono::steady_clock::now();

    // update the battery level
    battery_service.set_battery_level(battery_level);
    battery_level = (battery_level % 100) + 1;

    // sleep
    std::this_thread::sleep_until(start + 1s);
  }
  //! [gfps service example]
}

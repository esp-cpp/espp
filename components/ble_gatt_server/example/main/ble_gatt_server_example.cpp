#include <chrono>
#include <vector>

#include "ble_gatt_server.hpp"
#include "ble_gatt_server_menu.hpp"
#include "cli.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BLE GATT Server Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [ble gatt server example]

  // NOTE: esp-nimble-cpp already depends on nvs_flash and initializes
  //       nvs_flash in the NimBLEDevice::init(), so we don't have to do that
  //       to store bonding info

  // create the GATT server
  espp::BleGattServer ble_gatt_server;
  std::string device_name = "Espp BLE GATT Server";
  ble_gatt_server.set_log_level(espp::Logger::Verbosity::INFO);
  ble_gatt_server.set_callbacks({
      .connect_callback = [&](NimBLEConnInfo &conn_info) { logger.debug("Device connected"); },
      .disconnect_callback =
          [&](NimBLEConnInfo &conn_info) { logger.debug("Device disconnected"); },
      .authentication_complete_callback =
          [&](NimBLEConnInfo &conn_info) { logger.debug("Device authenticated"); },
  });
  ble_gatt_server.init(device_name);
  ble_gatt_server.set_advertise_on_disconnect(true);

  // let's configure the security
  bool bonding = true;
  bool mitm = false;
  bool secure_connections = true;
  ble_gatt_server.set_security(bonding, mitm, secure_connections);
  // and some i/o and key config
  ble_gatt_server.set_io_capabilities(BLE_HS_IO_NO_INPUT_OUTPUT);
  ble_gatt_server.set_init_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  ble_gatt_server.set_resp_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  // you can create a service and add it to the server using
  // ble_gatt_server.server().addService()

  // now start the services
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
  uint8_t vendor_source = 0x01;
  uint16_t vid = 0xCafe;
  uint16_t pid = 0xFace;
  uint16_t product_version = 0x0100;
  device_info_service.set_pnp_id(vendor_source, vid, pid, product_version);
  device_info_service.set_manufacturer_name("ESP-CPP");
  device_info_service.set_model_number("esp-ble-01");
  device_info_service.set_serial_number("1234567890");
  device_info_service.set_software_version("1.0.0");
  device_info_service.set_firmware_version("1.0.0");
  device_info_service.set_hardware_version("1.0.0");

  // now lets start advertising
  espp::BleGattServer::AdvertisingData adv_data = {
      .name = device_name,
  };
  espp::BleGattServer::AdvertisingParameters adv_params = {};
  ble_gatt_server.start_advertising(adv_data, adv_params);

  // now lets update the battery level every second for a little while
  uint8_t battery_level = 99;
  for (int i = 0; i < 200; i++) {
    auto start = std::chrono::steady_clock::now();

    // update the battery level
    battery_service.set_battery_level(battery_level);
    battery_level = (battery_level % 100) + 1;

    // sleep
    std::this_thread::sleep_until(start + 1s);
  }

  // Now let's test and use the BLE menu (CLI)
  // turn off some of the logs so that it doesn't clutter up the CLI
  ble_gatt_server.set_log_level(espp::Logger::Verbosity::WARN);
  // and make the CLI
  auto ble_menu = espp::make_ble_gatt_server_menu(ble_gatt_server);
  cli::SetColor();
  cli::Cli cli(std::move(ble_menu));
  cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

  espp::Cli input(cli);
  input.SetInputHistorySize(10);
  input.Start(); // this will not return until the user enters the `exit` command.
                 //! [ble gatt server example]
}

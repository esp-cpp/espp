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

  {
    //! [ble gatt server example]

    // NOTE: esp-nimble-cpp already depends on nvs_flash and initializes
    //       nvs_flash in the NimBLEDevice::init(), so we don't have to do that
    //       to store bonding info

    // create the GATT server
    espp::BleGattServer ble_gatt_server;
    std::string device_name = "Espp BLE GATT Server";
    ble_gatt_server.set_log_level(espp::Logger::Verbosity::INFO);
    ble_gatt_server.set_callbacks({
        .connect_callback = [&](NimBLEConnInfo &conn_info) { logger.info("Device connected"); },
        .disconnect_callback = [&](auto &conn_info,
                                   auto reason) { logger.info("Device disconnected: {}", reason); },
        .authentication_complete_callback =
            [&](const NimBLEConnInfo &conn_info) { logger.info("Device authenticated"); },
        // NOTE: this is optional, if you don't provide this callback, it will
        // perform the exactly function as below:
        .get_passkey_callback =
            [&]() {
              logger.info("Getting passkey");
              return NimBLEDevice::getSecurityPasskey();
            },
        // NOTE: this is optional, if you don't provide this callback, it will
        // perform the exactly function as below:
        .confirm_passkey_callback =
            [&](const NimBLEConnInfo &conn_info, uint32_t passkey) {
              logger.info("Confirming passkey: {}", passkey);
              NimBLEDevice::injectConfirmPIN(conn_info,
                                             passkey == NimBLEDevice::getSecurityPasskey());
            },
    });
    ble_gatt_server.init(device_name);
#if !CONFIG_BT_NIMBLE_EXT_ADV
    // extended advertisement does not support automatically advertising on
    // disconnect
    ble_gatt_server.set_advertise_on_disconnect(true);
#endif

    // let's configure the security
    bool bonding = true;
    bool mitm = false;
    bool secure_connections = true;
    ble_gatt_server.set_security(bonding, mitm, secure_connections);
    // and some i/o and key config
    NimBLEDevice::setSecurityPasskey(123456);
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

    // set the advertising data
    espp::BleGattServer::AdvertisedData adv_data;
    // uint8_t flags = BLE_HS_ADV_F_DISC_LTD;
    uint8_t flags = BLE_HS_ADV_F_DISC_GEN;
    adv_data.setFlags(flags);
    adv_data.setName(device_name);
    adv_data.setAppearance((uint16_t)espp::BleAppearance::GENERIC_COMPUTER);
    adv_data.addTxPower();
    ble_gatt_server.set_advertisement_data(adv_data);

#if CONFIG_COMPILER_CXX_EXCEPTIONS
    // let's test and use the BLE menu (CLI)
    // turn off some of the logs so that it doesn't clutter up the CLI
    ble_gatt_server.set_log_level(espp::Logger::Verbosity::WARN);
    // and make the CLI
    auto ble_menu = espp::BleGattServerMenu(ble_gatt_server);
    cli::SetColor();
    cli::Cli cli(ble_menu.get());
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });

    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start(); // this will not return until the user enters the `exit` command.
#endif

    logger.info("Menu has finished, starting advertising");
    // The menu has finished, so let's go into a loop to keep the device running
    // now lets start advertising
    ble_gatt_server.start_advertising();

    logger.info("Waiting for connection...");
    // now lets update the battery level every second for a little while
    uint8_t battery_level = 99;
    bool was_connected = false;
    bool can_exit = false;
    int num_seconds_to_wait = 30;
    while (true) {
      auto start = std::chrono::steady_clock::now();

      // if we are now connected, but were not, then get the services
      if (ble_gatt_server.is_connected() && !was_connected) {
        was_connected = true;
        can_exit = true;
        auto connected_device_infos = ble_gatt_server.get_connected_device_infos();
        logger.info("Connected devices: {}", connected_device_infos.size());
        std::vector<std::string> connected_device_names;
        std::transform(connected_device_infos.begin(), connected_device_infos.end(),
                       std::back_inserter(connected_device_names),
                       [&](auto &info) { return ble_gatt_server.get_connected_device_name(info); });
        logger.info("            Names: {}", connected_device_names);
      } else if (!ble_gatt_server.is_connected()) {
        was_connected = false;
        if (can_exit) {
          logger.info("No longer connected, exiting");
          break;
        }
      }

      if (!ble_gatt_server.is_connected()) {
        logger.move_up();
        logger.clear_line();
        logger.info("Waiting for connection... {}s", --num_seconds_to_wait);
        if (num_seconds_to_wait == 0) {
          logger.info("No connection, exiting");
          break;
        }
        // sleep
        std::this_thread::sleep_until(start + 1s);
        continue;
      }

      // update the battery level
      battery_service.set_battery_level(battery_level);
      battery_level = (battery_level % 100) + 1;

      // sleep
      std::this_thread::sleep_until(start + 1s);
    }

    // we are done, so stop the server and deinit the BLE stack. NOTE: this will
    // automatically be called by ~BleGattServer(), but we call it here to show
    // manual control and to test calling it multiple times since the destructor
    // will be called immediately after this block ends
    ble_gatt_server.deinit();
    //! [ble gatt server example]
  }

  logger.info("Done");
  // now just sleep forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

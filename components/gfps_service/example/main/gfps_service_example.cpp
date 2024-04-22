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
      .disconnect_callback = [&](auto &conn_info,
                                 auto reason) { logger.info("Device disconnected: {}", reason); },
      .authentication_complete_callback =
          [&](NimBLEConnInfo &conn_info) { logger.info("Device authenticated"); },
      .get_passkey_callback = [&]() { return NimBLEDevice::getSecurityPasskey(); },
      .confirm_passkey_callback =
          [&](uint32_t passkey) {
            // NOTE: right now we have no way to do asynchronous passkey injection
            // (see: https://github.com/h2zero/esp-nimble-cpp/pull/117), so we
            // have to blindly return true here AND set the passkey since it will
            // be used by the GFPS BLE stack through the side channel.
            NimBLEDevice::setSecurityPasskey(passkey);
            return true;
          },
  });
  ble_gatt_server.init(device_name);
#if CONFIG_BT_NIMBLE_EXT_ADV
#error                                                                                             \
    "This example does not support extended advertising, as GFPS does not recognize ext advertisements"
#endif
#if !CONFIG_BT_NIMBLE_EXT_ADV
  ble_gatt_server.set_advertise_on_disconnect(true);
#endif

  // NOTE: we don't have to set security, since GFPS internally will set it to
  // what is required by the spec.

  // let's create a GFPS service
  espp::GfpsService gfps_service;
  gfps_service.set_log_level(espp::Logger::Verbosity::DEBUG);
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
  uint16_t vid = 0xCafe;
  uint16_t pid = 0xBabe;
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
  espp::BleGattServer::AdvertisedData adv_data;
  // uint8_t flags = BLE_HS_ADV_F_DISC_LTD;
  uint8_t flags = BLE_HS_ADV_F_DISC_GEN;
  adv_data.setFlags(flags);
  adv_data.setName(device_name);
  adv_data.setAppearance((uint16_t)espp::BleAppearance::GENERIC_DISPLAY);
  adv_data.setServiceData(gfps_service.uuid(), gfps_service.get_service_data());
  adv_data.addTxPower();
  ble_gatt_server.set_advertisement_data(adv_data);
  ble_gatt_server.start_advertising();

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

#include <chrono>
#include <vector>

#include "ble_gatt_server.hpp"
#include "gfps_service.hpp"

#include "hid-rp-xbox.hpp"
#include "hid_service.hpp"

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
      .connect_callback = [&](const NimBLEConnInfo &conn_info) { logger.info("Device connected"); },
      .disconnect_callback = [&](const auto &conn_info,
                                 auto reason) { logger.info("Device disconnected: {}", reason); },
      .authentication_complete_callback =
          [&](const NimBLEConnInfo &conn_info) { logger.info("Device authenticated"); },
      .get_passkey_callback = [&]() { return NimBLEDevice::getSecurityPasskey(); },
      .confirm_passkey_callback =
          [&](const NimBLEConnInfo &conn_info, uint32_t passkey) {
            // set the passkey here, so that GFPS can later compare against it
            // and inject confirmation/rejection
            NimBLEDevice::setSecurityPasskey(passkey);
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

  // for HID we need to set some security
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
  gfps_service.set_log_level(espp::Logger::Verbosity::DEBUG);
  gfps_service.init(ble_gatt_server.server());
  gfps_service.start();

  // let's make a hid service so that the android phone will pair and stay connected
  // HID Xbox controller
  uint16_t vid = 0x045E; // Microsoft
  uint16_t pid = 0x0B13; // Xbox One Controller (model 1708)
  uint16_t product_version = 0x0100;
  std::string software_version = "1.0.0";
  std::string firmware_version = "5.9.2709.0";
  std::string hardware_version = "1.0.0";
  std::string advertised_name = device_name;
  std::string manufacturer_name = "Microsoft";
  std::string model_number = "1708";
  std::string serial_number = "1234567890";

  using GamepadInput = espp::XboxGamepadInputReport<>;
  GamepadInput gamepad_input_report;
  static constexpr size_t num_buttons = GamepadInput::button_count;
  static constexpr uint8_t input_report_id = GamepadInput::ID;
  using namespace hid::page;
  using namespace hid::rdf;
  auto raw_descriptor = descriptor(usage_page<generic_desktop>(), usage(generic_desktop::GAMEPAD),
                                   collection::application(gamepad_input_report.get_descriptor()));
  // let's create a HID service
  espp::HidService hid_service;
  hid_service.init(ble_gatt_server.server());
  // configure it some
  uint8_t country_code = 0x00;
  uint8_t hid_info_flags = 0x01;
  hid_service.set_info(country_code, hid_info_flags);
  // Generate the report descriptor for the gamepad
  auto descriptor = std::vector<uint8_t>(raw_descriptor.begin(), raw_descriptor.end());
  // set the report map (vector of bytes)
  hid_service.set_report_map(descriptor);
  // use the HID service to make an input report characteristic
  [[maybe_unused]] auto input_report = hid_service.input_report(input_report_id);
  // now that we've made the input characteristic, we can start the service
  hid_service.start();

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

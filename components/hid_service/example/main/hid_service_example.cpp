#include <chrono>
#include <vector>

#include "ble_gatt_server.hpp"
#include "hid_service.hpp"

#include "hid-rp-gamepad.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Hid Service Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [hid service example]

  // NOTE: esp-nimble-cpp already depends on nvs_flash and initializes
  //       nvs_flash in the NimBLEDevice::init(), so we don't have to do that
  //       to store bonding info

  // create the GATT server
  espp::BleGattServer ble_gatt_server;
  std::string device_name = "ESP++ HID";
  ble_gatt_server.set_log_level(espp::Logger::Verbosity::INFO);
  ble_gatt_server.set_callbacks({
      .connect_callback = [&](NimBLEConnInfo &conn_info) { logger.info("Device connected"); },
      .disconnect_callback = [&](NimBLEConnInfo &conn_info) { logger.info("Device disconnected"); },
      .authentication_complete_callback =
          [&](NimBLEConnInfo &conn_info) { logger.info("Device authenticated"); },
  });
  ble_gatt_server.init(device_name);
  ble_gatt_server.set_advertise_on_disconnect(true);

  // for HID we need to set some security
  bool bonding = true;
  bool mitm = false;
  bool secure_connections = true;
  ble_gatt_server.set_security(bonding, mitm, secure_connections);
  // and some i/o and key config
  ble_gatt_server.set_io_capabilities(BLE_HS_IO_NO_INPUT_OUTPUT);
  ble_gatt_server.set_init_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  ble_gatt_server.set_resp_key_distribution(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);

  // let's create a HID service
  espp::HidService hid_service;
  hid_service.init(ble_gatt_server.server());

  // configure it some
  uint8_t country_code = 0x00;
  uint8_t hid_info_flags = 0x01;
  hid_service.set_info(country_code, hid_info_flags);

  static constexpr uint8_t report_id = 1;
  static constexpr size_t num_buttons = 15;
  static constexpr int joystick_min = 0;
  static constexpr int joystick_max = 65535;
  static constexpr int trigger_min = 0;
  static constexpr int trigger_max = 1024;

  using Gamepad = espp::GamepadReport<num_buttons, joystick_min, joystick_max, trigger_min,
                                      trigger_max, report_id>;
  Gamepad gamepad_input_report;

  // Generate the report descriptor for the gamepad
  auto descriptor = gamepad_input_report.get_descriptor();

  logger.info("Report Descriptor:");
  logger.info("  Size: {}", descriptor.size());
  logger.info("  Data: {::#02x}", descriptor);

  // set the report map (vector of bytes)
  hid_service.set_report_map(descriptor);

  // use the HID service to make an input report characteristic
  auto input_report = hid_service.input_report(report_id);

  // now that we've made the input characteristic, we can start the service
  hid_service.start();
  // starts the device info service and battery service, see
  // hid_service_example for more info
  ble_gatt_server.start_services();

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
  device_info_service.set_model_number("esp-hid-01");
  device_info_service.set_serial_number("1234567890");
  device_info_service.set_software_version("1.0.0");
  device_info_service.set_firmware_version("1.0.0");
  device_info_service.set_hardware_version("1.0.0");

  // now lets start advertising
  espp::BleGattServer::AdvertisingData adv_data = {
      .name = device_name,
      .appearance = 0x03C4, // Gamepad
      .services =
          {
              // these are the services that we want to advertise
              hid_service.uuid(), // hid service
          },
      .service_data =
          {
              // these are the service data that we want to advertise
          },
  };
  espp::BleGattServer::AdvertisingParameters adv_params = {};
  ble_gatt_server.start_advertising(adv_data, adv_params);

  // now lets update the battery level and send an input report every second
  uint8_t battery_level = 99;
  // change the gamepad inputs every second
  int button_index = 1;
  int hat_value = 1;
  while (true) {
    auto start = std::chrono::steady_clock::now();

    // update the battery level
    battery_service.set_battery_level(battery_level);
    battery_level = (battery_level % 100) + 1;

    // cycle through the possible d-pad states
    Gamepad::Hat hat = (Gamepad::Hat)hat_value;
    hat_value = hat_value % 8 + 1;
    // use the button index to set the position of the right joystick
    float angle = 2.0f * M_PI * button_index / num_buttons;

    gamepad_input_report.reset();
    gamepad_input_report.set_hat(hat);
    gamepad_input_report.set_button(button_index, true);
    // joystick inputs are in the range [-1, 1] float
    gamepad_input_report.set_right_joystick(cos(angle), sin(angle));
    gamepad_input_report.set_left_joystick(sin(angle), cos(angle));
    // trigger inputs are in the range [0, 1] float
    gamepad_input_report.set_accelerator(std::abs(sin(angle)));
    gamepad_input_report.set_brake(std::abs(cos(angle)));

    button_index = (button_index % num_buttons) + 1;

    // send an input report
    auto report = gamepad_input_report.get_report();
    logger.debug("Sending report data ({}): {::#02x}", report.size(), report);
    input_report->notify(report);

    // sleep
    std::this_thread::sleep_until(start + 1s);
  }
  //! [hid service example]
}

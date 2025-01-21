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
            NimBLEDevice::injectConfirmPasskey(conn_info,
                                               passkey == NimBLEDevice::getSecurityPasskey());
          },
  });
  ble_gatt_server.init(device_name);
#if CONFIG_BT_NIMBLE_EXT_ADV
#error                                                                                             \
    "This example does not support extended advertising, as iOS does not seem to show ext advertisements in their bluetooth settings menu (even if you turn on legacy advertising)"
#endif
#if !CONFIG_BT_NIMBLE_EXT_ADV
  ble_gatt_server.set_advertise_on_disconnect(true);
#endif

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

  static constexpr uint8_t input_report_id = 1;
  static constexpr uint8_t battery_report_id = 4;
  static constexpr size_t num_buttons = 15;
  static constexpr int joystick_min = 0;
  static constexpr int joystick_max = 65534;
  static constexpr int trigger_min = 0;
  static constexpr int trigger_max = 1023;

  using GamepadInput =
      espp::GamepadInputReport<num_buttons, std::uint16_t, std::uint16_t, joystick_min,
                               joystick_max, trigger_min, trigger_max, input_report_id>;
  GamepadInput gamepad_input_report;

  using BatteryReport = espp::XboxBatteryInputReport<battery_report_id>;
  BatteryReport battery_input_report;

  static constexpr uint8_t output_report_id = 2;
  static constexpr size_t num_leds = 4;
  using GamepadLeds = espp::GamepadLedOutputReport<num_leds, output_report_id>;
  GamepadLeds gamepad_leds_report;

  using namespace hid::page;
  using namespace hid::rdf;
  auto raw_descriptor = descriptor(usage_page<generic_desktop>(), usage(generic_desktop::GAMEPAD),
                                   collection::application(gamepad_input_report.get_descriptor(),
                                                           battery_input_report.get_descriptor(),
                                                           gamepad_leds_report.get_descriptor()));

  // Generate the report descriptor for the gamepad
  auto descriptor = std::vector<uint8_t>(raw_descriptor.begin(), raw_descriptor.end());

  logger.info("Report Descriptor:");
  logger.info("  Size: {}", descriptor.size());
  logger.info("  Data: {::#02x}", descriptor);

  // set the report map (vector of bytes)
  hid_service.set_report_map(descriptor);

  // use the HID service to make an input report characteristic
  [[maybe_unused]] auto input_report = hid_service.input_report(input_report_id);
  [[maybe_unused]] auto battery_report = hid_service.input_report(battery_report_id);

  // use the HID service to make an output report characteristic
  [[maybe_unused]] auto output_report = hid_service.output_report(output_report_id);

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

  // NOTE: iOS does not seem to show ext advertisements in their bluetooth
  // settings menu (even if you turn on legacy advertising)

  // now lets start advertising
  uint32_t advertise_duration_ms = 0; // 0 means never stop
  espp::BleGattServer::AdvertisedData adv_data;
  // uint8_t flags = BLE_HS_ADV_F_DISC_LTD;
  uint8_t flags = BLE_HS_ADV_F_DISC_GEN;
  adv_data.setFlags(flags);
  adv_data.setName(device_name);
  adv_data.setAppearance((uint16_t)espp::BleAppearance::GAMEPAD);
  adv_data.setPartialServices16({hid_service.uuid()});
  adv_data.addTxPower();
  ble_gatt_server.set_advertisement_data(adv_data);
  ble_gatt_server.start_advertising(advertise_duration_ms);

  // now lets update the battery level and send an input report every second
  uint8_t battery_level = 99;
  // change the gamepad inputs every second
  int button_index = 1;
  bool was_connected = false;
  while (true) {
    auto start = std::chrono::steady_clock::now();

    // if we are now connected, but were not, then get the services
    if (ble_gatt_server.is_connected() && !was_connected) {
      was_connected = true;
      auto connected_device_infos = ble_gatt_server.get_connected_device_infos();
      logger.info("Connected devices: {}", connected_device_infos.size());
      std::vector<std::string> connected_device_names;
      std::transform(connected_device_infos.begin(), connected_device_infos.end(),
                     std::back_inserter(connected_device_names),
                     [&](auto &info) { return ble_gatt_server.get_connected_device_name(info); });
      logger.info("            Names: {}", connected_device_names);
      std::vector<int> connected_device_rssis;
      std::transform(connected_device_infos.begin(), connected_device_infos.end(),
                     std::back_inserter(connected_device_rssis),
                     [&](auto &info) { return ble_gatt_server.get_connected_device_rssi(info); });
      logger.info("            RSSIs: {}", connected_device_rssis);
    } else if (!ble_gatt_server.is_connected()) {
      was_connected = false;
    }

    if (!ble_gatt_server.is_connected()) {
      // sleep
      std::this_thread::sleep_until(start + 1s);
      continue;
    }

    // update the battery level
    battery_service.set_battery_level(battery_level);
    battery_input_report.reset();
    battery_input_report.set_rechargeable(true);
    battery_input_report.set_charging(false);
    battery_input_report.set_battery_level(battery_level);
    battery_report->notify(battery_input_report.get_report());
    battery_level = (battery_level % 100) + 1;

    // cycle through the possible d-pad states
    GamepadInput::Hat hat = (GamepadInput::Hat)button_index;
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

    logger.debug("Setting left joystick: ({:.1f}, {:.1f})", sin(angle), cos(angle));
    logger.debug("Setting right joystick: ({:.1f}, {:.1f})", cos(angle), sin(angle));
    logger.debug("Setting brake: {:.1f}", std::abs(cos(angle)));
    logger.debug("Setting accelerator: {:.1f}", std::abs(sin(angle)));
    logger.debug("Setting hat: {}", (int)hat);
    logger.debug("Setting button: {}", button_index);

    button_index = (button_index % num_buttons) + 1;

    // send an input report
    auto report = gamepad_input_report.get_report();
    logger.debug("Sending report data ({}): {::#02x}", report.size(), report);

    // Get the stored pointer (we could use the one above, but this is just to
    // show how to get it)
    auto report_char = hid_service.input_report(input_report_id);
    report_char->notify(report);

    // sleep
    std::this_thread::sleep_until(start + 1s);
  }
  //! [hid service example]
}

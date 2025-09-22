#include <chrono>
#include <vector>

#include <esp_crc.h>

#include "ble_gatt_server.hpp"
#include "hid_service.hpp"

#include "hid-rp-playstation.hpp"
#include "hid-rp-switch-pro.hpp"
#include "hid-rp-xbox.hpp"

using namespace std::chrono_literals;

// make a callback for managing the pairing table when it fills up
class BleDeviceCallbacks : public NimBLEDeviceCallbacks {
  espp::Logger logger =
      espp::Logger({.tag = "NimBLEDeviceCallbacks", .level = espp::Logger::Verbosity::INFO});

public:
  /**
   * @brief Indicates an inability to perform a store operation.
   * This callback should do one of two things:
   *     -Address the problem and return 0, indicating that the store operation
   *      should proceed.
   *     -Return nonzero to indicate that the store operation should be aborted.
   * @param event     Describes the store event being reported.
   *                      BLE_STORE_EVENT_FULL; or
   *                      BLE_STORE_EVENT_OVERFLOW
   * @return          0 if the store operation should proceed;
   *                  nonzero if the store operation should be aborted.
   */
  virtual int onStoreStatus(struct ble_store_status_event *event, void *arg) {
    // see
    // https://github.com/apache/mynewt-nimble/blob/master/nimble/host/include/host/ble_store.h#L180
    // for definition of ble_store_status_event
    if (event->event_code == BLE_STORE_EVENT_FULL) {
      logger.info("Store full event: {}", event->event_code);
      // if the store is full, then we should delete some old devices
      // to make room for new ones
      //
      // the connection handle for the connection which prompted the write is
      // found at event->full->conn_handle.
      return ble_store_util_status_rr(event, arg);
    } else if (event->event_code == BLE_STORE_EVENT_OVERFLOW) {
      logger.info("Store overflow event: {}", event->event_code);
      // if the store overflows, then we should delete some old devices
      // to make room for new ones
      //
      // The object that failed to be written is found in
      // event->overflow->value (ble_store_value*)
      return ble_store_util_status_rr(event, arg);
    } else {
      logger.error("Unknown store event: {}", event->event_code);
      return ble_store_util_status_rr(event, arg);
    }
  }
};
static BleDeviceCallbacks device_callbacks;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Hid Service Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [hid service example]

  // NOTE: esp-nimble-cpp already depends on nvs_flash and initializes
  //       nvs_flash in the NimBLEDevice::init(), so we don't have to do that
  //       to store bonding info

#if CONFIG_EXAMPLE_AS_XBOX
  logger.info("Configuring as Xbox controller");
  // Xbox controller
  uint16_t vid = 0x045E; // Microsoft
  uint16_t pid = 0x0B13; // Xbox One Controller (model 1708)
  uint16_t product_version = 0x0100;
  std::string software_version = "1.0.0";
  std::string firmware_version = "5.9.2709.0";
  std::string hardware_version = "1.0.0";
  std::string device_name = "Xbox One Controller";
  std::string advertised_name = device_name;
  std::string manufacturer_name = "Microsoft";
  std::string model_number = "1708";
  std::string serial_number = "1234567890";

  using GamepadInput = espp::XboxGamepadInputReport<>;
  GamepadInput gamepad_input_report;
  static constexpr size_t num_buttons = GamepadInput::button_count;
  static constexpr uint8_t input_report_id = GamepadInput::ID;

  using BatteryReport = espp::XboxBatteryInputReport<>;
  BatteryReport battery_input_report;
  static constexpr uint8_t battery_report_id = BatteryReport::ID;

  static constexpr uint8_t led_output_report_id = 2;
  static constexpr size_t num_leds = 4;
  using GamepadLeds = espp::GamepadLedOutputReport<num_leds, led_output_report_id>;
  GamepadLeds gamepad_leds_report;

  using RumbleReport = espp::XboxRumbleOutputReport<>;
  RumbleReport gamepad_rumble_report;
  static constexpr uint8_t rumble_output_report_id = RumbleReport::ID;

  using namespace hid::page;
  using namespace hid::rdf;
  auto raw_descriptor = descriptor(usage_page<generic_desktop>(), usage(generic_desktop::GAMEPAD),
                                   collection::application(gamepad_input_report.get_descriptor(),
                                                           gamepad_rumble_report.get_descriptor(),
                                                           battery_input_report.get_descriptor(),
                                                           gamepad_leds_report.get_descriptor()));

#elif CONFIG_EXAMPLE_AS_SWITCH_PRO
  logger.info("Configuring as Switch Pro controller");

  // Switch Pro controller
  uint16_t vid = 0x057E; // Nintendo
  uint16_t pid = 0x2009; // Switch Pro Controller
  uint16_t product_version = 0x0100;
  std::string software_version = "1.0.0";
  std::string firmware_version = "1.0.0";
  std::string hardware_version = "1.0.0";
  std::string device_name = "Pro Controller";
  std::string advertised_name = device_name;
  std::string manufacturer_name = "Nintendo";
  std::string model_number = "HAC-013";
  std::string serial_number = "1234567890";

  using GamepadInput = espp::SwitchProGamepadInputReport<>;
  GamepadInput gamepad_input_report;
  static constexpr size_t num_report_bytes = GamepadInput::num_data_bytes;
  logger.info("Report size: {} bytes", num_report_bytes);
  static constexpr size_t num_buttons = GamepadInput::button_count;
  static constexpr uint8_t input_report_id = GamepadInput::ID;
  auto raw_descriptor = espp::switch_pro_descriptor();
#elif CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE
  logger.info("Configuring as Playstation Dualsense BLE controller");

  // Playstation Dualsense BLE controller
  uint16_t vid = 0x054C; // Sony
  uint16_t pid = 0x0CE6; // DualSense Wireless Controller
  uint16_t product_version = 0x0100;
  std::string software_version = "1.0.0";
  std::string firmware_version = "1.0.0";
  std::string hardware_version = "1.0.0";
  std::string device_name = "Dualsense Wireless Controller";
  std::string advertised_name = "Wireless Controller"; // must be shorter to fit in adv packet
  std::string manufacturer_name = "Sony";
  std::string model_number = "CFI-ZCT1W";
  std::string serial_number = "1234567890";

  using GamepadInput = espp::PlaystationDualsenseBLEComplexInputReport<>;
  GamepadInput gamepad_input_report;
  static constexpr size_t num_report_bytes = GamepadInput::num_data_bytes;
  logger.info("Report size: {} bytes", num_report_bytes);
  static constexpr size_t num_buttons = GamepadInput::button_count;
  static constexpr uint8_t input_report_id = GamepadInput::ID;
  auto raw_descriptor = espp::playstation_dualsense_ble_descriptor();
#else
#error                                                                                             \
    "You must define one of CONFIG_EXAMPLE_AS_XBOX, CONFIG_EXAMPLE_AS_SWITCH_PRO, or CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE"
#endif

  // create the GATT server
  espp::BleGattServer ble_gatt_server;
  ble_gatt_server.set_log_level(espp::Logger::Verbosity::INFO);
  ble_gatt_server.set_callbacks({
      .connect_callback = [&](const NimBLEConnInfo &conn_info) { logger.info("Device connected"); },
      .disconnect_callback = [&](const auto &conn_info,
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

  // NOTE: for information about the low-level NVS storage on esp chips, see
  // ~/esp/esp-idf/components/bt/host/nimble/nimble/nimble/host/store/config/src/ble_store_nvs.c
  //
  // now set the device callbacks to override the default one from esp-nimble-cpp
  NimBLEDevice::setDeviceCallbacks(&device_callbacks);
  uint8_t max_bonds = MYNEWT_VAL(BLE_STORE_MAX_BONDS);
  logger.info("Max bonds: {}", max_bonds);

  // print the bonded devices as well
  auto paired_device_addresses = ble_gatt_server.get_paired_devices();
  logger.info("Paired devices: {}", paired_device_addresses.size());
  for (const auto &addr : paired_device_addresses) {
    logger.info("          Addr:  {}", addr.toString());
  }

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

  // Generate the report descriptor for the gamepad
  auto descriptor = std::vector<uint8_t>(raw_descriptor.begin(), raw_descriptor.end());

  logger.info("Report Descriptor:");
  logger.info("  Size: {}", descriptor.size());
  logger.info("  Data: {::#02x}", descriptor);

  // set the report map (vector of bytes)
  hid_service.set_report_map(descriptor);

  // Only configure the battery, led, or rumble reports if we are emulating an
  // xbox controller
#if CONFIG_EXAMPLE_AS_XBOX
  // use the HID service to make an input report characteristic
  [[maybe_unused]] auto input_report = hid_service.input_report(input_report_id);
  auto battery_report = hid_service.input_report(battery_report_id);

  // use the HID service to make an output report characteristic
  auto led_output_report = hid_service.output_report(led_output_report_id);
  auto rumble_output_report = hid_service.output_report(rumble_output_report_id);

  // now make some characteristic callbacks for the Rumble Output characteristic
  // and the LED output characteristic
  class RumbleCallbacks : public NimBLECharacteristicCallbacks {
    RumbleReport *report = nullptr;
    espp::Logger logger =
        espp::Logger({.tag = "RumbleCallbacks", .level = espp::Logger::Verbosity::INFO});

  public:
    explicit RumbleCallbacks(RumbleReport *report)
        : report(report) {}

    virtual void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      // parse the output using the report
      auto data = pCharacteristic->getValue();
      if (!report) {
        return;
      }
      report->set_data(data);
      // now print the state of the report
      uint8_t enabled = report->get_enabled();
      uint8_t left_motor = report->get_magnitude(0);
      uint8_t right_motor = report->get_magnitude(1);
      uint8_t left_trigger = report->get_magnitude(2);
      uint8_t right_trigger = report->get_magnitude(3);
      uint8_t duration = report->get_duration();
      uint8_t delay = report->get_start_delay();
      uint8_t loop_count = report->get_loop_count();
      logger.info("enabled: 0x{:02x}, left_motor: {}, right_motor: {}, left_trigger: {}, "
                  "right_trigger: {}, duration: {}, delay: {}, loop_count: {}",
                  enabled, left_motor, right_motor, left_trigger, right_trigger, duration, delay,
                  loop_count);
    }
  };
  class LEDCallbacks : public NimBLECharacteristicCallbacks {
    GamepadLeds *report = nullptr;
    espp::Logger logger =
        espp::Logger({.tag = "LEDCallbacks", .level = espp::Logger::Verbosity::INFO});

  public:
    explicit LEDCallbacks(GamepadLeds *report)
        : report(report) {}

    virtual void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      // parse the output using the report
      auto data = pCharacteristic->getValue();
      if (!report) {
        return;
      }
      report->set_data(data);
      // now print the state of the report
      for (int i = 0; i < report->num_leds; i++) {
        bool enabled = report->get_led(i);
        logger.info("LED {}: {}", i, enabled);
      }
    }
  };
  RumbleCallbacks rumble_callbacks(&gamepad_rumble_report);
  LEDCallbacks led_callbacks(&gamepad_leds_report);
  rumble_output_report->setCallbacks(&rumble_callbacks);
  led_output_report->setCallbacks(&led_callbacks);
#elif CONFIG_EXAMPLE_AS_SWITCH_PRO
  // use the HID service to make an input report characteristic
  [[maybe_unused]] auto input_report = hid_service.input_report(input_report_id);
#elif CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE
  // use the HID service to make an input report characteristic
  [[maybe_unused]] auto input_report_01 =
      hid_service.input_report(1); // only one sent over ble before reading feature 05
  [[maybe_unused]] auto input_report =
      hid_service.input_report(input_report_id); // full report sent after reading feature 05

  using SimpleInputReport = espp::PlaystationDualsenseBLESimpleInputReport<>;
  SimpleInputReport simple_input_report;

  // go ahead and set the input reports for id 1 and id 49 so that they're ready to be read
  input_report_01->setValue(std::vector<uint8_t>(SimpleInputReport::num_data_bytes, 0));
  input_report->setValue(std::vector<uint8_t>(GamepadInput::num_data_bytes, 0));

  logger.info("Set input report {} to {}", simple_input_report.ID, simple_input_report);

  // now make some characteristic callbacks for when the output report is
  // written to
  class OutputCallbacks : public NimBLECharacteristicCallbacks {
    uint8_t report_id;
    espp::Logger logger =
        espp::Logger({.tag = "OutputCallbacks", .level = espp::Logger::Verbosity::INFO});

  public:
    explicit OutputCallbacks(uint8_t id)
        : report_id(id) {}
    virtual void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      // parse the output using the report
      auto data = pCharacteristic->getValue();
      logger.info("Output report (id={})[len={}]: {::#02x}", report_id, data.size(), data);
    }
  };
  // std::array<uint8_t, 9> output_report_ids = {49, 50, 51, 52, 53, 54, 55, 56, 57};
  std::array<uint8_t, 1> output_report_ids = {49};
  for (auto report_id : output_report_ids) {
    auto ps_output_report = hid_service.output_report(report_id);
    auto output_callbacks = std::make_unique<OutputCallbacks>(report_id);
    ps_output_report->setCallbacks(output_callbacks.get());
    output_callbacks.release(); // the hid service will now own the callbacks
  }

  // now make some characteristic callbacks for when the feature report is
  // written to
  class FeatureCallbacks : public NimBLECharacteristicCallbacks {
    uint8_t report_id;
    espp::Logger logger =
        espp::Logger({.tag = "FeatureCallbacks", .level = espp::Logger::Verbosity::INFO});

  public:
    explicit FeatureCallbacks(uint8_t id)
        : report_id(id) {}
    virtual void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      // parse the feature using the report
      auto data = pCharacteristic->getValue();
      logger.info("Feature report (id={})[len={}]: {::#02x}", report_id, data.size(), data);
    }
  };
  // std::array<uint8_t, 12> feature_report_ids = {5, 8, 9, 32, 34, 128, 129, 130, 131, 241, 242,
  // 240};
  std::array<uint8_t, 2> feature_report_ids = {5, 8};
  for (auto report_id : feature_report_ids) {
    auto ps_feature_report = hid_service.feature_report(report_id);
    auto feature_callbacks = std::make_unique<FeatureCallbacks>(report_id);
    ps_feature_report->setCallbacks(feature_callbacks.get());
    feature_callbacks.release(); // the hid service will now own the callbacks
  }
#endif // CONFIG_EXAMPLE_AS_XBOX

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
  device_info_service.set_pnp_id(vendor_source, vid, pid, product_version);
  device_info_service.set_manufacturer_name(manufacturer_name);
  device_info_service.set_model_number(model_number);
  device_info_service.set_serial_number(serial_number);
  device_info_service.set_software_version(software_version);
  device_info_service.set_firmware_version(firmware_version);
  device_info_service.set_hardware_version(hardware_version);

  // NOTE: iOS does not seem to show ext advertisements in their bluetooth
  // settings menu (even if you turn on legacy advertising)

  // set the advertisement data
  espp::BleGattServer::AdvertisedData adv_data;
  // uint8_t flags = BLE_HS_ADV_F_DISC_LTD;
  uint8_t flags = BLE_HS_ADV_F_DISC_GEN;
  adv_data.setFlags(flags);
  adv_data.setName(advertised_name);
  // adv_data.setAppearance((uint16_t)espp::BleAppearance::GAMEPAD);
  adv_data.setPartialServices16({hid_service.uuid()});
  // adv_data.addTxPower();
  ble_gatt_server.set_advertisement_data(adv_data);

  // set scan response data to include the name
  espp::BleGattServer::AdvertisedData scan_response_data;
  scan_response_data.setFlags(flags);
  // Ensure the name is set correctly in the scan response
  // This is critical for proper device identification during scanning
  scan_response_data.setName(device_name);
  // Add appearance to scan response for consistency
  scan_response_data.setAppearance((uint16_t)espp::BleAppearance::GAMEPAD);
  // Add partial service list to scan response
  scan_response_data.setPartialServices16({hid_service.uuid()});
  ble_gatt_server.set_scan_response_data(scan_response_data);

  // now lets start advertising
  espp::BleGattServer::AdvertisingParameters adv_params;
  adv_params.duration_ms = 0; // never stop
  // Ensure scan response is enabled
  adv_params.scan_response = true;
  ble_gatt_server.start_advertising(adv_params);

  // now lets update the battery level and send an input report every second
  uint8_t battery_level = 99;
  // change the gamepad inputs every second
  int button_index = 1;
  bool was_connected = false;
  while (true) {
    auto start = std::chrono::steady_clock::now();

    // if we are now connected + bonded, but were not, then get the services
    if (ble_gatt_server.is_connected() && !was_connected) {
      auto connected_device_infos = ble_gatt_server.get_connected_device_infos();
      // check to make sure the first connection has bonded at least.
      //
      // NOTE: if we are not bonded, then the name that will be read out will be
      // generic, such as "iPhone". If we are bonded, then the name will be the
      // actual device name, such as "iPhone 14 Plus William".
      const auto &first_device = connected_device_infos.front();
      if (first_device.isBonded()) {
        // if it was, mark connected and print all the device infos
        was_connected = true;
        logger.info("Connected devices: {}", connected_device_infos.size());
        for (const auto &info : connected_device_infos) {
          auto rssi = ble_gatt_server.get_connected_device_rssi(info);
          auto name = ble_gatt_server.get_connected_device_name(info);
          auto mfg_name = ble_gatt_server.get_connected_device_manufacturer_name(info);
          auto model_number = ble_gatt_server.get_connected_device_model_number(info);
          auto pnp_id = ble_gatt_server.get_connected_device_pnp_id(info);
          logger.info("            RSSI:  {}", rssi);
          logger.info("            Name:  {}", name);
          // NOTE: these are optionals, so they may not be set
          logger.info("            Mfg:   {}", mfg_name);
          logger.info("            Model: {}", model_number);
          logger.info("            PnP:   {}", pnp_id);
        }

        // print the bonded devices as well
        paired_device_addresses = ble_gatt_server.get_paired_devices();
        logger.info("Paired devices: {}", paired_device_addresses.size());
        for (const auto &addr : paired_device_addresses) {
          logger.info("          Addr:  {}", addr.toString());
        }
      }
    } else if (!ble_gatt_server.is_connected()) {
      was_connected = false;
    }

    if (!ble_gatt_server.is_connected()) {
      // sleep
      std::this_thread::sleep_until(start + 1s);
      continue;
    }

    // reset the input report
    gamepad_input_report.reset();

    // update the battery level
    logger.debug("Setting battery level: {}", battery_level);
    battery_service.set_battery_level(battery_level);
#if CONFIG_EXAMPLE_AS_XBOX
    battery_input_report.reset();
    battery_input_report.set_rechargeable(true);
    battery_input_report.set_charging(false);
    battery_input_report.set_battery_level(battery_level);
    battery_report->notify(battery_input_report.get_report());
#elif CONFIG_EXAMPLE_AS_SWITCH_PRO
    // switch pro doesn't have a battery input report
#elif CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE
    gamepad_input_report.set_battery_level(battery_level);
#endif // CONFIG_EXAMPLE_AS_XBOX
    battery_level = (battery_level % 100) + 1;

    // update the report counter (if any)
#if CONFIG_EXAMPLE_AS_XBOX
    // xbox doesn't have a report counter
#elif CONFIG_EXAMPLE_AS_SWITCH_PRO
    gamepad_input_report.increment_counter();
#elif CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE
    gamepad_input_report.increment_timestamp();
    gamepad_input_report.increment_counter();
    gamepad_input_report.increment_coarse_timestamp();
    gamepad_input_report.increment_fine_timestamps();
#endif

    static uint8_t hat_value = 0;
    static constexpr uint8_t max_hat_value = 9;
    // cycle through the possible d-pad states
    GamepadInput::Hat hat = (GamepadInput::Hat)(hat_value);
    hat_value = (hat_value + 1) % max_hat_value;
    // use the button index to set the position of the right joystick
    float angle = 2.0f * M_PI * button_index / num_buttons;

    gamepad_input_report.set_hat(hat);
    gamepad_input_report.set_button(button_index, true);
    // joystick inputs are in the range [-1, 1] float
    gamepad_input_report.set_right_joystick(cos(angle), sin(angle));
    gamepad_input_report.set_left_joystick(sin(angle), cos(angle));
    // trigger inputs are in the range [0, 1] float
    gamepad_input_report.set_right_trigger((float)std::abs(sin(angle)));
    gamepad_input_report.set_left_trigger((float)std::abs(cos(angle)));

    logger.debug("Setting left joystick: ({:.1f}, {:.1f})", sin(angle), cos(angle));
    logger.debug("Setting right joystick: ({:.1f}, {:.1f})", cos(angle), sin(angle));
    logger.debug("Setting brake: {:.1f}", std::abs(cos(angle)));
    logger.debug("Setting accelerator: {:.1f}", std::abs(sin(angle)));
    logger.debug("Setting hat: {}", (int)hat);
    logger.debug("Setting button: {}", button_index);

#if CONFIG_EXAMPLE_AS_XBOX
    static bool consumer_record = false;
    gamepad_input_report.set_consumer_record(consumer_record);
    logger.debug("Setting consumer record: {}", consumer_record);
    consumer_record = !consumer_record;
#endif // CONFIG_EXAMPLE_AS_XBOX

    button_index = (button_index % num_buttons) + 1;

    // send an input report
    auto report = gamepad_input_report.get_report();

#if CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE
    // calculate and set the CRC32 at the end of the report
    [[maybe_unused]] static constexpr uint8_t ps_input_crc32_seed = 0xA1;
    [[maybe_unused]] static constexpr uint8_t ps_output_crc32_seed = 0xA2;
    [[maybe_unused]] static constexpr uint8_t ps_feature_crc32_seed = 0xA3;
    // our CRC has to include the seed and the report_id, so make a new array
    uint8_t report_id = input_report_id;
    uint8_t seed = ps_input_crc32_seed;
    uint32_t crc = 0xFFFFFFFF;
    crc = esp_crc32_le(~crc, &seed, 1);
    crc = esp_crc32_le(crc, &report_id, 1);
    crc = esp_crc32_le(crc, report.data(), report.size() - 4);
    report[report.size() - 4] = (crc >> 0) & 0xFF;
    report[report.size() - 3] = (crc >> 8) & 0xFF;
    report[report.size() - 2] = (crc >> 16) & 0xFF;
    report[report.size() - 1] = (crc >> 24) & 0xFF;
    logger.debug("Setting CRC32: 0x{:08x}", crc);
#endif // CONFIG_EXAMPLE_AS_PLAYSTATION_DUALSENSE_BLE

    logger.debug("Set input report {} to {}", gamepad_input_report.ID, gamepad_input_report);
    logger.debug("Sending report data ({}): {::#02x}", report.size(), report);

    // Get the stored pointer (we could use the one above, but this is just to
    // show how to get it)
    auto report_char = hid_service.input_report(input_report_id);
    report_char->notify(report);
    // Could also just do:
    // input_report->notify(report);

    // sleep
    std::this_thread::sleep_until(start + 500ms);
  }
  //! [hid service example]
}

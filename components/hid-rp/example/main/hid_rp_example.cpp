#include <chrono>
#include <vector>

#include "logger.hpp"

#include "hid-rp-gamepad.hpp"
#include "hid-rp-playstation.hpp"
#include "hid-rp-ps4.hpp"
#include "hid-rp-switch-pro.hpp"
#include "hid-rp-xbox.hpp"
#include "hid-rp.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Hid RP Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  //! [hid rp example]
  static constexpr uint8_t input_report_id = 1;
  static constexpr uint8_t battery_report_id = 4;
  static constexpr size_t num_buttons = 15;
  static constexpr int joystick_min = 0;
  static constexpr int joystick_max = 65535;
  static constexpr int trigger_min = 0;
  static constexpr int trigger_max = 1023;

  using GamepadInput =
      espp::GamepadInputReport<num_buttons, std::uint16_t, std::uint16_t, joystick_min,
                               joystick_max, trigger_min, trigger_max, input_report_id>;
  GamepadInput gamepad_input_report;

  using XboxInput = espp::XboxGamepadInputReport<input_report_id>;
  XboxInput xbox_input_report;

  using BatteryReport = espp::XboxBatteryInputReport<battery_report_id>;
  BatteryReport battery_input_report;

  static constexpr uint8_t led_output_report_id = 2;
  static constexpr size_t num_leds = 4;
  using GamepadLeds = espp::GamepadLedOutputReport<num_leds, led_output_report_id>;
  GamepadLeds gamepad_leds_report;

  static constexpr uint8_t rumble_output_report_id = 3;
  using RumbleReport = espp::XboxRumbleOutputReport<rumble_output_report_id>;
  RumbleReport rumble_output_report;

  using namespace hid::page;
  using namespace hid::rdf;
  auto raw_descriptor = descriptor(usage_page<generic_desktop>(), usage(generic_desktop::GAMEPAD),
                                   collection::application(gamepad_input_report.get_descriptor(),
                                                           rumble_output_report.get_descriptor(),
                                                           battery_input_report.get_descriptor(),
                                                           gamepad_leds_report.get_descriptor()));

  // Generate the report descriptor for the gamepad
  auto descriptor = std::vector<uint8_t>(raw_descriptor.begin(), raw_descriptor.end());

  logger.info("Report Descriptor:");
  logger.info("  Size: {}", descriptor.size());
  logger.info("  Data: {::#04X}", descriptor);

  using SwitchProInput = espp::SwitchProGamepadInputReport<>;
  SwitchProInput switch_pro_input_report;
  logger.info("{}", switch_pro_input_report);
  logger.info("Switch Pro Input Report Size: {}", switch_pro_input_report.get_report().size());
  logger.info("Switch Pro Input Report Data: {::#04X}", switch_pro_input_report.get_report());

  auto sp_raw_descriptor = espp::switch_pro_descriptor();
  auto sp_descriptor = std::vector<uint8_t>(sp_raw_descriptor.begin(), sp_raw_descriptor.end());

  logger.info("Switch Report Descriptor:");
  logger.info("  Size: {}", sp_descriptor.size());
  std::string str = "";
  for (auto &byte : sp_descriptor) {
    str += fmt::format("0x{:02X}, ", byte);
  }
  logger.info("  Data: [{}]", str);

  using PlaystationDualsenseBleSimpleInput = espp::PlaystationDualsenseBLESimpleInputReport<>;
  PlaystationDualsenseBleSimpleInput dualsense_simple_input_report;
  using PlaystationDualsenseBleComplexInput = espp::PlaystationDualsenseBLEComplexInputReport<>;
  PlaystationDualsenseBleComplexInput dualsense_complex_input_report;
  logger.info("Playstation Dualsense BLE Report Descriptor:");
  auto ps_raw_descriptor = espp::playstation_dualsense_ble_descriptor();
  auto ps_descriptor = std::vector<uint8_t>(ps_raw_descriptor.begin(), ps_raw_descriptor.end());
  logger.info("  Size: {}", ps_descriptor.size());
  str = "";
  for (auto &byte : ps_descriptor) {
    str += fmt::format("0x{:02X}, ", byte);
  }
  logger.info("  Data: [{}]", str);

  using PS4DualShock4Input = espp::PS4DualShock4GamepadInputReport<>;
  PS4DualShock4Input ps4_input_report;
  logger.info("{}", ps4_input_report);
  logger.info("PS4 DualShock 4 Input Report Size: {}", ps4_input_report.get_report().size());
  logger.info("PS4 DualShock 4 Input Report Data: {::#04X}", ps4_input_report.get_report());

  auto ps4_raw_descriptor = espp::ps4_dualshock4_descriptor();
  auto ps4_descriptor = std::vector<uint8_t>(ps4_raw_descriptor.begin(), ps4_raw_descriptor.end());

  logger.info("PS4 DualShock 4 Report Descriptor:");
  logger.info("  Size: {}", ps4_descriptor.size());
  str = "";
  for (auto &byte : ps4_descriptor) {
    str += fmt::format("0x{:02X}, ", byte);
  }
  logger.info("  Data: [{}]", str);

  GamepadInput::Hat hat = GamepadInput::Hat::UP_RIGHT;
  int button_index = 5;
  float angle = 2.0f * M_PI * button_index / num_buttons;

  // reset all reports
  gamepad_input_report.reset();
  xbox_input_report.reset();
  switch_pro_input_report.reset();
  dualsense_simple_input_report.reset();
  dualsense_complex_input_report.reset();
  ps4_input_report.reset();

  // print out the reports in their default states
  logger.info("{}", gamepad_input_report);
  logger.info("{}", xbox_input_report);
  logger.info("{}", switch_pro_input_report);
  logger.info("{}", dualsense_simple_input_report);
  logger.info("{}", dualsense_complex_input_report);
  logger.info("{}", ps4_input_report);

  // update the gamepad input report
  logger.info("{}", gamepad_input_report);
  gamepad_input_report.set_hat(hat);
  gamepad_input_report.set_button(button_index, true);
  // joystick inputs are in the range [-1, 1] float
  gamepad_input_report.set_right_joystick(cos(angle), sin(angle));
  gamepad_input_report.set_left_joystick(sin(angle), cos(angle));
  // trigger inputs are in the range [0, 1] float
  gamepad_input_report.set_accelerator(std::abs(sin(angle)));
  gamepad_input_report.set_brake(std::abs(cos(angle)));

  switch_pro_input_report.set_button(button_index, true);
  switch_pro_input_report.set_dpad(false, true, false, true); // down-right
  switch_pro_input_report.set_left_joystick(sin(angle), cos(angle));
  switch_pro_input_report.set_right_joystick(cos(angle), sin(angle));
  switch_pro_input_report.set_left_trigger((float)std::abs(cos(angle)));
  switch_pro_input_report.set_right_trigger((float)std::abs(sin(angle)));

  dualsense_simple_input_report.set_button(button_index, true);
  dualsense_simple_input_report.set_hat(hat);
  dualsense_simple_input_report.set_left_joystick(sin(angle), cos(angle));
  dualsense_simple_input_report.set_right_joystick(cos(angle), sin(angle));
  dualsense_simple_input_report.set_left_trigger(std::abs(cos(angle)));
  dualsense_simple_input_report.set_right_trigger(std::abs(sin(angle)));

  dualsense_complex_input_report.set_button(button_index, true);
  dualsense_complex_input_report.set_hat(hat);
  dualsense_complex_input_report.set_left_joystick(sin(angle), cos(angle));
  dualsense_complex_input_report.set_right_joystick(cos(angle), sin(angle));
  dualsense_complex_input_report.set_left_trigger(std::abs(cos(angle)));
  dualsense_complex_input_report.set_right_trigger(std::abs(sin(angle)));

  ps4_input_report.set_hat(hat);
  ps4_input_report.set_button_cross(button_index == 1);
  ps4_input_report.set_button_circle(button_index == 2);
  ps4_input_report.set_button_square(button_index == 3);
  ps4_input_report.set_button_triangle(button_index == 4);
  ps4_input_report.set_left_joystick(128 + 127 * sin(angle), 128 + 127 * cos(angle));
  ps4_input_report.set_right_joystick(128 + 127 * cos(angle), 128 + 127 * sin(angle));
  ps4_input_report.set_l2_trigger(std::abs(cos(angle)) * 255);
  ps4_input_report.set_r2_trigger(std::abs(sin(angle)) * 255);
  ps4_input_report.set_battery_level(8);

  button_index = (button_index % num_buttons) + 1;

  // send an input report
  auto report = gamepad_input_report.get_report();
  logger.info("{}", gamepad_input_report);
  logger.info("Gamepad Input report:");
  logger.info("  Size: {}", report.size());
  logger.info("  Data: {::#02X}", report);

  report = switch_pro_input_report.get_report();
  logger.info("{}", switch_pro_input_report);
  logger.info("Switch Pro Input report:");
  logger.info("  Size: {}", report.size());
  logger.info("  Data: {::#02X}", report);

  report = dualsense_simple_input_report.get_report();
  logger.info("{}", dualsense_simple_input_report);
  logger.info("Playstation Dualsense BLE Simple Input report:");
  logger.info("  Size: {}", report.size());
  logger.info("  Data: {::#02X}", report);

  report = dualsense_complex_input_report.get_report();
  logger.info("{}", dualsense_complex_input_report);
  logger.info("Playstation Dualsense BLE Complex Input report:");
  logger.info("  Size: {}", report.size());
  logger.info("  Data: {::#02X}", report);

  // update the battery input report
  battery_input_report.reset();
  battery_input_report.set_rechargeable(true);
  battery_input_report.set_charging(false);
  battery_input_report.set_rechargeable(true);
  // note: it can only show 5, 40, 70, 100 so this will be rounded to 40
  battery_input_report.set_battery_level(50);

  // send a battery report
  report = battery_input_report.get_report();
  logger.info("Battery report:");
  logger.info("  Size: {}", report.size());
  logger.info("  Data: {::#02X}", report);

  //! [hid rp example]
}

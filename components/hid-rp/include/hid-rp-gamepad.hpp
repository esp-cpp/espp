#pragma once

#include <algorithm>

#include "hid-rp.hpp"

namespace espp {

/// HID Gamepad Input Report
/// This class implements a HID Gamepad with a configurable number of buttons, a
/// hat switch, 4 joystick axes and two trigger axes. It supports setting the
/// buttons, hat switch, joysticks, and triggers, as well as serializing the
/// input report and getting the report descriptor.
///
/// \section hid_rp_ex1 HID-RP Example
/// \snippet hid_rp_example.cpp hid rp example
template <size_t BUTTON_COUNT = 15, typename JOYSTICK_TYPE = std::uint16_t,
          typename TRIGGER_TYPE = std::uint16_t, JOYSTICK_TYPE JOYSTICK_MIN = 0,
          JOYSTICK_TYPE JOYSTICK_MAX = 65535, TRIGGER_TYPE TRIGGER_MIN = 0,
          TRIGGER_TYPE TRIGGER_MAX = 1023, uint8_t REPORT_ID = 1>
class GamepadInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
protected:
  static constexpr size_t button_count = BUTTON_COUNT;
  static constexpr JOYSTICK_TYPE joystick_min = JOYSTICK_MIN;
  static constexpr JOYSTICK_TYPE joystick_max = JOYSTICK_MAX;
  static constexpr JOYSTICK_TYPE joystick_center = (joystick_min + joystick_max) / 2;
  static constexpr TRIGGER_TYPE trigger_min = TRIGGER_MIN;
  static constexpr TRIGGER_TYPE trigger_max = TRIGGER_MAX;
  static constexpr TRIGGER_TYPE trigger_center = TRIGGER_MIN;
  static constexpr size_t joystick_value_range = joystick_max - joystick_min;
  static constexpr size_t joystick_range = joystick_value_range / 2;
  static constexpr size_t trigger_range = trigger_max - trigger_min;
  static constexpr size_t num_joystick_bits = num_bits(joystick_value_range);
  static constexpr size_t num_trigger_bits = num_bits(trigger_range);

  static constexpr size_t num_joystick_bytes = (num_joystick_bits + 7) / 8;
  static constexpr size_t num_trigger_bytes = (num_trigger_bits + 7) / 8;
  static constexpr size_t num_hat_bytes = 1;
  static constexpr size_t num_button_bytes = (BUTTON_COUNT + 7) / 8;
  static constexpr size_t num_consumer_bytes = 1;
  static constexpr size_t num_data_bytes = num_joystick_bytes * 4 + num_trigger_bytes * 2 +
                                           num_hat_bytes + num_button_bytes + num_consumer_bytes;

  std::array<JOYSTICK_TYPE, 4> joystick_axes{0};
  std::array<TRIGGER_TYPE, 2> trigger_axes{0};
  std::uint8_t hat_switch{0};
  hid::report_bitset<hid::page::button, hid::page::button(1), hid::page::button(BUTTON_COUNT)>
      buttons;
  struct {
    std::uint8_t consumer_record : 1;
    std::uint8_t : 7;
  };

public:
  /// Possible Hat switch directions
  enum class Hat {
    CENTERED = 0x0f, ///< Centered, no direction pressed.
    UP = 1,
    UP_RIGHT,
    RIGHT,
    DOWN_RIGHT,
    DOWN,
    DOWN_LEFT,
    LEFT,
    UP_LEFT
  };

  /// Construct a new Gamepad Input Report object
  constexpr GamepadInputReport() = default;

  /// Reset the gamepad inputs
  constexpr void reset() {
    std::fill(joystick_axes.begin(), joystick_axes.end(), joystick_center);
    std::fill(trigger_axes.begin(), trigger_axes.end(), trigger_center);
    set_hat(Hat::CENTERED);
    buttons.reset();
    consumer_record = 0;
  }

  /// Set the left joystick X and Y axis values
  /// @param lx left joystick x axis value, in the range [-1, 1]
  /// @param ly left joystick y axis value, in the range [-1, 1]
  constexpr void set_left_joystick(float lx, float ly) {
    set_joystick_axis(0, lx);
    set_joystick_axis(1, ly);
  }
  /// Set the right joystick X and Y axis values
  /// @param rx right joystick x axis value, in the range [-1, 1]
  /// @param ry right joystick y axis value, in the range [-1, 1]
  constexpr void set_right_joystick(float rx, float ry) {
    set_joystick_axis(2, rx);
    set_joystick_axis(3, ry);
  }
  /// Set the brake trigger value
  /// @param value brake trigger value, in the range [0, 1]
  constexpr void set_brake(float value) { set_trigger_axis(0, value); }
  /// Set the accelerator trigger value
  /// @param value accelerator trigger value, in the range [0, 1]
  constexpr void set_accelerator(float value) { set_trigger_axis(1, value); }
  /// Set the hat switch (d-pad) value
  /// @param hat Hat enum / direction to set
  constexpr void set_hat(Hat hat) { set_hat_switch(uint8_t(hat)); }
  /// Set the button value
  /// \param button_index The button for which you want to set the value.
  ///        Should be between 1 and BUTTON_COUNT, inclusive.
  /// \param value The true/false value you want to se the button to.
  constexpr void set_button(int button_index, bool value) {
    if (button_index < 1 || button_index > BUTTON_COUNT) {
      return;
    }
    buttons.set(hid::page::button(button_index), value);
  }

  /// Set the joystick axis value
  /// \param index The index of the joystick axis to set. Should be between 0 and 3, inclusive.
  /// \param value The value to set the joystick axis to.
  /// \note The value should be in the range [joystick_min, joystick_max].
  constexpr void set_joystick_axis(size_t index, JOYSTICK_TYPE value) {
    if (index < 4) {
      joystick_axes[index] = std::clamp(value, joystick_min, joystick_max);
    }
  }
  /// Set the joystick axis value
  /// \param index The index of the joystick axis to set. Should be between 0 and 3, inclusive.
  /// \param value The value to set the joystick axis to.
  /// \note The value should be in the range [-1, 1].
  constexpr void set_joystick_axis(size_t index, float value) {
    if (index < 4) {
      joystick_axes[index] =
          std::clamp(static_cast<JOYSTICK_TYPE>(value * joystick_range + joystick_center),
                     joystick_min, joystick_max);
    }
  }
  /// Set the trigger axis value
  /// \param index The index of the trigger axis to set. Should be between 0 and 1, inclusive.
  /// \param value The value to set the trigger axis to.
  /// \note The value should be in the range [trigger_min, trigger_max].
  constexpr void set_trigger_axis(size_t index, TRIGGER_TYPE value) {
    if (index < 2) {
      trigger_axes[index] = std::clamp(value, trigger_min, trigger_max);
    }
  }
  /// Set the trigger axis value
  /// \param index The index of the trigger axis to set. Should be between 0 and 1, inclusive.
  /// \param value The value to set the trigger axis to.
  /// \note The value should be in the range [0, 1].
  constexpr void set_trigger_axis(size_t index, float value) {
    if (index < 2) {
      trigger_axes[index] =
          std::clamp(static_cast<TRIGGER_TYPE>(value * trigger_range + trigger_center), trigger_min,
                     trigger_max);
    }
  }
  /// Set the hat switch value
  /// \param hat The hat switch value to set.
  constexpr void set_hat_switch(Hat hat) { hat_switch = static_cast<std::uint8_t>(hat); }
  /// Set the hat switch value
  /// \param value The hat switch value to set.
  /// \note The value should match the values within the Hat enum.
  constexpr void set_hat_switch(std::uint8_t value) { hat_switch = (value & 0xf); }

  /// Set the consumer record button value
  /// \param value The true/false value you want to se the consumer record button to.
  constexpr void set_consumer_record(bool value) { consumer_record = value; }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() {
    // the first two bytes are the id and the selector, which we don't want
    size_t offset = 2;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      \code{.cpp}
  ///      using namespace hid::page;
  ///      using namespace hid::rdf;
  ///      auto gamepad_descriptor = gamepad_input_report.get_descriptor();
  ///      auto rdf_descriptor = descriptor(
  ///          usage_page<generic_desktop>(),
  ///          usage(generic_desktop::GAMEPAD),
  ///          collection::application(
  ///              gamepad_descriptor
  ///          )
  ///      );
  ///      auto descriptor = std::vector<uint8_t>(rdf_descriptor.begin(), rdf_descriptor.end());
  ///      \endcode
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return descriptor(
                        conditional_report_id<REPORT_ID>(),

                        // left joystick
                        usage(generic_desktop::POINTER),
                        collection::physical(
                                             usage(generic_desktop::X),
                                             usage(generic_desktop::Y),
                                             logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                             report_count(2),
                                             report_size(num_joystick_bits),
                                             input::absolute_variable()
                                             ),

                        // right joystick
                        usage(generic_desktop::POINTER),
                        collection::physical(
                                             usage(generic_desktop::Z),
                                             usage(generic_desktop::RZ),
                                             logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                             report_count(2),
                                             report_size(num_joystick_bits),
                                             input::absolute_variable()
                                             ),

                        // left trigger
                        usage_page<simulation>(),
                        usage(simulation::BRAKE),
                        logical_limits<1, 2>(TRIGGER_MIN, TRIGGER_MAX),
                        report_count(1),
                        report_size(num_trigger_bits),
                        input::absolute_variable(),
                        logical_limits<1, 1>(0, 0),
                        input::byte_padding<num_trigger_bits>(),

                        // right trigger
                        usage_page<simulation>(),
                        usage(simulation::ACCELERATOR),
                        logical_limits<1, 2>(TRIGGER_MIN, TRIGGER_MAX),
                        report_count(1),
                        report_size(num_trigger_bits),
                        input::absolute_variable(),
                        logical_limits<1, 1>(0, 0),
                        input::byte_padding<num_trigger_bits>(),

                        // hat switch
                        usage_page<generic_desktop>(),
                        usage(generic_desktop::HAT_SWITCH),
                        logical_limits<1, 1>(1, 8),
                        physical_limits<1, 2>(0, 315),
                        unit::unit<2>(unit::code::DEGREE),
                        report_size(4),
                        report_count(1),
                        input::absolute_variable(static_cast<main::field_flags>(main::field_flags::NULL_STATE)),
                        logical_limits<1, 1>(0, 0),
                        physical_limits<1, 1>(0, 0),
                        unit::none(),
                        input::byte_padding<4>(),

                        // buttons
                        usage_page<button>(),
                        usage_limits(button(1), button(BUTTON_COUNT)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(BUTTON_COUNT),
                        input::absolute_variable(),
                        logical_limits<1, 1>(0, 0),
                        input::byte_padding<BUTTON_COUNT>(),

                        // consumer record button
                        usage_page<consumer>(),
                        usage(consumer::RECORD),
                        logical_limits<1, 1>(0, 1),
                        report_count(1),
                        report_size(1),
                        input::absolute_variable(),
                        logical_limits<1, 1>(0, 0),
                        input::byte_padding<1>()
                        );
    // clang-format on
  }
};

/// HID Gamepad Output Report
/// This class implements a HID Gamepad that supports setting the rumble effect
/// on the left and right motors, as well as serializing the output report and
/// getting the report descriptor.
template <uint8_t REPORT_ID = 3>
class XboxRumbleOutputReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
protected:
  struct {
    std::uint8_t enabled : 4; // which motors are enabled
    std::uint8_t : 4;
  };

  std::array<std::uint8_t, 4> magnitude; // [0, 100] magnitude of the effect on each motor
  std::uint8_t duration;                 // [0, 255] duration of the effect
  std::uint8_t start_delay;              // [0, 255] delay before the effect starts
  std::uint8_t loop_count;               // [0, 255] number of times to loop the effect

public:
  static constexpr std::uint8_t MAX_MAGNITUDE{100};
  static constexpr std::uint8_t MAX_DURATION{255};
  static constexpr std::uint8_t MAX_DELAY{255};
  static constexpr std::uint8_t MAX_LOOP_COUNT{255};
  static constexpr std::size_t num_data_bytes = sizeof(XboxRumbleOutputReport);

  /// Construct a new Xbox Rumble Output Report object
  constexpr XboxRumbleOutputReport() = default;

  /// Reset the rumble effect
  constexpr void reset() {
    enabled = 0;
    std::fill(magnitude.begin(), magnitude.end(), 0);
    duration = 0;
    start_delay = 0;
    loop_count = 0;
  }

  /// Get the enabled mask for the rumble motors
  /// \return The enabled mask for the rumble motors
  constexpr auto get_enabled() { return enabled; }

  /// Set the enabled mask for the rumble motors
  /// \param enabled The enabled mask for the rumble motors
  constexpr void set_enabled(std::uint8_t enabled) { enabled = enabled; }

  /// Get the magnitude of the rumble effect for the specified motor
  /// \param motor The motor for which you want to get the magnitude.
  /// \return The magnitude of the rumble effect for the specified motor.
  constexpr auto get_magnitude(std::size_t motor) { return magnitude[motor]; }

  /// Set the magnitude of the rumble effect for the specified motor
  /// \param motor The motor for which you want to set the magnitude.
  /// \param value The magnitude of the rumble effect for the specified motor.
  /// \note The value should be in the range [0, 100].
  constexpr void set_magnitude(std::size_t motor, std::uint8_t value) { magnitude[motor] = value; }

  /// Set the magnitude of the rumble effect for the specified motor
  /// \param motor The motor for which you want to set the magnitude.
  /// \param value The magnitude of the rumble effect for the specified motor.
  /// \note The value should be in the range [0, 1].
  constexpr void set_magnitude(std::size_t motor, float value) {
    magnitude[motor] = std::clamp<std::uint8_t>(value * MAX_MAGNITUDE, 0, MAX_MAGNITUDE);
  }

  /// Get the duration of the rumble effect
  /// \return The duration of the rumble effect
  constexpr auto get_duration() { return duration; }

  /// Set the duration of the rumble effect
  /// \param value The duration of the rumble effect.
  /// \note The value should be in the range [0, 255].
  constexpr void set_duration(std::uint8_t value) { duration = value; }

  /// Get the start delay of the rumble effect
  /// \return The start delay of the rumble effect
  constexpr auto get_start_delay() { return start_delay; }

  /// Set the start delay of the rumble effect
  /// \param value The start delay of the rumble effect.
  /// \note The value should be in the range [0, 255].
  constexpr void set_start_delay(std::uint8_t value) { start_delay = value; }

  /// Get the loop count of the rumble effect
  /// \return The loop count of the rumble effect
  constexpr auto get_loop_count() { return loop_count; }

  /// Set the loop count of the rumble effect
  /// \param value The loop count of the rumble effect.
  /// \note The value should be in the range [0, 255].
  constexpr void set_loop_count(std::uint8_t value) { loop_count = value; }

  /// Get the output report as a vector of bytes
  /// \return The output report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id
    std::copy(data.begin(), data.end(), this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      \code{.cpp}
  ///      using namespace hid::page;
  ///      using namespace hid::rdf;
  ///      auto rumble_descriptor = rumble_output_report.get_descriptor();
  ///      auto rdf_descriptor = descriptor(
  ///          usage_page<generic_desktop>(),
  ///          usage(generic_desktop::GAMEPAD),
  ///          collection::application(
  ///              rumble_descriptor
  ///          )
  ///      );
  ///      auto descriptor = std::vector<uint8_t>(rdf_descriptor.begin(), rdf_descriptor.end());
  ///      \endcode
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
    return descriptor(
            usage_page<physical_input_device>(),
            usage(physical_input_device::SET_EFFECT_REPORT),
            conditional_report_id<REPORT_ID>(),
            collection::logical(
                                usage(physical_input_device::DC_ENABLE_ACTUATORS),
                                logical_limits<1, 1>(0, 1),
                                report_size(4),
                                report_count(1),
                                output::absolute_variable(),
                                logical_limits<1, 1>(0, 0),
                                report_size(4),
                                report_count(1),
                                output::absolute_constant(),

                                usage(physical_input_device::MAGNITUDE),
                                logical_limits<1, 1>(0, MAX_MAGNITUDE),
                                report_size(8),
                                report_count(4),
                                output::absolute_variable(),

                                usage(physical_input_device::DURATION),
                                unit::second(-2), // System: SI Linear, Unit: Second, exponent: -2
                                logical_limits<1, 2>(0, MAX_DURATION),
                                report_size(8),
                                report_count(1),
                                output::absolute_variable(),

                                usage(physical_input_device::START_DELAY),
                                logical_limits<1, 2>(0, MAX_DELAY),
                                report_size(8),
                                report_count(1),
                                output::absolute_variable(),

                                unit::none(),
                                usage(physical_input_device::LOOP_COUNT),
                                logical_limits<1, 2>(0, MAX_LOOP_COUNT),
                                report_size(8),
                                report_count(1),
                                output::absolute_variable()
                                )
    );
    // clang-format on
  } // get_descriptor
};  // class XboxRumbleOutputReport

/// HID Xbox Battery Input Report
/// This class implements a copy of the Xbox Battery input report. It is a
/// single byte input which contains information about the type of the battery,
/// its battery level, as well as a few other things.
///
/// \section hid_rp_ex1 HID-RP Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 4>
class XboxBatteryInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
protected:
  static constexpr uint8_t battery_min{0};
  static constexpr uint8_t battery_max{255};
  static constexpr uint8_t num_data_bytes{1};

  uint8_t battery_status{0}; ///< The battery status byte

public:
  /// The possible errors for the battery
  enum class Error {
    NONE,             ///< No error
    BATTERY_LOW,      ///< The battery is low
    BATTERY_CRITICAL, ///< The battery is critically low
  };

  /// Reset the battery status
  constexpr void reset() { battery_status = 0; }

  /// Set whether the battery is rechargeable
  /// \param rechargeable True if the battery is rechargeable, false otherwise.
  constexpr void set_rechargeable(bool rechargeable) {
    if (rechargeable) {
      battery_status |= 0x04;
    } else {
      battery_status &= ~0x04;
    }
  }

  /// Set the battery level
  /// \param level The battery level as a percentage, from 0 to 100.
  constexpr void set_battery_level(int level) {
    if (level > 70) {
      battery_status |= 0x03;
    } else if (level > 40) {
      battery_status |= 0x02;
    } else if (level > 5) {
      battery_status |= 0x01;
    }
  }

  /// Set whether the battery is connected to a cable
  /// \param connected True if the battery is connected to a cable, false otherwise.
  constexpr void set_cable_connected(bool connected) {
    if (connected) {
      battery_status |= 0x10;
    } else {
      battery_status &= ~0x10;
    }
  }

  /// Set whether the battery is charging
  /// \param charging True if the battery is charging, false otherwise.
  constexpr void set_charging(bool charging) {
    if (charging) {
      battery_status |= 0x20;
    } else {
      battery_status &= ~0x20;
    }
  }

  /// Set the error state of the battery
  /// \param error The error state of the battery.
  constexpr void set_error(Error error) {
    switch (error) {
    case Error::BATTERY_LOW:
      battery_status |= 0x80;
      break;
    case Error::BATTERY_CRITICAL:
      battery_status |= 0xC0;
      break;
    default:
      break;
    }
  }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      \code{.cpp}
  ///      using namespace hid::page;
  ///      using namespace hid::rdf;
  ///      auto gamepad_descriptor = gamepad_input_report.get_descriptor();
  ///      auto rdf_descriptor = descriptor(
  ///          usage_page<generic_desktop>(),
  ///          usage(generic_desktop::GAMEPAD),
  ///          collection::application(
  ///              gamepad_descriptor
  ///          )
  ///      );
  ///      auto descriptor = std::vector<uint8_t>(rdf_descriptor.begin(), rdf_descriptor.end());
  ///      \endcode
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return descriptor(
                        conditional_report_id<REPORT_ID>(),

                        // Battery status
                        usage(generic_device::BATTERY_STRENGTH),
                        conditional_report_id<4>(),
                        logical_limits<1,2>(battery_min, battery_max),
                        report_size(8),
                        report_count(1),
                        usage(generic_device::BATTERY_STRENGTH),
                        input::absolute_variable()
                        );
    // clang-format on
  }
};

/// HID Gamepad LED Output Report
/// This class implements a HID Gamepad with a configurable number of LEDs.
/// It supports setting the LEDs, as well as serializing the output report and
/// getting the report descriptor.
template <size_t LED_COUNT = 4, uint8_t REPORT_ID = 2>
class GamepadLedOutputReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
protected:
  hid::report_bitset<hid::page::leds, hid::page::leds::PLAYER_1, hid::page::leds::PLAYER_4> leds;

public:
  static constexpr size_t led_count = LED_COUNT;
  static constexpr size_t num_leds = led_count;
  static constexpr size_t num_led_bits = num_bits(led_count);
  static constexpr size_t num_led_bytes = (led_count + 7) / 8;

  /// Set the LED value
  /// \param led_index The LED for which you want to set the value.
  ///        Should be between 1 and LED_COUNT, inclusive.
  /// \param value The true/false value you want to se the LED to.
  constexpr void set_led(int led_index, bool value) {
    if (led_index < 1 || led_index > LED_COUNT) {
      return;
    }
    leds.set(hid::page::leds(led_index), value);
  }

  /// Get the minimum led usage
  /// \return The minimum led usage
  constexpr auto get_min_led() { return leds.min(); }

  /// Get the maximum led usage
  /// \return The maximum led usage
  constexpr auto get_max_led() { return leds.max(); }

  /// Get the LED value
  /// \param led The LED for which you want to get the value.
  /// \return The true/false value of the LED.
  constexpr bool get_led(hid::page::leds led) { return leds.test(led); }

  /// Get the LED value
  /// \param led_index The LED for which you want to get the value.
  ///        Should be between 1 and LED_COUNT, inclusive.
  /// \return The true/false value of the LED.
  constexpr bool get_led(int led_index) {
    if (led_index < 1 || led_index > LED_COUNT) {
      return false;
    }
    return leds.test(hid::page::leds(led_index));
  }

  /// Get the output report as a vector of bytes
  /// \return The output report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() {
    // the first byte is the id, which we don't want...
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_led_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array - skip the first byte, which is the
    // report id
    std::copy(data.begin(), data.end(), this->data() + 1);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  ///      \code{.cpp}
  ///      using namespace hid::page;
  ///      using namespace hid::rdf;
  ///      auto led_descriptor = gamepad_led_report.get_descriptor();
  ///      auto rdf_descriptor = descriptor(
  ///          usage_page<generic_desktop>(),
  ///          usage(generic_desktop::GAMEPAD),
  ///          collection::application(
  ///              led_descriptor
  ///          )
  ///      );
  ///      auto descriptor = std::vector<uint8_t>(rdf_descriptor.begin(), rdf_descriptor.end());
  ///      \endcode
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return descriptor(
                        conditional_report_id<REPORT_ID>(),
                        usage_page<hid::page::leds>(),
                        usage_limits(leds::PLAYER_1, leds::PLAYER_4),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(LED_COUNT),
                        output::absolute_variable(),
                        output::byte_padding<LED_COUNT>()
                        );
    // clang-format on
  }
};

} // namespace espp

#pragma once

#include <algorithm>

#include "format.hpp"
#include "hid-rp.hpp"

#include "gamepad_hat.hpp"

namespace espp {

/// HID Gamepad Input Report
/// This class implements a HID Gamepad with a configurable number of buttons, a
/// hat switch, 4 joystick axes and two trigger axes. It supports setting the
/// buttons, hat switch, joysticks, and triggers, as well as serializing the
/// input report and getting the report descriptor.
///
/// \section hid_rp_gamepad_ex1 HID-RP Gamepad Example
/// \snippet hid_rp_example.cpp hid rp example
template <size_t BUTTON_COUNT = 15, typename JOYSTICK_TYPE = std::uint16_t,
          typename TRIGGER_TYPE = std::uint16_t, JOYSTICK_TYPE JOYSTICK_MIN = 0,
          JOYSTICK_TYPE JOYSTICK_MAX = 65535, TRIGGER_TYPE TRIGGER_MIN = 0,
          TRIGGER_TYPE TRIGGER_MAX = 1023, uint8_t REPORT_ID = 1>
class GamepadInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
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

protected:
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
  using Hat = espp::gamepad::Hat;

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

  /// Get the left joystick X and Y axis values
  /// @param[out] lx left joystick x axis value, in the range [-1, 1]
  /// @param[out] ly left joystick y axis value, in the range [-1, 1]
  constexpr void get_left_joystick(float &lx, float &ly) const {
    lx = (joystick_axes[0] - joystick_center) / static_cast<float>(joystick_range);
    ly = (joystick_axes[1] - joystick_center) / static_cast<float>(joystick_range);
  }

  /// Get the left joystick X and Y axis values
  /// @param[out] lx left joystick x axis value
  /// @param[out] ly left joystick y axis value
  constexpr void get_left_joystick(JOYSTICK_TYPE &lx, JOYSTICK_TYPE &ly) const {
    lx = joystick_axes[0];
    ly = joystick_axes[1];
  }

  /// Get the right joystick X and Y axis values
  /// @param[out] rx right joystick x axis value, in the range [-1, 1]
  /// @param[out] ry right joystick y axis value, in the range [-1, 1]
  constexpr void get_right_joystick(float &rx, float &ry) const {
    rx = (joystick_axes[2] - joystick_center) / static_cast<float>(joystick_range);
    ry = (joystick_axes[3] - joystick_center) / static_cast<float>(joystick_range);
  }

  /// Get the right joystick X and Y axis values
  /// @param[out] rx right joystick x axis value
  /// @param[out] ry right joystick y axis value
  constexpr void get_right_joystick(JOYSTICK_TYPE &rx, JOYSTICK_TYPE &ry) const {
    rx = joystick_axes[2];
    ry = joystick_axes[3];
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

  /// Get the brake trigger value
  /// @param value brake trigger value, in the range [0, 1]
  constexpr void get_brake(float &value) const {
    value = (trigger_axes[0] - trigger_center) / static_cast<float>(trigger_range);
  }

  /// Get the brake trigger value
  /// @return brake trigger value
  constexpr void get_brake(TRIGGER_TYPE &value) const { value = trigger_axes[0]; }

  /// Get the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void get_left_trigger(float &value) const { get_brake(value); }

  /// Set the brake trigger value
  /// @param value brake trigger value, in the range [0, 1]
  constexpr void set_brake(float value) { set_trigger_axis(0, value); }

  /// Set the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void set_left_trigger(float value) { set_trigger_axis(0, value); }

  /// Get the accelerator trigger value
  /// @param value accelerator trigger value, in the range [0, 1]
  constexpr void get_accelerator(float &value) const {
    value = (trigger_axes[1] - trigger_center) / static_cast<float>(trigger_range);
  }

  /// Get the accelerator trigger value
  /// @return accelerator trigger value
  constexpr void get_accelerator(TRIGGER_TYPE &value) const { value = trigger_axes[1]; }

  /// Get the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void get_right_trigger(float &value) const { get_accelerator(value); }

  /// Set the accelerator trigger value
  /// @param value accelerator trigger value, in the range [0, 1]
  constexpr void set_accelerator(float value) { set_trigger_axis(1, value); }

  /// Set the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void set_right_trigger(float value) { set_trigger_axis(1, value); }

  /// Set the hat switch (d-pad) value
  /// @param hat Hat enum / direction to set
  constexpr void set_hat(Hat hat) { set_hat_switch(uint8_t(hat)); }

  /// Set the hat switch (d-pad) value
  /// @param up up direction
  /// @param down down direction
  /// @param left left direction
  /// @param right right direction
  constexpr void set_hat(bool up, bool down, bool left, bool right) {
    if (up) {
      if (left) {
        set_hat(Hat::UP_LEFT);
      } else if (right) {
        set_hat(Hat::UP_RIGHT);
      } else {
        set_hat(Hat::UP);
      }
    } else if (down) {
      if (left) {
        set_hat(Hat::DOWN_LEFT);
      } else if (right) {
        set_hat(Hat::DOWN_RIGHT);
      } else {
        set_hat(Hat::DOWN);
      }
    } else if (left) {
      set_hat(Hat::LEFT);
    } else if (right) {
      set_hat(Hat::RIGHT);
    } else {
      set_hat(Hat::CENTERED);
    }
  }

  /// Get the hat switch (d-pad) value
  /// @return Hat enum / direction
  constexpr Hat get_hat() const { return static_cast<Hat>(hat_switch); }

  /// Get the hat switch (d-pad) value
  /// @param up up direction
  /// @param down down direction
  /// @param left left direction
  /// @param right right direction
  constexpr void get_hat(bool &up, bool &down, bool &left, bool &right) const {
    auto hat = get_hat();
    up = hat == Hat::UP || hat == Hat::UP_RIGHT || hat == Hat::UP_LEFT;
    down = hat == Hat::DOWN || hat == Hat::DOWN_RIGHT || hat == Hat::DOWN_LEFT;
    left = hat == Hat::LEFT || hat == Hat::UP_LEFT || hat == Hat::DOWN_LEFT;
    right = hat == Hat::RIGHT || hat == Hat::UP_RIGHT || hat == Hat::DOWN_RIGHT;
  }

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

  /// Get the button value
  /// \param button_index The button for which you want to get the value.
  /// \return The true/false value of the button.
  constexpr bool get_button(int button_index) const {
    if (button_index < 1 || button_index > BUTTON_COUNT) {
      return false;
    }
    return buttons.test(hid::page::button(button_index));
  }

  /// Set the hat switch value
  /// \param hat The hat switch value to set.
  constexpr void set_hat_switch(Hat hat) { // cppcheck-suppress passedByValue
    hat_switch = static_cast<std::uint8_t>(hat);
  }
  /// Set the hat switch value
  /// \param value The hat switch value to set.
  /// \note The value should match the values within the Hat enum.
  constexpr void set_hat_switch(std::uint8_t value) { hat_switch = (value & 0xf); }

  /// Get the consumer record button value
  /// \return The consumer record button value.
  constexpr bool get_consumer_record() const { return consumer_record; }

  /// Set the consumer record button value
  /// \param value The true/false value you want to se the consumer record button to.
  constexpr void set_consumer_record(bool value) { consumer_record = value; }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first two bytes are the id and the selector, which we don't want
    size_t offset = 2;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // the first two bytes are the id and the selector, which we don't want
    size_t offset = 2;
    // copy the data into our data array
    std::copy(data.begin(), data.end(), this->data() + offset);
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

  friend fmt::formatter<GamepadInputReport<BUTTON_COUNT, JOYSTICK_TYPE, TRIGGER_TYPE, JOYSTICK_MIN,
                                           JOYSTICK_MAX, TRIGGER_MIN, TRIGGER_MAX, REPORT_ID>>;
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

#include "hid-rp-gamepad-formatters.hpp"

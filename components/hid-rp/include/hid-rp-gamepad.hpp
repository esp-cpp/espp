#pragma once

#include "hid-rp.hpp"

namespace espp {

/// HID Gamepad Report
/// This class implements a HID Gamepad with a configurable number of buttons, a
/// hat switch, 4 joystick axes and two trigger axes. It supports setting the
/// buttons, hat switch, joysticks, and triggers, as well as serializing the
/// input report and getting the report descriptor.
///
/// \section hid_rp_ex1 HID-RP Example
/// \snippet hid_rp_example.cpp hid rp example
template <size_t BUTTON_COUNT, uint16_t JOYSTICK_MIN = 0, uint16_t JOYSTICK_MAX = 65535,
          uint16_t TRIGGER_MIN = 0, uint16_t TRIGGER_MAX = 1023, uint8_t REPORT_ID = 0>
class GamepadReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
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

protected:
  static constexpr size_t button_count = BUTTON_COUNT;
  static constexpr size_t num_button_padding = BUTTON_COUNT % 8 ? 8 - (BUTTON_COUNT % 8) : 0;
  static constexpr uint16_t joystick_min = JOYSTICK_MIN;
  static constexpr uint16_t joystick_max = JOYSTICK_MAX;
  static constexpr uint16_t joystick_center = (joystick_min + joystick_max) / 2;
  static constexpr uint16_t trigger_min = TRIGGER_MIN;
  static constexpr uint16_t trigger_max = TRIGGER_MAX;
  static constexpr uint16_t trigger_center = TRIGGER_MIN;
  static constexpr size_t joystick_value_range = joystick_max - joystick_min;
  static constexpr uint16_t joystick_range = joystick_value_range / 2;
  static constexpr size_t trigger_range = trigger_max - trigger_min;
  static constexpr size_t num_joystick_bits = num_bits(joystick_value_range);
  static constexpr size_t num_joystick_padding =
      num_joystick_bits % 8 ? 8 - (num_joystick_bits % 8) : 0;
  static constexpr size_t num_trigger_bits = num_bits(trigger_range);
  static constexpr size_t num_trigger_padding =
      num_trigger_bits % 8 ? 8 - (num_trigger_bits % 8) : 0;

  std::array<std::uint16_t, 4> joystick_axes{0};
  std::array<std::uint16_t, 2> trigger_axes{0};
  std::uint8_t hat_switch{0};
  hid::report_bitset<hid::page::button, hid::page::button(1), hid::page::button(BUTTON_COUNT)>
      buttons;

public:
  /// Reset the gamepad inputs
  constexpr void reset() {
    for (auto &axis : joystick_axes) {
      axis = joystick_center;
    }
    for (auto &axis : trigger_axes) {
      axis = trigger_center;
    }
    set_hat(Hat::CENTERED);
    buttons.reset();
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
  ///        Should be between 1 and BUTTON_COUNT
  /// \param value The true/false value you want to se the button to.
  constexpr void set_button(int button_index, bool value) {
    // buttons[button_index] = value;
    buttons.set(hid::page::button(button_index), value);
  }

  constexpr void set_joystick_axis(size_t index, std::uint16_t value) {
    if (index < 4) {
      joystick_axes[index] = std::clamp(value, joystick_min, joystick_max);
    }
  }
  constexpr void set_joystick_axis(size_t index, float value) {
    if (index < 4) {
      joystick_axes[index] =
          std::clamp(static_cast<std::uint16_t>(value * joystick_range + joystick_center),
                     joystick_min, joystick_max);
    }
  }
  constexpr void set_trigger_axis(size_t index, std::uint16_t value) {
    if (index < 2) {
      trigger_axes[index] = std::clamp(value, trigger_min, trigger_max);
    }
  }
  constexpr void set_trigger_axis(size_t index, float value) {
    if (index < 2) {
      trigger_axes[index] =
          std::clamp(static_cast<std::uint16_t>(value * trigger_range + trigger_center),
                     trigger_min, trigger_max);
    }
  }
  constexpr void set_hat_switch(std::uint8_t value) { hat_switch = (value & 0xf); }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  constexpr auto get_report() {
    // the first two bytes are the id and size, which we don't want...
    size_t offset = 2;
    auto report_data = this->data() + offset;
    auto report_size = sizeof(*this) - offset;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Get the report descriptor as a vector of bytes
  /// \return The report descriptor as a vector of bytes.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      auto desc = descriptor(
                        usage_page<generic_desktop>(),
                        usage(generic_desktop::GAMEPAD),
                        collection::application(
                                                report_id(REPORT_ID),
                                                usage(generic_desktop::POINTER),

                                                // left joystick
                                                collection::physical(
                                                                     usage(generic_desktop::X),
                                                                     logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                                                     report_count(1),
                                                                     report_size(num_joystick_bits),
                                                                     input::absolute_variable(),
                                                                     usage(generic_desktop::Y),
                                                                     logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                                                     report_count(1),
                                                                     report_size(num_joystick_bits),
                                                                     input::absolute_variable()
                                                                     ),

                                                // right joystick
                                                usage(generic_desktop::POINTER),
                                                collection::physical(
                                                                     usage(generic_desktop::Z),
                                                                     logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                                                     report_count(1),
                                                                     report_size(num_joystick_bits),
                                                                     input::absolute_variable(),
                                                                     usage(generic_desktop::RZ),
                                                                     logical_limits<1, 4>(JOYSTICK_MIN, JOYSTICK_MAX),
                                                                     report_count(1),
                                                                     report_size(num_joystick_bits),
                                                                     input::absolute_variable()
                                                                     ),

                                                // left trigger
                                                usage_page<simulation>(),
                                                usage(simulation::BRAKE),
                                                logical_limits<1, 2>(TRIGGER_MIN, TRIGGER_MAX),
                                                report_size(num_trigger_bits),
                                                report_count(1),
                                                input::absolute_variable(),
                                                input::padding(num_trigger_padding),

                                                // right trigger
                                                usage_page<simulation>(),
                                                usage(simulation::ACCELERATOR),
                                                logical_limits<1, 2>(TRIGGER_MIN, TRIGGER_MAX),
                                                report_size(num_trigger_bits),
                                                report_count(1),
                                                input::absolute_variable(),
                                                input::padding(num_trigger_padding),

                                                // hat switch
                                                usage_page<generic_desktop>(),
                                                usage(generic_desktop::HAT_SWITCH),
                                                logical_limits<1, 1>(1, 8),
                                                physical_limits<2, 2>(0, 315),
                                                unit::unit_item<2>(0x0014), // system: english rotation, length: centimeter
                                                report_size(4),
                                                report_count(1),
                                                input::absolute_variable(static_cast<main::field_flags>(main::field_flags::NULL_STATE)),
                                                input::padding(4),

                                                // buttons
                                                usage_page<button>(),
                                                usage_limits(button(1), button(BUTTON_COUNT)),
                                                logical_limits<1, 1>(0, 1),
                                                report_size(1),
                                                report_count(BUTTON_COUNT),
                                                input::absolute_variable(),
                                                input::padding(num_button_padding)
                                                )
                        );
    // clang-format on
    return std::vector<uint8_t>(desc.begin(), desc.end());
  }
};
} // namespace espp

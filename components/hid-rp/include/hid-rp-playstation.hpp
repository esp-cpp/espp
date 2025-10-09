#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <type_traits>
#include <utility>

#include "gamepad_imu.hpp"
#include "hid-rp-gamepad.hpp"

namespace hid::page {
enum class playstation_vendor_00 : std::uint16_t;
template <> struct info<playstation_vendor_00> {
  constexpr static page_id_t page_id = 0xFF00;
  constexpr static usage_id_t max_usage_id = 0xFFFF;
  constexpr static const char *name = "Playstation Vendor Page (0xFF00)";
};
} // namespace hid::page

namespace hid::page {
enum class playstation_vendor_80 : std::uint16_t;
template <> struct info<playstation_vendor_80> {
  constexpr static page_id_t page_id = 0xFF80;
  constexpr static usage_id_t max_usage_id = 0xFFFF;
  constexpr static const char *name = "Playstation Vendor Page (0xFF80)";
};
} // namespace hid::page

namespace espp {

#pragma pack(push, 1)

/// Playstation DualSense Touchpad Data Used in the vendor defined data for
/// input report ID 0x31 (49) when used over BLE.
struct PlaystationTouchpadData {
  union {
    std::uint8_t raw[9]; ///< Raw data array
    struct {
      std::uint32_t position[2];  ///< Touch positions (X, Y)
      std::uint8_t touch_counter; ///< Touch counter
    };
  };
};

/// Playstation DualSense Gamepad Buttons Used in input reports (0x01, 0x31)
/// when used over BLE.
union PlaystationDualsenseGamepadButtons {
  std::array<std::uint8_t, 3> raw; ///< Buttons as bytes
  struct {
    // byte 0
    std::uint8_t hat_switch : 4; ///< Hat switch (d-pad) value
    std::uint8_t square : 1;     ///< Square button
    std::uint8_t cross : 1;      ///< Cross button
    std::uint8_t circle : 1;     ///< Circle button
    std::uint8_t triangle : 1;   ///< Triangle button
    // byte 1
    std::uint8_t l1 : 1;      ///< L1 button
    std::uint8_t r1 : 1;      ///< R1 button
    std::uint8_t l2 : 1;      ///< L2 button
    std::uint8_t r2 : 1;      ///< R2 button
    std::uint8_t options : 1; ///< Share button
    std::uint8_t menu : 1;    ///< Options button
    std::uint8_t l3 : 1;      ///< L3 button
    std::uint8_t r3 : 1;      ///< R3 button
    // byte 2
    std::uint8_t home : 1;            ///< Playstation button
    std::uint8_t capture : 1;         ///< Touchpad button
    std::uint8_t microphone_mute : 1; ///< Microphone mute button
    std::uint16_t vendor_buttons : 5; ///< padding
  };                                  // struct
};                                    // union PlaystationDualsenseGamepadButtons

/// Playstation DualSense Vendor Defined Data. Used for IMU, touchpad, battery,
/// etc. in input report 0x31 (49) over Bluetooth.
union PlaystationDualsenseVendorDefinedData {
  using Accelerometer = espp::gamepad::Accelerometer; ///< Accelerometer type
  using Gyroscope = espp::gamepad::Gyroscope;         ///< Gyroscope type

  std::array<std::uint8_t, 65> raw; ///< Vendor defined data
  struct {
    std::uint32_t coarse_timestamp; ///< Coarse timestamp
    union {
      std::array<std::int16_t, 6> imu; ///< IMU data as array
      struct {
        Gyroscope gyro;                   ///< 3 axes of gyroscope data
        Accelerometer accel;              ///< 3 axes of accelerometer data
      };                                  // IMU struct
    };                                    // IMU union
    std::uint32_t fine_timestamp1;        ///< Fine timestamp 1
    std::uint8_t unknown0;                ///< Unknown data
    PlaystationTouchpadData touchpad;     ///< Touchpad data
    std::uint8_t r2_feedback;             ///< R2 trigger feedback
    std::uint8_t l2_feedback;             ///< L2 trigger feedback
    std::array<std::uint8_t, 5> unknown1; ///< Unknown data
    std::uint32_t fine_timestamp2;        ///< Fine timestamp 2
    union {
      std::array<std::uint8_t, 2> battery_raw;
      struct {
        uint8_t battery_percent : 4; ///< Battery percentage (0-100%) in 4 bits, 0=0%, 8=100%, other
                                     ///< values unused
        uint8_t : 1;                 ///< padding
        uint8_t battery_charging : 1; ///< Battery charging flag
        uint8_t : 2;                  ///< padding
      };
    };
    std::array<std::uint8_t, 18> unknown2; ///< Unknown data
    std::array<std::uint8_t, 4>
        crc; ///< CRC32 of all prior bytes, including report id and report-type seed. Seed is 0xA1
             ///< for input reports, 0xA2 for output reports, 0xA3 for feature reports.
  };         // struct
};           // union PlaystationDualsenseVendorDefinedData
#pragma pack(pop)

/// Button Struct concept for the Playstation controller
/// This concept defines the requirements for a struct that contains the
/// button values for the Playstation controller. Any struct which meets these
/// requirements can be used with the PlaystationDualsenseBLESimpleInputReport
/// and PlaystationDualsenseBLEComplexInputReport classes.
template <typename T>
concept PlaystationDualsenseButtonStruct = requires(T t) {
  { auto(t.cross) } -> std::integral;    ///< Cross button, same as xbox a
  { auto(t.circle) } -> std::integral;   ///< Circle button, same as xbox b
  { auto(t.square) } -> std::integral;   ///< Square button, same as xbox x
  { auto(t.triangle) } -> std::integral; ///< Triangle button, same as xbox y
  { auto(t.l1) } -> std::integral;       ///< L1 (left shoulder) button
  { auto(t.r1) } -> std::integral;       ///< R1 (right shoulder) button
  { auto(t.l2) } -> std::integral;       ///< L2 (left trigger, digital representation) button
  { auto(t.r2) } -> std::integral;       ///< R2 (right trigger, digital representation) button
  { auto(t.l3) } -> std::integral;       ///< L3 (left joystick) button
  { auto(t.r3) } -> std::integral;       ///< R3 (left joystick) button
  { auto(t.home) } -> std::integral;     ///< Playstation button
  { auto(t.capture) } -> std::integral;  ///< Create button
  { auto(t.menu) } -> std::integral;     ///< Touchpad button
  { auto(t.options) } -> std::integral;  ///< Options button
  { auto(t.microphone_mute) } -> std::integral; ///< Microphone mute button
};

/// Possible Hat switch directions for Playstation controller. Uses different
/// values than espp::gamepad::Hat
enum class PlaystationHat {
  CENTERED = 0x08, ///< Centered, no direction pressed.
  UP = 0,
  UP_RIGHT = 1,
  RIGHT = 2,
  DOWN_RIGHT = 3,
  DOWN = 4,
  DOWN_LEFT = 5,
  LEFT = 6,
  UP_LEFT = 7
};

/// Convert from espp::gamepad::Hat to PlaystationHat
/// \param hat The espp::gamepad::Hat value to convert
/// \return The corresponding PlaystationHat value
/// \note This is a constexpr function, so it can be used in compile-time
///       expressions, as well as at runtime.
[[maybe_unused]] static constexpr PlaystationHat from_gamepad_hat(espp::gamepad::Hat hat) {
  if (hat == espp::gamepad::Hat::CENTERED) {
    return PlaystationHat::CENTERED;
  }
  return static_cast<PlaystationHat>(static_cast<int>(hat) - 1);
}

/// HID Playstation DualSense Bluetooth Gamepad Input Report
///
/// This class implements a HID Playstation DualSense Bluetooth Gamepad Report.
/// It supports 14 buttons, a d-pad, 4 joystick axes, and 2 trigger axes.
///
/// This is the simple report which is used by default when connecting the
/// controller over bluetooth and matches report ID 0x01.
///
/// \note I have tested the playstation dualsense report descriptor and input
/// reports over BLE with iOS, MacOS, Windows, and Android and found that:
/// - iOS, MacOS, and Windows only parse the complex report (0x31) and ignore
///   the simple report (0x01). This means that the simple report is effectively
///   unused fields in the report descriptor.
/// - Android seems to require Audio support, which it cannot find over BLE, so
///   it does not work at all.
///
/// \section hid_rp_playstation_ex2 HID-RP Playstation Gamepad Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 1>
class PlaystationDualsenseBLESimpleInputReport
    : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  static constexpr size_t button_count = 14; ///< Number of buttons supported

  using Hat = PlaystationHat; ///< The type used for the hat switch

  using JOYSTICK_TYPE = std::uint8_t;                ///< The type used for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_min = 0;   ///< Minimum value for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_max = 255; ///< Maximum value for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_center =
      (joystick_min + joystick_max) / 2; ///< Center value for the joystick axes
  static constexpr size_t joystick_value_range =
      joystick_max - joystick_min; ///< Range of values for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_range =
      joystick_value_range / 2; ///< Half the range of values for the joystick axes
  static constexpr size_t num_joystick_bits =
      num_bits(joystick_value_range); ///< Number of bits needed to represent the joystick axes

  using TRIGGER_TYPE = std::uint8_t;               ///< The type used for the trigger axes
  static constexpr TRIGGER_TYPE trigger_min = 0;   ///< Minimum value for the trigger axes
  static constexpr TRIGGER_TYPE trigger_max = 255; ///< Maximum value for the trigger axes
  static constexpr TRIGGER_TYPE trigger_center = trigger_min; ///< Center value for the trigger axes
  static constexpr size_t trigger_range =
      trigger_max - trigger_min; ///< Range of values for the trigger axes
  static constexpr size_t num_trigger_bits =
      num_bits(trigger_range); ///< Number of bits needed to represent the trigger axes

  static constexpr size_t num_data_bytes =
      4 * 1 /* 4 joystick axes */ + sizeof(PlaystationDualsenseGamepadButtons) /* buttons */ +
      2 * 1 /* 2 trigger axes */;

  static_assert(num_data_bytes == 9,
                "PlaystationDualsenseBLESimpleInputReport: num_data_bytes must be 9");

  static constexpr std::size_t BTN_SQUARE_INDEX{1};   ///< The index of the SQUARE button
  static constexpr std::size_t BTN_CROSS_INDEX{2};    ///< The index of the CROSS button
  static constexpr std::size_t BTN_CIRCLE_INDEX{3};   ///< The index of the CIRCLE button
  static constexpr std::size_t BTN_TRIANGLE_INDEX{4}; ///< The index of the TRIANGLE button
  static constexpr std::size_t BTN_L1_INDEX{5};       ///< The index of the L1 button
  static constexpr std::size_t BTN_R1_INDEX{6};       ///< The index of the R1 button
  static constexpr std::size_t BTN_L2_INDEX{7};       ///< The index of the L2 button
  static constexpr std::size_t BTN_R2_INDEX{8};       ///< The index of the R2 button
  static constexpr std::size_t BTN_L3_INDEX{11};      ///< The index of the L3 button
  static constexpr std::size_t BTN_R3_INDEX{12};      ///< The index of the R3 button
  static constexpr std::size_t BTN_HOME_INDEX{13};    ///< The index of the Home button
  static constexpr std::size_t BTN_MENU_INDEX{10};    ///< The index of the Menu button
  static constexpr std::size_t BTN_OPTIONS_INDEX{9};  ///< The index of the Options button
  static constexpr std::size_t BTN_CAPTURE_INDEX{14}; ///< The index of the Capture button

protected:
  union {
    std::array<std::uint8_t, num_data_bytes> raw{0};
    struct {
      std::array<std::uint8_t, 4> joystick_axes;  ///< Joystick axes (LX, LY, RX, RY)
      PlaystationDualsenseGamepadButtons buttons; ///< Gamepad buttons
      std::array<std::uint8_t, 2> trigger_axes;   ///< Trigger axes (L2, R2)
    };
  };

public:
  /// Construct a new PlaystationPDualSenseBLE Gamepad Input Report object
  constexpr PlaystationDualsenseBLESimpleInputReport() = default;

  /// Reset the gamepad inputs
  constexpr void reset() {
    std::fill(joystick_axes.begin(), joystick_axes.end(), joystick_center);
    std::fill(trigger_axes.begin(), trigger_axes.end(), trigger_center);
    std::fill(buttons.raw.begin(), buttons.raw.end(), 0);
    set_hat(Hat::CENTERED);
  }

  /// Set the buttons
  /// @param buttons The struct containing the button values
  template <PlaystationDualsenseButtonStruct T> constexpr void set_buttons(const T &buttons) {
    this->set_button_cross(buttons.cross);
    this->set_button_circle(buttons.circle);
    this->set_button_square(buttons.square);
    this->set_button_triangle(buttons.triangle);
    this->set_button_l1(buttons.l1);
    this->set_button_r1(buttons.r1);
    this->set_button_l2(buttons.l2);
    this->set_button_r2(buttons.r2);
    this->set_button_l3(buttons.l3);
    this->set_button_r3(buttons.r3);
    this->set_button_home(buttons.home);
    this->set_button_capture(buttons.capture);
    this->set_button_menu(buttons.menu);
    this->set_button_options(buttons.options);
  }

  /// Get the button values
  /// @param t The struct to fill with the button values
  template <PlaystationDualsenseButtonStruct T> constexpr void get_buttons(T &t) const {
    t.cross = this->get_button_cross();
    t.circle = this->get_button_circle();
    t.square = this->get_button_square();
    t.triangle = this->get_button_triangle();
    t.l1 = this->get_button_l1();
    t.r1 = this->get_button_r1();
    t.l2 = this->get_button_l2();
    t.r2 = this->get_button_r2();
    t.l3 = this->get_button_l3();
    t.r3 = this->get_button_r3();
    t.home = this->get_button_home();
    t.capture = this->get_button_capture();
    t.menu = this->get_button_menu();
    t.options = this->get_button_options();
  }

  constexpr void set_button_cross(bool pressed) { this->set_button(BTN_CROSS_INDEX, pressed); }
  constexpr void set_button_circle(bool pressed) { this->set_button(BTN_CIRCLE_INDEX, pressed); }
  constexpr void set_button_square(bool pressed) { this->set_button(BTN_SQUARE_INDEX, pressed); }
  constexpr void set_button_triangle(bool pressed) {
    this->set_button(BTN_TRIANGLE_INDEX, pressed);
  }

  constexpr void set_button_l1(bool pressed) { this->set_button(BTN_L1_INDEX, pressed); }
  constexpr void set_button_r1(bool pressed) { this->set_button(BTN_R1_INDEX, pressed); }
  constexpr void set_button_l2(bool pressed) { this->set_button(BTN_L2_INDEX, pressed); }
  constexpr void set_button_r2(bool pressed) { this->set_button(BTN_R2_INDEX, pressed); }
  constexpr void set_button_l3(bool pressed) { this->set_button(BTN_L3_INDEX, pressed); }
  constexpr void set_button_r3(bool pressed) { this->set_button(BTN_R3_INDEX, pressed); }

  constexpr void set_button_home(bool pressed) { this->set_button(BTN_HOME_INDEX, pressed); }
  constexpr void set_button_capture(bool pressed) { this->set_button(BTN_CAPTURE_INDEX, pressed); }
  constexpr void set_button_menu(bool pressed) { this->set_button(BTN_MENU_INDEX, pressed); }
  constexpr void set_button_options(bool pressed) { this->set_button(BTN_OPTIONS_INDEX, pressed); }

  constexpr bool get_button_cross() const { return this->get_button(BTN_CROSS_INDEX); }
  constexpr bool get_button_circle() const { return this->get_button(BTN_CIRCLE_INDEX); }
  constexpr bool get_button_square() const { return this->get_button(BTN_SQUARE_INDEX); }
  constexpr bool get_button_triangle() const { return this->get_button(BTN_TRIANGLE_INDEX); }

  constexpr bool get_button_l1() const { return this->get_button(BTN_L1_INDEX); }
  constexpr bool get_button_r1() const { return this->get_button(BTN_R1_INDEX); }
  constexpr bool get_button_l2() const { return this->get_button(BTN_L2_INDEX); }
  constexpr bool get_button_r2() const { return this->get_button(BTN_R2_INDEX); }
  constexpr bool get_button_l3() const { return this->get_button(BTN_L3_INDEX); }
  constexpr bool get_button_r3() const { return this->get_button(BTN_R3_INDEX); }

  constexpr bool get_button_home() const { return this->get_button(BTN_HOME_INDEX); }
  constexpr bool get_button_capture() const { return this->get_button(BTN_CAPTURE_INDEX); }
  constexpr bool get_button_menu() const { return this->get_button(BTN_MENU_INDEX); }
  constexpr bool get_button_options() const { return this->get_button(BTN_OPTIONS_INDEX); }

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

  /// Get the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void get_left_trigger(float &value) const { get_trigger_axis(0, value); }

  /// Set the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void set_left_trigger(float value) { set_trigger_axis(0, value); }

  /// Get the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void get_right_trigger(float &value) const { get_trigger_axis(1, value); }

  /// Set the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void set_right_trigger(float value) { set_trigger_axis(1, value); }

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

  /// Get the joystick axis value
  /// \param index The index of the joystick axis to get. Should be between 0
  ///        and 3, inclusive.
  /// \param value The variable to store the joystick axis value in.
  constexpr void get_joystick_axis(size_t index, float &value) const {
    if (index < 4) {
      value = (joystick_axes[index] - joystick_center) / static_cast<float>(joystick_range);
    }
  }

  /// Get the joystick axis value
  /// \param index The index of the joystick axis to get. Should be between 0
  ///        and 3, inclusive.
  /// \param value The variable to store the joystick axis value in.
  constexpr void get_joystick_axis(size_t index, JOYSTICK_TYPE &value) const {
    if (index < 4) {
      value = joystick_axes[index];
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

  /// Get the trigger axis value
  /// \param index The index of the trigger axis to get. Should be between 0
  ///        and 1, inclusive.
  /// \param value The variable to store the trigger axis value in.
  constexpr void get_trigger_axis(size_t index, float &value) const {
    if (index < 2) {
      value = (trigger_axes[index] - trigger_center) / static_cast<float>(trigger_range);
    }
  }

  /// Get the trigger axis value
  /// \param index The index of the trigger axis to get. Should be between 0
  ///        and 1, inclusive.
  /// \param value The variable to store the trigger axis value in.
  constexpr void get_trigger_axis(size_t index, TRIGGER_TYPE &value) const {
    if (index < 2) {
      value = trigger_axes[index];
    }
  }

  /// Get the button value
  /// \param button_index The button for which you want to get the value.
  /// \return The true/false value of the button.
  constexpr bool get_button(int button_index) const {
    if (button_index < 1 || button_index > button_count) {
      return false;
    }
    // have to add 4 to button_index to account for the hat switch, but also
    // subtract 1 to make it 0-indexed
    button_index += 4 - 1; // yes this is 3.
    int byte_index = button_index / 8;
    return (buttons.raw[byte_index] & (1 << (button_index % 8))) != 0;
  }

  /// Set the button value
  /// \param button_index The button for which you want to set the value.
  ///        Should be between 1 and 14, inclusive.
  /// \param value The true/false value you want to se the button to.
  constexpr void set_button(int button_index, bool value) {
    if (button_index < 1 || button_index > button_count) {
      return;
    }
    // have to add 4 to button_index to account for the hat switch, but also
    // subtract 1 to make it 0-indexed
    button_index += 4 - 1; // yes this is 3.
    int byte_index = button_index / 8;
    buttons.raw[byte_index] &= ~(1 << (button_index % 8));
    if (value) {
      buttons.raw[byte_index] |= (1 << (button_index % 8));
    }
  }

  /// Set the hat switch value from espp::gamepad::Hat
  /// \param hat The espp::gamepad::Hat value to set.
  constexpr void set_hat_switch(espp::gamepad::Hat hat) { set_hat_switch(from_gamepad_hat(hat)); }

  /// Set the hat switch value
  /// \param hat The hat switch value to set.
  constexpr void set_hat_switch(Hat hat) { buttons.hat_switch = static_cast<std::uint8_t>(hat); }

  /// Set the hat switch value
  /// \param value The hat switch value to set.
  /// \note The value should match the values within the Hat enum.
  constexpr void set_hat_switch(std::uint8_t value) { buttons.hat_switch = (value & 0xf); }

  /// Set the hat switch (d-pad) value
  /// @param hat Hat enum / direction to set
  constexpr void set_hat(espp::gamepad::Hat hat) { set_hat_switch(from_gamepad_hat(hat)); }

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
  constexpr Hat get_hat() const { return static_cast<Hat>(buttons.hat_switch); }

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

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    auto report_data = raw.data();
    static constexpr auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array
    std::copy(data.begin(), data.end(), raw.begin());
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
    using namespace hid::rdf::main;

    // clang-format off
      return descriptor(
                        conditional_report_id<REPORT_ID>(),

                        // left / right joysticks (8-bit)
                        usage(generic_desktop::X),
                        usage(generic_desktop::Y),
                        usage(generic_desktop::Z),
                        usage(generic_desktop::RZ),
                        logical_limits<1, 2>(PlaystationDualsenseBLESimpleInputReport<>::joystick_min, PlaystationDualsenseBLESimpleInputReport<>::joystick_max),
                        report_size(PlaystationDualsenseBLESimpleInputReport<>::num_joystick_bits),
                        report_count(4),
                        input::absolute_variable(),

                        // hat switch
                        usage(generic_desktop::HAT_SWITCH),
                        logical_limits<1, 1>(0, 7),
                        physical_limits<1, 2>(0, 315),
                        unit::unit<1>(unit::code::DEGREE),
                        report_size(4),
                        report_count(1),
                        input::absolute_variable(static_cast<main::field_flags>(main::field_flags::NULL_STATE)),
                        unit::unit<1>(unit::code::NONE),

                        // No byte padding, buttons start half way through a
                        // byte

                        // buttons
                        usage_page<button>(),
                        usage_limits<1, 1>(button(1), button(PlaystationDualsenseBLESimpleInputReport<>::button_count)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(PlaystationDualsenseBLESimpleInputReport<>::button_count),
                        input::absolute_variable(),
                        report_size(6), // padding to next byte
                        report_count(1),
                        // need absolute constant array
                        data_field_item<1>(report_type_to_tag<hid::report::type::INPUT>(), data_field_flag::ARRAY | data_field_flag::ABSOLUTE | data_field_flag::CONSTANT),

                        // left / right triggers (8-bit)
                        usage_page<generic_desktop>(),
                        usage(generic_desktop::RX),
                        usage(generic_desktop::RY),
                        logical_limits<1, 2>(PlaystationDualsenseBLESimpleInputReport<>::trigger_min, PlaystationDualsenseBLESimpleInputReport<>::trigger_max),
                        report_size(PlaystationDualsenseBLESimpleInputReport<>::num_trigger_bits),
                        report_count(2),
                        input::absolute_variable()
                        );
    // clang-format on
  }

  friend fmt::formatter<PlaystationDualsenseBLESimpleInputReport<REPORT_ID>>;
};

/// HID Playstation DualSense Bluetooth Gamepad Input Report
///
/// This class implements a HID Playstation DualSense Bluetooth Gamepad Report.
/// It supports 15 buttons, a d-pad, 4 joystick axes, and 2 trigger axes. It
/// also includes a timestamp, packet counter, and vendor defined data. The
/// vendor defined data includes IMU data, battery level, touchpad data, and
/// CRC32.
///
/// This matches the information found here:
/// https://github.com/nondebug/dualsense
/// which provides a bluetooth VID/PID of 054C:0CE6
///
/// \note I have tested the playstation dualsense report descriptor and input
/// reports over BLE with iOS, MacOS, Windows, and Android and found that:
/// - iOS, MacOS, and Windows only parse the complex report (0x31) and ignore
///   the simple report (0x01). This means that the simple report is effectively
///   unused fields in the report descriptor.
/// - Android seems to require Audio support, which it cannot find over BLE, so
///   it does not work at all.
///
/// \section hid_rp_playstation_ex1 HID-RP Playstation Gamepad Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 49>
class PlaystationDualsenseBLEComplexInputReport
    : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  using Hat = PlaystationHat; ///< The type used for the hat switch

  static constexpr size_t button_count = 15; ///< Number of buttons supported

  using JOYSTICK_TYPE = std::uint8_t;                ///< The type used for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_min = 0;   ///< Minimum value for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_max = 255; ///< Maximum value for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_center =
      (joystick_min + joystick_max) / 2; ///< Center value for the joystick axes
  static constexpr size_t joystick_value_range =
      joystick_max - joystick_min; ///< Range of values for the joystick axes
  static constexpr JOYSTICK_TYPE joystick_range =
      joystick_value_range / 2; ///< Half the range of values for the joystick axes
  static constexpr size_t num_joystick_bits =
      num_bits(joystick_value_range); ///< Number of bits needed to represent the joystick axes

  using TRIGGER_TYPE = std::uint8_t;               ///< The type used for the trigger axes
  static constexpr TRIGGER_TYPE trigger_min = 0;   ///< Minimum value for the trigger axes
  static constexpr TRIGGER_TYPE trigger_max = 255; ///< Maximum value for the trigger axes
  static constexpr TRIGGER_TYPE trigger_center = trigger_min; ///< Center value for the trigger axes
  static constexpr size_t trigger_range =
      trigger_max - trigger_min; ///< Range of values for the trigger axes
  static constexpr size_t num_trigger_bits =
      num_bits(trigger_range); ///< Number of bits needed to represent the trigger axes

  static constexpr size_t num_data_bytes =
      1 +                                                            // timestamp
      4 * 1 /* 4 joystick axes */ + 2 * 1 /* 2 trigger axes */ + 1 + // counter
      sizeof(PlaystationDualsenseGamepadButtons) /* buttons */ + 1 + // unknown
      sizeof(PlaystationDualsenseVendorDefinedData);                 /* vendor defined bytes */
  static_assert(num_data_bytes == 77,
                "Unexpected size for PlaystationDualsenseBLEComplexInputReport");

  static constexpr std::size_t BTN_SQUARE_INDEX{1};   ///< The index of the SQUARE button
  static constexpr std::size_t BTN_CROSS_INDEX{2};    ///< The index of the CROSS button
  static constexpr std::size_t BTN_CIRCLE_INDEX{3};   ///< The index of the CIRCLE button
  static constexpr std::size_t BTN_TRIANGLE_INDEX{4}; ///< The index of the TRIANGLE button
  static constexpr std::size_t BTN_L1_INDEX{5};       ///< The index of the L1 button
  static constexpr std::size_t BTN_R1_INDEX{6};       ///< The index of the R1 button
  static constexpr std::size_t BTN_L2_INDEX{7};       ///< The index of the L2 button
  static constexpr std::size_t BTN_R2_INDEX{8};       ///< The index of the R2 button
  static constexpr std::size_t BTN_L3_INDEX{11};      ///< The index of the L3 button
  static constexpr std::size_t BTN_R3_INDEX{12};      ///< The index of the R3 button
  static constexpr std::size_t BTN_HOME_INDEX{13};    ///< The index of the Home button
  static constexpr std::size_t BTN_MENU_INDEX{10};    ///< The index of the Menu button
  static constexpr std::size_t BTN_OPTIONS_INDEX{9};  ///< The index of the Options button
  static constexpr std::size_t BTN_CAPTURE_INDEX{14}; ///< The index of the Capture button
  static constexpr std::size_t BTN_MIC_INDEX{15};     ///< The index of the Mic Mute button

protected:
  union {
    std::array<std::uint8_t, num_data_bytes> raw{0};
    struct {
      std::uint8_t timestamp;                               ///< Timestamp
      std::array<std::uint8_t, 4> joystick_axes;            ///< Joystick axes (LX, LY, RX, RY)
      std::array<std::uint8_t, 2> trigger_axes;             ///< Trigger axes (L2, R2)
      std::uint8_t counter;                                 ///< Packet counter
      PlaystationDualsenseGamepadButtons buttons;           ///< Gamepad buttons
      std::uint8_t unknown;                                 ///< Unknown byte
      PlaystationDualsenseVendorDefinedData vendor_defined; ///< Vendor defined bytes
    };
  };

public:
  /// Construct a new PlaystationPDualSenseBLE Gamepad Input Report object
  constexpr PlaystationDualsenseBLEComplexInputReport() = default;

  /// Reset the gamepad inputs
  constexpr void reset() {
    uint8_t prev_timestamp = timestamp;
    uint8_t prev_counter = counter;
    uint32_t prev_coarse_timestamp = vendor_defined.coarse_timestamp;
    uint32_t prev_fine_timestamp1 = vendor_defined.fine_timestamp1;
    uint32_t prev_fine_timestamp2 = vendor_defined.fine_timestamp2;
    uint8_t prev_battery = vendor_defined.battery_percent;
    bool prev_charging = vendor_defined.battery_charging != 0;
    std::fill(joystick_axes.begin(), joystick_axes.end(), joystick_center);
    std::fill(trigger_axes.begin(), trigger_axes.end(), trigger_center);
    std::fill(buttons.raw.begin(), buttons.raw.end(), 0);
    std::fill(vendor_defined.raw.begin(), vendor_defined.raw.end(), 0);
    timestamp = prev_timestamp; // cppcheck-suppress redundantAssignment
    counter = prev_counter;     // cppcheck-suppress redundantAssignment
    vendor_defined.coarse_timestamp = prev_coarse_timestamp;
    vendor_defined.fine_timestamp1 = prev_fine_timestamp1;
    vendor_defined.fine_timestamp2 = prev_fine_timestamp2;
    set_battery_level(prev_battery, prev_charging);
    set_hat(Hat::CENTERED);
    // set first byte of each touch position to 0x80
    vendor_defined.touchpad.raw[0] = 0x80;
    vendor_defined.touchpad.raw[4] = 0x80;
    // set unknown1 to be 00 00 00 00 33 (for me)
    static constexpr std::array<uint8_t, 5> unknown1_default = {0x00, 0x00, 0x00, 0x00, 0x33};
    std::copy(unknown1_default.begin(), unknown1_default.end(), vendor_defined.unknown1.begin());
    // set first two byets of unknown2 to 09 04 (for me)
    vendor_defined.unknown2[0] = 0x09;
    vendor_defined.unknown2[1] = 0x04;
  }

  constexpr void set_timestamp(uint8_t value) { timestamp = value; }
  constexpr void increment_timestamp(uint8_t inc = 16) { timestamp = (timestamp + inc); }
  constexpr uint8_t get_timestamp() const { return timestamp; }

  /// Set the counter
  /// @param value The value to set the counter to.
  constexpr void set_counter(uint8_t value) { counter = value; }

  /// Increment the counter
  constexpr void increment_counter(uint8_t inc = 1) { counter = (counter + inc); }

  /// Get the counter value
  /// @return The counter value
  constexpr uint8_t get_counter() const { return counter; }

  constexpr void set_coarse_timestamp(uint32_t value) { vendor_defined.coarse_timestamp = value; }
  constexpr void increment_coarse_timestamp(uint32_t inc = 1) {
    vendor_defined.coarse_timestamp = (vendor_defined.coarse_timestamp + inc);
  }
  constexpr uint32_t get_coarse_timestamp() const { return vendor_defined.coarse_timestamp; }

  constexpr void set_imu_data(const std::array<int16_t, 6> &data) { vendor_defined.imu = data; }
  constexpr const std::array<int16_t, 6> &get_imu_data() const { return vendor_defined.imu; }

  /// Set the battery level as a percentage (0-100) and if it is charging
  /// @param level The battery level as a percentage (0-100)
  /// @param charging True if the battery is charging, false otherwise
  constexpr void set_battery_level(uint8_t level, bool charging) {
    set_battery_level(level);
    set_battery_charging(charging);
  }

  /// Set the battery level as a percentage (0-100)
  /// @param level The battery level as a percentage (0-100)
  constexpr void set_battery_level(uint8_t level) {
    // convert level from 0-100 to 0-8
    if (level > 100) {
      level = 100;
    }
    level = (level * 8 + 99) / 100; // round up
    if (level > 8) {
      level = 8;
    }
    vendor_defined.battery_percent = level & 0x0F;
  }
  /// Get the battery level as a percentage (0-100)
  /// @return The battery level as a percentage (0-100)
  constexpr uint8_t get_battery_level() const {
    return (vendor_defined.battery_percent * 100 + 7) / 8; // round up
  }

  /// Check if the battery is charging
  /// @return True if the battery is charging, false otherwise
  constexpr bool is_battery_charging() const { return vendor_defined.battery_charging != 0; }

  /// Set if the battery is charging
  /// @param charging True if the battery is charging, false otherwise
  constexpr void set_battery_charging(bool charging) {
    vendor_defined.battery_charging = charging ? 1 : 0;
  }

  constexpr void set_fine_timestamps(uint32_t ts1, uint32_t ts2) {
    vendor_defined.fine_timestamp1 = ts1;
    vendor_defined.fine_timestamp2 = ts2;
  }
  constexpr void increment_fine_timestamps(uint32_t inc1 = 4000, uint32_t inc2 = 5000) {
    vendor_defined.fine_timestamp1 = (vendor_defined.fine_timestamp1 + inc1);
    vendor_defined.fine_timestamp2 = (vendor_defined.fine_timestamp2 + inc2);
  }
  constexpr uint32_t get_fine_timestamp1() const { return vendor_defined.fine_timestamp1; }
  constexpr uint32_t get_fine_timestamp2() const { return vendor_defined.fine_timestamp2; }

  /// Set the buttons
  /// @param buttons The struct containing the button values
  template <PlaystationDualsenseButtonStruct T> constexpr void set_buttons(const T &buttons) {
    this->set_button_cross(buttons.cross);
    this->set_button_circle(buttons.circle);
    this->set_button_square(buttons.square);
    this->set_button_triangle(buttons.triangle);
    this->set_button_l1(buttons.l1);
    this->set_button_r1(buttons.r1);
    this->set_button_l2(buttons.l2);
    this->set_button_r2(buttons.r2);
    this->set_button_l3(buttons.l3);
    this->set_button_r3(buttons.r3);
    this->set_button_home(buttons.home);
    this->set_button_capture(buttons.capture);
    this->set_button_menu(buttons.menu);
    this->set_button_options(buttons.options);
  }

  /// Get the button values
  /// @param t The struct to fill with the button values
  template <PlaystationDualsenseButtonStruct T> constexpr void get_buttons(T &t) const {
    t.cross = this->get_button_cross();
    t.circle = this->get_button_circle();
    t.square = this->get_button_square();
    t.triangle = this->get_button_triangle();
    t.l1 = this->get_button_l1();
    t.r1 = this->get_button_r1();
    t.l2 = this->get_button_l2();
    t.r2 = this->get_button_r2();
    t.l3 = this->get_button_l3();
    t.r3 = this->get_button_r3();
    t.home = this->get_button_home();
    t.capture = this->get_button_capture();
    t.menu = this->get_button_menu();
    t.options = this->get_button_options();
  }

  constexpr void set_button_cross(bool pressed) { this->set_button(BTN_CROSS_INDEX, pressed); }
  constexpr void set_button_circle(bool pressed) { this->set_button(BTN_CIRCLE_INDEX, pressed); }
  constexpr void set_button_square(bool pressed) { this->set_button(BTN_SQUARE_INDEX, pressed); }
  constexpr void set_button_triangle(bool pressed) {
    this->set_button(BTN_TRIANGLE_INDEX, pressed);
  }

  constexpr void set_button_l1(bool pressed) { this->set_button(BTN_L1_INDEX, pressed); }
  constexpr void set_button_r1(bool pressed) { this->set_button(BTN_R1_INDEX, pressed); }
  constexpr void set_button_l2(bool pressed) { this->set_button(BTN_L2_INDEX, pressed); }
  constexpr void set_button_r2(bool pressed) { this->set_button(BTN_R2_INDEX, pressed); }
  constexpr void set_button_l3(bool pressed) { this->set_button(BTN_L3_INDEX, pressed); }
  constexpr void set_button_r3(bool pressed) { this->set_button(BTN_R3_INDEX, pressed); }

  constexpr void set_button_home(bool pressed) { this->set_button(BTN_HOME_INDEX, pressed); }
  constexpr void set_button_capture(bool pressed) { this->set_button(BTN_CAPTURE_INDEX, pressed); }
  constexpr void set_button_menu(bool pressed) { this->set_button(BTN_MENU_INDEX, pressed); }
  constexpr void set_button_options(bool pressed) { this->set_button(BTN_OPTIONS_INDEX, pressed); }

  constexpr bool get_button_cross() const { return this->get_button(BTN_CROSS_INDEX); }
  constexpr bool get_button_circle() const { return this->get_button(BTN_CIRCLE_INDEX); }
  constexpr bool get_button_square() const { return this->get_button(BTN_SQUARE_INDEX); }
  constexpr bool get_button_triangle() const { return this->get_button(BTN_TRIANGLE_INDEX); }

  constexpr bool get_button_l1() const { return this->get_button(BTN_L1_INDEX); }
  constexpr bool get_button_r1() const { return this->get_button(BTN_R1_INDEX); }
  constexpr bool get_button_l2() const { return this->get_button(BTN_L2_INDEX); }
  constexpr bool get_button_r2() const { return this->get_button(BTN_R2_INDEX); }
  constexpr bool get_button_l3() const { return this->get_button(BTN_L3_INDEX); }
  constexpr bool get_button_r3() const { return this->get_button(BTN_R3_INDEX); }

  constexpr bool get_button_home() const { return this->get_button(BTN_HOME_INDEX); }
  constexpr bool get_button_capture() const { return this->get_button(BTN_CAPTURE_INDEX); }
  constexpr bool get_button_menu() const { return this->get_button(BTN_MENU_INDEX); }
  constexpr bool get_button_options() const { return this->get_button(BTN_OPTIONS_INDEX); }

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

  /// Get the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void get_left_trigger(float &value) const { get_trigger_axis(0, value); }

  /// Set the left trigger value
  /// @param value left trigger value, in the range [0, 1]
  constexpr void set_left_trigger(float value) { set_trigger_axis(0, value); }

  /// Get the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void get_right_trigger(float &value) const { get_trigger_axis(1, value); }

  /// Set the right trigger value
  /// @param value right trigger value, in the range [0, 1]
  constexpr void set_right_trigger(float value) { set_trigger_axis(1, value); }

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

  /// Get the joystick axis value
  /// \param index The index of the joystick axis to get. Should be between 0
  ///        and 3, inclusive.
  /// \param value The variable to store the joystick axis value in.
  constexpr void get_joystick_axis(size_t index, float &value) const {
    if (index < 4) {
      value = (joystick_axes[index] - joystick_center) / static_cast<float>(joystick_range);
    }
  }

  /// Get the joystick axis value
  /// \param index The index of the joystick axis to get. Should be between 0
  ///        and 3, inclusive.
  /// \param value The variable to store the joystick axis value in.
  constexpr void get_joystick_axis(size_t index, JOYSTICK_TYPE &value) const {
    if (index < 4) {
      value = joystick_axes[index];
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

  /// Get the trigger axis value
  /// \param index The index of the trigger axis to get. Should be between 0
  ///        and 1, inclusive.
  /// \param value The variable to store the trigger axis value in.
  constexpr void get_trigger_axis(size_t index, float &value) const {
    if (index < 2) {
      value = (trigger_axes[index] - trigger_center) / static_cast<float>(trigger_range);
    }
  }

  /// Get the trigger axis value
  /// \param index The index of the trigger axis to get. Should be between 0
  ///        and 1, inclusive.
  /// \param value The variable to store the trigger axis value in.
  constexpr void get_trigger_axis(size_t index, TRIGGER_TYPE &value) const {
    if (index < 2) {
      value = trigger_axes[index];
    }
  }

  /// Get the button value
  /// \param button_index The button for which you want to get the value.
  /// \return The true/false value of the button.
  constexpr bool get_button(int button_index) const {
    if (button_index < 1 || button_index > button_count) {
      return false;
    }
    // have to add 4 to button_index to account for the hat switch, but also
    // subtract 1 to make it 0-indexed
    button_index += 4 - 1; // yes this is 3.
    int byte_index = button_index / 8;
    return (buttons.raw[byte_index] & (1 << (button_index % 8))) != 0;
  }

  /// Set the button value
  /// \param button_index The button for which you want to set the value.
  ///        Should be between 1 and 14, inclusive.
  /// \param value The true/false value you want to se the button to.
  constexpr void set_button(int button_index, bool value) {
    if (button_index < 1 || button_index > button_count) {
      return;
    }
    // have to add 4 to button_index to account for the hat switch, but also
    // subtract 1 to make it 0-indexed
    button_index += 4 - 1; // yes this is 3.
    int byte_index = button_index / 8;
    buttons.raw[byte_index] &= ~(1 << (button_index % 8));
    if (value) {
      buttons.raw[byte_index] |= (1 << (button_index % 8));
    }
  }

  /// Set the hat switch value from espp::gamepad::Hat
  /// \param hat The espp::gamepad::Hat value to set.
  constexpr void set_hat_switch(espp::gamepad::Hat hat) { set_hat_switch(from_gamepad_hat(hat)); }

  /// Set the hat switch value
  /// \param hat The hat switch value to set.
  constexpr void set_hat_switch(Hat hat) { buttons.hat_switch = static_cast<std::uint8_t>(hat); }

  /// Set the hat switch value
  /// \param value The hat switch value to set.
  /// \note The value should match the values within the Hat enum.
  constexpr void set_hat_switch(std::uint8_t value) { buttons.hat_switch = (value & 0xf); }

  /// Set the hat switch (d-pad) value
  /// @param hat Hat enum / direction to set
  constexpr void set_hat(espp::gamepad::Hat hat) { set_hat_switch(from_gamepad_hat(hat)); }

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
  constexpr Hat get_hat() const { return static_cast<Hat>(buttons.hat_switch); }

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

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    auto report_data = raw.data();
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // copy the data into our data array
    std::copy(data.begin(), data.end(), raw.begin());
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
    using namespace hid::rdf::main;

    // clang-format off
      return descriptor(usage_page<playstation_vendor_00>(),
                        logical_limits<1, 2>(0, 255),
                        report_size(8),
                        report_count(77),

                        conditional_report_id<REPORT_ID>(),
                        hid::rdf::short_item<1>(local::tag::USAGE, 0x31),
                        output::absolute_variable(),
                        hid::rdf::short_item<1>(local::tag::USAGE, 0x3B),
                        input::absolute_variable()
                        );
    // clang-format on
  }

  friend fmt::formatter<PlaystationDualsenseBLEComplexInputReport<REPORT_ID>>;
};

/// Get the descriptor for the Playstation DualSense BLE controller
/// \return The descriptor for the Playstation DualSense controller over BT/LE
/// \note This is the complete descriptor for the Playstation Dualsense, and
///       matches byte-for-byte the descriptor provided by the actual device.
/// \note Most platforms only really parse the Complex Input Report (ID 49),
///       but the Simple Input Report (ID 48) is included for completeness.
[[maybe_unused]] static constexpr auto playstation_dualsense_ble_descriptor() {
  using namespace hid::page;
  using namespace hid::rdf;

  // clang-format off
    return descriptor(
      usage_page<generic_desktop>(),
      usage(generic_desktop::GAMEPAD),
      collection::application(PlaystationDualsenseBLESimpleInputReport<>::get_descriptor(),
                              PlaystationDualsenseBLEComplexInputReport<>::get_descriptor(),

                              report_id(50),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x32),
                              report_count(141),
                              output::absolute_variable(),

                              report_id(51),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x33),
                              report_count(205),
                              output::absolute_variable(),

                              report_id(52),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x34),
                              report_count<2>(269),
                              output::absolute_variable(),

                              report_id(53),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x35),
                              report_count<2>(333),
                              output::absolute_variable(),

                              report_id(54),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x36),
                              report_count<2>(397),
                              output::absolute_variable(),

                              report_id(55),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x37),
                              report_count<2>(461),
                              output::absolute_variable(),

                              report_id(56),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x38),
                              report_count<2>(525),
                              output::absolute_variable(),

                              report_id(57),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x39),
                              report_count<2>(546),
                              output::absolute_variable(),

                              usage_page<playstation_vendor_80>(),

                              report_id(5),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x33),
                              report_count(40),
                              feature::absolute_variable(),

                              report_id(8),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x34),
                              report_count(47),
                              feature::absolute_variable(),

                              report_id(9),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x24),
                              report_count(19),
                              feature::absolute_variable(),

                              report_id(32),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x26),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(34),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x40),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(128),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x28),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(129),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x29),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(130),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x2A),
                              report_count(9),
                              feature::absolute_variable(),

                              report_id(131),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x2B),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(241),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x31),
                              report_count(63),
                              feature::absolute_variable(),

                              report_id(242),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x32),
                              report_count(15),
                              feature::absolute_variable(),

                              report_id(240),
                              hid::rdf::short_item<1>(local::tag::USAGE, 0x30),
                              report_count(63),
                              feature::absolute_variable()
                              )
      );
  // clang-format on
} // playstation_dualsense_ble_descriptor

} // namespace espp

#include "hid-rp-playstation-formatters.hpp"

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "gamepad_imu.hpp"
#include "hid-rp-gamepad.hpp"

namespace espp {

#pragma pack(push, 1)

/// PS4 DualShock 4 Gamepad Buttons
/// Used in input report 0x01 over BLE
union PS4DualShock4GamepadButtons {
  std::array<std::uint8_t, 3> raw;
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
    std::uint8_t share : 1;   ///< Share button
    std::uint8_t options : 1; ///< Options button
    std::uint8_t l3 : 1;      ///< L3 button
    std::uint8_t r3 : 1;      ///< R3 button
    // byte 2
    std::uint8_t home : 1;     ///< PS button
    std::uint8_t touchpad : 1; ///< Touchpad button
    std::uint8_t counter : 6;  ///< Report counter
  };
};

/// PS4 DualShock 4 Touchpad Data
struct PS4TouchpadData {
  std::uint8_t touch_id : 7; ///< Touch ID
  std::uint8_t active : 1;   ///< Active touch flag
  std::uint16_t x : 12;      ///< X position (0-1919)
  std::uint16_t y : 12;      ///< Y position (0-941)
  std::uint8_t padding : 4;  ///< Padding
};

/// PS4 DualShock 4 Touchpad Report
struct PS4TouchpadReport {
  std::uint8_t timestamp;   ///< Timestamp
  PS4TouchpadData touch[2]; ///< Two touch points
};

#pragma pack(pop)

/// HID PS4 DualShock 4 Gamepad Input Report
///
/// This class implements a HID PS4 DualShock 4 Gamepad Input Report for BLE.
/// It supports 14 buttons, a d-pad (hat switch), 2 joystick axes, 2 analog triggers,
/// touchpad data, IMU (accelerometer and gyroscope), and battery status.
///
/// The PS4 DualShock 4 uses report ID 0x01 for input reports over BLE (77 bytes).
/// The report structure follows the official Sony PS4 DualShock 4 HID descriptor.
///
/// Button mapping:
/// - Buttons 1-4: Square, Cross, Circle, Triangle
/// - Buttons 5-8: L1, R1, L2, R2
/// - Buttons 9-12: Share, Options, L3, R3
/// - Buttons 13-14: PS Home, Touchpad Click
///
/// \section hid_rp_ps4_references References
/// - Protocol documentation: https://www.psdevwiki.com/ps4/DS4-BT
/// - Protocol documentation: https://www.psdevwiki.com/ps4/DS4-USB
/// - HID descriptor:
/// https://github.com/DJm00n/ControllersInfo/blob/master/dualshock4/dualshock4_hid_report_descriptor.txt
/// - Linux kernel: https://github.com/torvalds/linux/blob/master/drivers/hid/hid-sony.c
/// - Community reverse engineering: https://github.com/chrippa/ds4drv
///
/// \section hid_rp_ps4_ex1 HID-RP PS4 Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 0x01>
class PS4DualShock4GamepadInputReport
    : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  using Hat = espp::gamepad::Hat;
  using Accelerometer = espp::gamepad::Accelerometer;
  using Gyroscope = espp::gamepad::Gyroscope;

  static constexpr size_t button_count = 14;

  using JOYSTICK_TYPE = std::uint8_t;
  static constexpr JOYSTICK_TYPE joystick_min = 0;
  static constexpr JOYSTICK_TYPE joystick_max = 255;
  static constexpr JOYSTICK_TYPE joystick_center = 128;
  static constexpr size_t joystick_value_range = joystick_max - joystick_min;
  static constexpr JOYSTICK_TYPE joystick_range = joystick_value_range / 2;

  using TRIGGER_TYPE = std::uint8_t;
  static constexpr TRIGGER_TYPE trigger_min = 0;
  static constexpr TRIGGER_TYPE trigger_max = 255;

protected:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t left_stick_x;           ///< Left stick X axis
      std::uint8_t left_stick_y;           ///< Left stick Y axis
      std::uint8_t right_stick_x;          ///< Right stick X axis
      std::uint8_t right_stick_y;          ///< Right stick Y axis
      PS4DualShock4GamepadButtons buttons; ///< Button states
      std::uint8_t l2_trigger;             ///< L2 trigger analog value
      std::uint8_t r2_trigger;             ///< R2 trigger analog value
      std::uint16_t timestamp;             ///< Timestamp
      std::uint8_t battery;                ///< Battery level
      std::int16_t gyro_x;                 ///< Gyroscope X axis
      std::int16_t gyro_y;                 ///< Gyroscope Y axis
      std::int16_t gyro_z;                 ///< Gyroscope Z axis
      std::int16_t accel_x;                ///< Accelerometer X axis
      std::int16_t accel_y;                ///< Accelerometer Y axis
      std::int16_t accel_z;                ///< Accelerometer Z axis
      std::uint8_t reserved[5];            ///< Reserved bytes
      std::uint8_t extension;              ///< Extension byte
      std::uint8_t reserved2[2];           ///< Reserved bytes
      PS4TouchpadReport touchpad[3];       ///< Up to 3 touchpad reports
      std::uint8_t reserved3[3];           ///< Reserved bytes
    };
    std::array<std::uint8_t, 77> raw; ///< Raw report data
  };
#pragma pack(pop)

public:
  static constexpr size_t num_data_bytes = sizeof(raw);
  constexpr PS4DualShock4GamepadInputReport() { reset(); }

  constexpr void reset() {
    std::fill(raw.begin(), raw.end(), 0);
    left_stick_x = joystick_center;
    left_stick_y = joystick_center;
    right_stick_x = joystick_center;
    right_stick_y = joystick_center;
    buttons.raw[0] = 0x08; // hat_switch centered
  }

  // Joystick methods
  constexpr void set_left_joystick(JOYSTICK_TYPE x, JOYSTICK_TYPE y) {
    left_stick_x = x;
    left_stick_y = y;
  }

  constexpr void set_right_joystick(JOYSTICK_TYPE x, JOYSTICK_TYPE y) {
    right_stick_x = x;
    right_stick_y = y;
  }

  constexpr void set_left_joystick_x(JOYSTICK_TYPE x) { left_stick_x = x; }
  constexpr void set_left_joystick_y(JOYSTICK_TYPE y) { left_stick_y = y; }
  constexpr void set_right_joystick_x(JOYSTICK_TYPE x) { right_stick_x = x; }
  constexpr void set_right_joystick_y(JOYSTICK_TYPE y) { right_stick_y = y; }

  constexpr JOYSTICK_TYPE get_left_joystick_x() const { return left_stick_x; }
  constexpr JOYSTICK_TYPE get_left_joystick_y() const { return left_stick_y; }
  constexpr JOYSTICK_TYPE get_right_joystick_x() const { return right_stick_x; }
  constexpr JOYSTICK_TYPE get_right_joystick_y() const { return right_stick_y; }

  // Trigger methods
  constexpr void set_l2_trigger(TRIGGER_TYPE value) { l2_trigger = value; }
  constexpr void set_r2_trigger(TRIGGER_TYPE value) { r2_trigger = value; }
  constexpr TRIGGER_TYPE get_l2_trigger() const { return l2_trigger; }
  constexpr TRIGGER_TYPE get_r2_trigger() const { return r2_trigger; }

  // D-pad / Hat methods
  constexpr void set_hat(Hat hat_value) {
    buttons.hat_switch = static_cast<std::uint8_t>(hat_value);
  }

  constexpr Hat get_hat() const { return static_cast<Hat>(buttons.hat_switch); }

  // Button methods
  constexpr void set_button_square(bool pressed) { buttons.square = pressed; }
  constexpr void set_button_cross(bool pressed) { buttons.cross = pressed; }
  constexpr void set_button_circle(bool pressed) { buttons.circle = pressed; }
  constexpr void set_button_triangle(bool pressed) { buttons.triangle = pressed; }
  constexpr void set_button_l1(bool pressed) { buttons.l1 = pressed; }
  constexpr void set_button_r1(bool pressed) { buttons.r1 = pressed; }
  constexpr void set_button_l2(bool pressed) { buttons.l2 = pressed; }
  constexpr void set_button_r2(bool pressed) { buttons.r2 = pressed; }
  constexpr void set_button_share(bool pressed) { buttons.share = pressed; }
  constexpr void set_button_options(bool pressed) { buttons.options = pressed; }
  constexpr void set_button_l3(bool pressed) { buttons.l3 = pressed; }
  constexpr void set_button_r3(bool pressed) { buttons.r3 = pressed; }
  constexpr void set_button_home(bool pressed) { buttons.home = pressed; }
  constexpr void set_button_touchpad(bool pressed) { buttons.touchpad = pressed; }

  constexpr bool get_button_square() const { return buttons.square; }
  constexpr bool get_button_cross() const { return buttons.cross; }
  constexpr bool get_button_circle() const { return buttons.circle; }
  constexpr bool get_button_triangle() const { return buttons.triangle; }
  constexpr bool get_button_l1() const { return buttons.l1; }
  constexpr bool get_button_r1() const { return buttons.r1; }
  constexpr bool get_button_l2() const { return buttons.l2; }
  constexpr bool get_button_r2() const { return buttons.r2; }
  constexpr bool get_button_share() const { return buttons.share; }
  constexpr bool get_button_options() const { return buttons.options; }
  constexpr bool get_button_l3() const { return buttons.l3; }
  constexpr bool get_button_r3() const { return buttons.r3; }
  constexpr bool get_button_home() const { return buttons.home; }
  constexpr bool get_button_touchpad() const { return buttons.touchpad; }

  // Battery methods
  constexpr void set_battery_level(std::uint8_t level) {
    battery = (level & 0x0F) | (battery & 0xF0);
  }

  constexpr void set_battery_charging(bool charging) {
    if (charging) {
      battery |= 0x10;
    } else {
      battery &= 0x0F;
    }
  }

  constexpr std::uint8_t get_battery_level() const { return battery & 0x0F; }
  constexpr bool get_battery_charging() const { return (battery & 0x10) != 0; }

  // IMU methods
  constexpr void set_gyroscope(const Gyroscope &gyro) {
    gyro_x = gyro.X;
    gyro_y = gyro.Y;
    gyro_z = gyro.Z;
  }

  constexpr void set_accelerometer(const Accelerometer &accel) {
    accel_x = accel.X;
    accel_y = accel.Y;
    accel_z = accel.Z;
  }

  constexpr Gyroscope get_gyroscope() const { return {{gyro_x, gyro_y, gyro_z}}; }

  constexpr Accelerometer get_accelerometer() const { return {{accel_x, accel_y, accel_z}}; }

  // Timestamp methods
  constexpr void set_timestamp(std::uint16_t ts) { timestamp = ts; }
  constexpr std::uint16_t get_timestamp() const { return timestamp; }

  // Counter methods
  constexpr void set_counter(std::uint8_t count) { buttons.counter = count & 0x3F; }
  constexpr std::uint8_t get_counter() const { return buttons.counter; }

  // Touchpad methods
  constexpr void set_touchpad_data(size_t report_idx, size_t touch_idx, bool active,
                                   std::uint16_t x, std::uint16_t y) {
    if (report_idx < 3 && touch_idx < 2) {
      auto &touch = touchpad[report_idx].touch[touch_idx];
      touch.active = active;
      touch.x = x & 0x0FFF;
      touch.y = y & 0x0FFF;
    }
  }

  constexpr void set_touchpad_timestamp(size_t report_idx, std::uint8_t ts) {
    if (report_idx < 3) {
      touchpad[report_idx].timestamp = ts;
    }
  }

  // Serialization methods
  constexpr auto get_report() const { return std::vector<uint8_t>(raw.begin() + 1, raw.end()); }

  constexpr void set_data(const std::vector<uint8_t> &data) {
    std::copy(data.begin(), data.end(), raw.begin() + 1);
  }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
    // Based on official PS4 DualShock 4 Bluetooth HID descriptor
    // Reference: https://github.com/DJm00n/ControllersInfo/blob/master/dualshock4/dualshock4_hid_report_descriptor.txt
    // Reference: Linux kernel drivers/hid/hid-sony.c
    return descriptor(
      conditional_report_id<REPORT_ID>(),
      usage_page<generic_desktop>(),
      
      // Left stick X, Y and Right stick X (Z), Y (Rz) - 4 bytes
      usage(generic_desktop::X),
      usage(generic_desktop::Y),
      usage(generic_desktop::Z),
      usage(generic_desktop::RZ),
      logical_limits<1, 1>(joystick_min, joystick_max),
      report_size(8),
      report_count(4),
      input::absolute_variable(),
      
      // D-pad / Hat switch - 4 bits
      usage(generic_desktop::HAT_SWITCH),
      logical_limits<1, 1>(0, 7),
      physical_limits<1, 2>(0, 315),
      unit::unit<1>(unit::code::DEGREE),
      report_size(4),
      report_count(1),
      input::absolute_variable(input::flags::NULL_STATE),
      
      unit::unit<1>(unit::code::NONE),
      
      // Face buttons: Square, Cross, Circle, Triangle - 4 bits
      usage_page<button>(),
      usage_limits(button(1), button(4)),
      logical_limits<1, 1>(0, 1),
      report_size(1),
      report_count(4),
      input::absolute_variable(),
      
      // Shoulder buttons: L1, R1, L2, R2 - 4 bits
      usage_limits(button(5), button(8)),
      report_count(4),
      input::absolute_variable(),
      
      // System buttons: Share, Options, L3, R3 - 4 bits
      usage_limits(button(9), button(12)),
      report_count(4),
      input::absolute_variable(),
      
      // Special buttons: PS Home, Touchpad - 2 bits
      usage_limits(button(13), button(14)),
      report_count(2),
      input::absolute_variable(),
      
      // Report counter - 6 bits
      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
      hid::rdf::short_item<1>(local::tag::USAGE, 0x20),
      logical_limits<1, 1>(0, 0x3F),
      report_size(6),
      report_count(1),
      input::absolute_variable(),
      
      // L2/R2 analog triggers - 2 bytes
      usage_page<generic_desktop>(),
      usage(generic_desktop::RX),
      usage(generic_desktop::RY),
      logical_limits<1, 1>(trigger_min, trigger_max),
      report_size(8),
      report_count(2),
      input::absolute_variable(),
      
      // Vendor-defined data (timestamp, battery, IMU, touchpad, etc.) - 54 bytes
      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
      report_count(54),
      input::absolute_variable()
    );
    // clang-format on
  }

  friend fmt::formatter<PS4DualShock4GamepadInputReport<REPORT_ID>>;
};

/// HID PS4 DualShock 4 Feature Report (0x02)
///
/// This class implements a HID PS4 DualShock 4 Feature Report for BLE.
/// Report ID 0x02 - 37 bytes
template <uint8_t REPORT_ID = 0x02>
class PS4DualShock4FeatureReport02
    : public hid::report::base<hid::report::type::FEATURE, REPORT_ID> {
public:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t data[37]; ///< Feature data
    };
    std::array<std::uint8_t, 38> raw;
  };
#pragma pack(pop)

  constexpr PS4DualShock4FeatureReport02() { reset(); }

  constexpr void reset() { std::fill(raw.begin(), raw.end(), 0); }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
                      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
                      logical_limits<1, 1>(0, 255), report_size(8), report_count(37),
                      feature::absolute_variable());
  }
};

/// HID PS4 DualShock 4 Feature Report (0x04)
///
/// This class implements a HID PS4 DualShock 4 Feature Report for BLE.
/// Report ID 0x04 - 41 bytes
template <uint8_t REPORT_ID = 0x04>
class PS4DualShock4FeatureReport04
    : public hid::report::base<hid::report::type::FEATURE, REPORT_ID> {
public:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t data[41]; ///< Feature data
    };
    std::array<std::uint8_t, 42> raw;
  };
#pragma pack(pop)

  constexpr PS4DualShock4FeatureReport04() { reset(); }

  constexpr void reset() { std::fill(raw.begin(), raw.end(), 0); }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
                      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
                      logical_limits<1, 1>(0, 255), report_size(8), report_count(41),
                      feature::absolute_variable());
  }
};

/// HID PS4 DualShock 4 Feature Report (0xF1)
///
/// This class implements a HID PS4 DualShock 4 Feature Report for BLE.
/// Report ID 0xF1 - 63 bytes (device info)
template <uint8_t REPORT_ID = 0xF1>
class PS4DualShock4FeatureReportF1
    : public hid::report::base<hid::report::type::FEATURE, REPORT_ID> {
public:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t data[63]; ///< Feature data (device info, MAC, etc.)
    };
    std::array<std::uint8_t, 64> raw;
  };
#pragma pack(pop)

  constexpr PS4DualShock4FeatureReportF1() { reset(); }

  constexpr void reset() { std::fill(raw.begin(), raw.end(), 0); }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
                      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
                      logical_limits<1, 1>(0, 255), report_size(8), report_count(63),
                      feature::absolute_variable());
  }
};

/// HID PS4 DualShock 4 Feature Report (0xF2)
///
/// This class implements a HID PS4 DualShock 4 Feature Report for BLE.
/// Report ID 0xF2 - 63 bytes (firmware info)
template <uint8_t REPORT_ID = 0xF2>
class PS4DualShock4FeatureReportF2
    : public hid::report::base<hid::report::type::FEATURE, REPORT_ID> {
public:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t data[63]; ///< Feature data (firmware info)
    };
    std::array<std::uint8_t, 64> raw;
  };
#pragma pack(pop)

  constexpr PS4DualShock4FeatureReportF2() { reset(); }

  constexpr void reset() { std::fill(raw.begin(), raw.end(), 0); }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
                      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
                      logical_limits<1, 1>(0, 255), report_size(8), report_count(63),
                      feature::absolute_variable());
  }
};

/// HID PS4 DualShock 4 Output Report
///
/// This class implements a HID PS4 DualShock 4 Output Report for BLE.
/// It supports rumble (left/right motors), LED color, and flash settings.
///
/// \note See https://www.psdevwiki.com/ps4/DS4-BT for BLE output report format
/// \note See https://www.psdevwiki.com/ps4/DS4-USB for USB output report format
/// \note For BLE, the report uses report ID 0x11 (17 decimal)
/// \note The official descriptor uses report ID 0x05 for OUTPUT reports (31 bytes)
template <uint8_t REPORT_ID = 0x05>
class PS4DualShock4OutputReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
public:
#pragma pack(push, 1)
  union {
    struct {
      std::uint8_t data[31]; ///< Output data (vendor-defined format)
    };
    std::array<std::uint8_t, 32> raw;
  };
#pragma pack(pop)

  static constexpr size_t num_data_bytes = sizeof(raw);

  constexpr PS4DualShock4OutputReport() { reset(); }

  constexpr void reset() { std::fill(raw.begin(), raw.end(), 0); }

  constexpr void set_rumble(std::uint8_t left, std::uint8_t right) {
    // For BLE report 0x11, rumble is at offsets 3 and 4
    // data[3] = right (weak motor)
    // data[4] = left (strong motor)
    if (REPORT_ID == 0x11) {
      data[3] = right;
      data[4] = left;
    }
  }

  constexpr void set_led_color(std::uint8_t r, std::uint8_t g, std::uint8_t b) {
    // For BLE report 0x11, LED color is at offsets 5-7
    if (REPORT_ID == 0x11) {
      data[5] = r;
      data[6] = g;
      data[7] = b;
    }
  }

  constexpr void set_led_flash(std::uint8_t on_time, std::uint8_t off_time) {
    // For BLE report 0x11, flash times are at offsets 8-9
    if (REPORT_ID == 0x11) {
      data[8] = on_time;
      data[9] = off_time;
    }
  }

  constexpr auto get_report() const { return std::vector<uint8_t>(raw.begin() + 1, raw.end()); }

  constexpr void set_data(const std::vector<uint8_t> &data_in) {
    std::copy(data_in.begin(), data_in.end(), raw.begin() + 1);
  }

  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // Based on official PS4 DualShock 4 Bluetooth HID descriptor
    // Output report with vendor-defined format (31 bytes for report ID 0x05)
    // Reference:
    // https://github.com/DJm00n/ControllersInfo/blob/master/dualshock4/dualshock4_hid_report_descriptor.txt
    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<2>(global::tag::USAGE_PAGE, 0xFF00),
                      hid::rdf::short_item<1>(local::tag::USAGE, 0x21),
                      logical_limits<1, 1>(0, 255), report_size(8), report_count(31),
                      output::absolute_variable());
  }
};

/// Get the descriptor for the PS4 DualShock 4 controller
/// \return The descriptor for the PS4 DualShock 4 controller
[[maybe_unused]] static constexpr auto ps4_dualshock4_descriptor() {
  using namespace hid::page;
  using namespace hid::rdf;

  // clang-format off
  // Complete PS4 DualShock 4 BLE descriptor with all reports
  // Reference: https://github.com/DJm00n/ControllersInfo/blob/master/dualshock4/dualshock4_hid_report_descriptor.txt
  // Reference: Linux kernel drivers/hid/hid-sony.c
  return descriptor(
    usage_page<generic_desktop>(),
    usage(generic_desktop::GAMEPAD),
    collection::application(
      PS4DualShock4GamepadInputReport<>::get_descriptor(),
      PS4DualShock4FeatureReport02<>::get_descriptor(),
      PS4DualShock4FeatureReport04<>::get_descriptor(),
      PS4DualShock4OutputReport<>::get_descriptor(),
      PS4DualShock4FeatureReportF1<>::get_descriptor(),
      PS4DualShock4FeatureReportF2<>::get_descriptor()
    )
  );
  // clang-format on
}

} // namespace espp

#include "hid-rp-ps4-formatters.hpp"

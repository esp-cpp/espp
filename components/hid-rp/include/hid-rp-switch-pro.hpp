#pragma once

#include <algorithm>

#include "gamepad_imu.hpp"
#include "hid-rp-gamepad.hpp"

namespace hid::page {
enum class switch_vendor : std::uint16_t;
template <> constexpr inline auto get_info<switch_vendor>() {
  return info(
      0xFF00, 0xFFFF, "Switch Vendor Page (0xFF00)",
      [](hid::usage_id_t id) {
        switch (id) {
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class switch_vendor : std::uint16_t {};
} // namespace hid::page

namespace espp {
/// Button struct concept for the Switch Pro controller This concept defines
/// the requirements for a struct that contains the button values for the
/// Switch Pro controller.
template <typename T>
concept SwitchProButtonStruct = requires(T t) {
  { auto(t.a) } -> std::integral;
  { auto(t.b) } -> std::integral;
  { auto(t.x) } -> std::integral;
  { auto(t.y) } -> std::integral;
  { auto(t.l) } -> std::integral;
  { auto(t.r) } -> std::integral;
  { auto(t.zl) } -> std::integral;
  { auto(t.zr) } -> std::integral;
  { auto(t.dpad_up) } -> std::integral;
  { auto(t.dpad_down) } -> std::integral;
  { auto(t.dpad_left) } -> std::integral;
  { auto(t.dpad_right) } -> std::integral;
  { auto(t.thumb_l) } -> std::integral;
  { auto(t.thumb_r) } -> std::integral;
  { auto(t.home) } -> std::integral;
  { auto(t.capture) } -> std::integral;
  { auto(t.minus) } -> std::integral;
  { auto(t.plus) } -> std::integral;
  { auto(t.left_sr) } -> std::integral;
  { auto(t.left_sl) } -> std::integral;
  { auto(t.right_sr) } -> std::integral;
  { auto(t.right_sl) } -> std::integral;
};

/// HID Switch Pro Gamepad Input Report
///
/// This class implements a HID Switch Pro Gamepad Input Report. It supports 15
/// buttons, a d-pad, 4 joystick axes and two trigger buttons. It supports
/// setting the buttons, d-pad, joysticks, and triggers, as well as serializing
/// the input report and getting the report descriptor.
///
/// \section hid_rp_switch_pro_ex1 HID-RP Switch Pro Example
/// \snippet hid_rp_example.cpp hid rp example
template <uint8_t REPORT_ID = 0x30>
class SwitchProGamepadInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
public:
  using Hat = espp::gamepad::Hat;

  static constexpr size_t button_count = 15;

  using JOYSTICK_TYPE = std::int16_t;
  static constexpr JOYSTICK_TYPE joystick_min = 0;
  static constexpr JOYSTICK_TYPE joystick_max = 4096;
  static constexpr JOYSTICK_TYPE joystick_center = (joystick_min + joystick_max) / 2;
  static constexpr size_t joystick_value_range = joystick_max - joystick_min;
  static constexpr JOYSTICK_TYPE joystick_range = joystick_value_range / 2;

  using Accelerometer = espp::gamepad::Accelerometer;
  using Gyroscope = espp::gamepad::Gyroscope;

protected:
  // union for the input report data
  union {
    // struct for the input report and follow-up data such as IMU or command
    struct {
      // union for the input report data
      union {
        // See
        // https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md#standard-input-report-format
        // for more information.
        struct {
          // Byte 0: Counter which increments with each packet
          uint8_t counter;
          // Byte 1:
          // - low nibble: Connection info.
          //     Bit 0 is USB powered, bits 1-2 are connection info.
          // - high nibble: Battery level.
          //     8=full, 6=medium, 4=low, 2=critical, 0=empty. Bit 0 is charging status.
          //     low = 10%,
          //     medium = 50%,
          //     full = 100%
          uint8_t connection_info : 4;
          uint8_t battery_charging : 1;
          uint8_t battery_level : 3;
          // Byte 2, 3, 4: Button status
          bool btn_y : 1;
          bool btn_x : 1;
          bool btn_b : 1;
          bool btn_a : 1;
          bool btn_right_sr : 1;
          bool btn_right_sl : 1;
          bool btn_r : 1;
          bool btn_zr : 1;
          //    byte boundary
          bool btn_minus : 1;
          bool btn_plus : 1;
          bool btn_thumb_r : 1;
          bool btn_thumb_l : 1;
          bool btn_home : 1;
          bool btn_capture : 1;
          uint8_t dummy : 1;
          bool charging_grip : 1;
          //    byte boundary
          bool dpad_down : 1;
          bool dpad_up : 1;
          bool dpad_right : 1;
          bool dpad_left : 1;
          bool btn_left_sr : 1;
          bool btn_left_sl : 1;
          bool btn_l : 1;
          bool btn_zl : 1;
          // Byte 5, 6, 7: left joystick data
          // Byte 8, 9, 10: right joystick data
          union {
            uint8_t analog[6];
            struct {
              uint16_t joy_lx : 12;
              uint16_t joy_ly : 12;
              uint16_t joy_rx : 12;
              uint16_t joy_ry : 12;
            } __attribute__((packed));
          } __attribute__((packed));
          // Byte 11: Vibrator input report.
          //          Decides if next vibration pattern should be sent.
          uint8_t vibrator_input_report;
        } __attribute__((packed)); // input report data struct
        uint8_t raw_input_report[12];
      } __attribute__((packed)); // input report data union
      // Union for post-input report data such as IMU or command replies
      union {
        // for report IDs 0x30, 0x31, 0x32, 0x33, this is 6-axis data. 3 frames of 2
        // groups of 2 int16LE each. group is ACC followed by GYRO.
        struct {
          union {
            uint8_t imuData[36]; // 6-axis data; 3 frames of 2 groups of 2 int16LE each
            struct {
              // Frame 0
              Accelerometer acc_0;
              Gyroscope gyro_0;
              // Frame 1
              Accelerometer acc_1;
              Gyroscope gyro_1;
              // Frame 2
              Accelerometer acc_2;
              Gyroscope gyro_2;
            } __attribute__((packed));
          } __attribute__((packed));
        } __attribute__((packed)); //
        // 0x21 subcommand reply data; max len 35
        struct {
          uint8_t subcmd_ack;
          uint8_t subcmd_id;
          uint8_t subcmd_reply[35];
        } __attribute__((packed));
        // TODO: for report ID 0x23, this is NFC/IR MCU FW update input report (max len 37)
        //
        // TODO: for report id 0x31, there are aditional 313 bytes of NFC/IR data input
        // after this.
      } __attribute__((packed)); // data union
    } __attribute__((packed));   // input report data struct
    // this will ensure we always have enough space for the largest report
    // without having padding bytes defined anywhere.
    uint8_t raw_report[63];
  } __attribute__((packed));

  static constexpr size_t num_data_bytes = 63;

public:
  /// Construct a new Gamepad Input Report object
  constexpr SwitchProGamepadInputReport() = default;

  /// Reset the gamepad inputs
  constexpr void reset() {
    uint8_t prev_counter = counter;
    // fill all 0s
    std::fill(raw_report, raw_report + sizeof(raw_report), 0);
    // now set the counter to the previous value
    counter = prev_counter; // cppcheck-suppress redundantAssignment
    // set the joysticks to 0 since their config may not put them at 0
    set_left_joystick(0, 0);
    set_right_joystick(0, 0);
  }

  /// Set the counter
  /// @param value The value to set the counter to.
  constexpr void set_counter(uint8_t value) { counter = value; }

  /// Increment the counter
  constexpr void increment_counter() { counter = (counter + 1); }

  /// Get the counter value
  /// @return The counter value
  constexpr uint8_t get_counter() const { return counter; }

  /// Set the subcommand ACK
  /// @param ack The subcommand ACK to set
  constexpr void set_subcmd_ack(uint8_t ack) { subcmd_ack = ack; }

  /// Get the subcommand ACK
  /// @return The subcommand ACK
  constexpr uint8_t get_subcmd_ack() const { return subcmd_ack; }

  /// Set the subcommand ID
  /// @param id The subcommand ID to set
  constexpr void set_subcmd_id(uint8_t id) { subcmd_id = id; }

  /// Get the subcommand ID
  /// @return The subcommand ID
  constexpr uint8_t get_subcmd_id() const { return subcmd_id; }

  /// Set the subcommand reply
  /// @param reply The subcommand reply to set
  constexpr void set_subcmd_reply(const std::vector<uint8_t> &reply) {
    std::copy(reply.begin(), reply.end(), subcmd_reply);
  }

  /// Get the subcommand reply
  /// @return The subcommand reply
  constexpr auto get_subcmd_reply() const {
    return std::vector<uint8_t>(subcmd_reply, subcmd_reply + sizeof(subcmd_reply));
  }

  /// Set the left joystick X and Y axis values
  /// @param lx left joystick x axis value, in the range [-1, 1]
  /// @param ly left joystick y axis value, in the range [-1, 1]
  constexpr void set_left_joystick(float lx, float ly) {
    lx = std::clamp(lx, -1.0f, 1.0f);
    ly = std::clamp(ly, -1.0f, 1.0f);
    JOYSTICK_TYPE x = std::clamp(static_cast<JOYSTICK_TYPE>(lx * joystick_range + joystick_center),
                                 joystick_min, joystick_max);
    JOYSTICK_TYPE y = std::clamp(static_cast<JOYSTICK_TYPE>(ly * joystick_range + joystick_center),
                                 joystick_min, joystick_max);
    joy_lx = x;
    joy_ly = y;
  }

  /// Set the right joystick X and Y axis values
  /// @param rx right joystick x axis value, in the range [-1, 1]
  /// @param ry right joystick y axis value, in the range [-1, 1]
  constexpr void set_right_joystick(float rx, float ry) {
    rx = std::clamp(rx, -1.0f, 1.0f);
    ry = std::clamp(ry, -1.0f, 1.0f);
    JOYSTICK_TYPE x = std::clamp(static_cast<JOYSTICK_TYPE>(rx * joystick_range + joystick_center),
                                 joystick_min, joystick_max);
    JOYSTICK_TYPE y = std::clamp(static_cast<JOYSTICK_TYPE>(ry * joystick_range + joystick_center),
                                 joystick_min, joystick_max);
    joy_rx = x;
    joy_ry = y;
  }

  /// Get the left joystick X and Y axis values
  /// @param lx left joystick x axis value, in the range [-1, 1]
  /// @param ly left joystick y axis value, in the range [-1, 1]
  constexpr void get_left_joystick(float &lx, float &ly) const {
    JOYSTICK_TYPE x = joy_lx;
    JOYSTICK_TYPE y = joy_ly;
    lx = static_cast<float>(x - joystick_center) / joystick_range;
    ly = static_cast<float>(y - joystick_center) / joystick_range;
  }

  /// Get the left joystick X and Y axis values
  /// @param lx left joystick x axis value
  /// @param ly left joystick y axis value
  constexpr void get_left_joystick(JOYSTICK_TYPE &lx, JOYSTICK_TYPE &ly) const {
    JOYSTICK_TYPE x = joy_lx;
    JOYSTICK_TYPE y = joy_ly;
    lx = x;
    ly = y;
  }

  /// Get the right joystick X and Y axis values
  /// @param rx right joystick x axis value, in the range [-1, 1]
  /// @param ry right joystick y axis value, in the range [-1, 1]
  constexpr void get_right_joystick(float &rx, float &ry) const {
    JOYSTICK_TYPE x = joy_rx;
    JOYSTICK_TYPE y = joy_ry;
    rx = static_cast<float>(x - joystick_center) / joystick_range;
    ry = static_cast<float>(y - joystick_center) / joystick_range;
  }

  /// Get the right joystick X and Y axis values
  /// @param rx right joystick x axis value
  /// @param ry right joystick y axis value
  constexpr void get_right_joystick(JOYSTICK_TYPE &rx, JOYSTICK_TYPE &ry) const {
    JOYSTICK_TYPE x = joy_rx;
    JOYSTICK_TYPE y = joy_ry;
    rx = x;
    ry = y;
  }

  /// Set the left trigger value
  /// @param value The value to set the left trigger to.
  ///        Should be in the range [0, 1].
  constexpr void set_left_trigger(float value) { set_trigger_axis(0, value); }

  /// Set the right trigger value
  /// @param value The value to set the right trigger to.
  constexpr void set_left_trigger(bool pressed) { btn_zl = pressed; }

  /// Set the right trigger value
  /// @param value The value to set the right trigger to.
  ///        Should be in the range [0, 1].
  constexpr void set_right_trigger(float value) { set_trigger_axis(1, value); }

  /// Set the right trigger value
  /// @param value The value to set the right trigger to.
  constexpr void set_right_trigger(bool pressed) { btn_zr = pressed; }

  /// Set the brake trigger value
  /// @param pressed Whether the brake trigger is pressed or not
  constexpr void set_brake(bool pressed) { set_trigger_axis(0, pressed); }

  /// Set the brake trigger value
  /// @param value The value to set the brake trigger to.
  ///        Should be in the range [0, 1].
  constexpr void set_brake(float value) { set_trigger_axis(0, value); }

  /// Set the accelerator trigger value
  /// @param pressed Whether the accelerator trigger is pressed or not
  constexpr void set_accelerator(bool pressed) { set_trigger_axis(1, pressed); }

  /// Set the accelerator trigger value
  /// @param value The value to set the accelerator trigger to.
  ///        Should be in the range [0, 1].
  constexpr void set_accelerator(float value) { set_trigger_axis(1, value); }

  /// Get the brake trigger value
  /// @param value The value of the brake trigger
  constexpr void get_brake(float &value) const { value = btn_zl ? 1.0f : 0.0f; }

  /// Get the accelerator trigger value
  /// @param value The value of the accelerator trigger
  constexpr void get_accelerator(float &value) const { value = btn_zr ? 1.0f : 0.0f; }

  /// Set the battery level
  /// @param level battery level, in the range [0, 100]
  constexpr void set_battery_level(float level) {
    // now set the level bits (3 bits)
    if (level > 90) {
      battery_level = 4;
    } else if (level > 60) {
      battery_level = 3;
    } else if (level > 30) {
      battery_level = 2;
    } else if (level > 10) {
      battery_level = 1;
    } else {
      battery_level = 0;
    }
  }

  /// Set the battery charging status
  /// @param charging whether the battery is charging or not
  constexpr void set_battery_charging(bool charging) { battery_charging = charging; }

  /// Set the connection info
  /// @param info connection info
  constexpr void set_connection_info(uint8_t info) {
    // (conn_info >> 1) & 3 == 3: JC,
    // (conn_info >> 1) & 3 == 0: Pro Controller / ChrGrip
    //  conn_info & 1       == 1: switch / usb powered
    connection_info = info;
  }

  constexpr void set_usb_powered(bool powered) {
    // usb powered is the least significant bit of the connection info
    // so we just need to set the least significant bit of the connection info
    connection_info = (connection_info & 0xE) | (powered ? 1 : 0);
  }

  /// Set the dpad value
  /// @param up up dpad value
  /// @param down down dpad value
  /// @param left left dpad value
  /// @param right right dpad value
  constexpr void set_dpad(bool up, bool down, bool left, bool right) {
    dpad_up = up;
    dpad_down = down;
    dpad_left = left;
    dpad_right = right;
  }

  /// Set the d-pad as a hat switch value from espp::gamepad::Hat
  /// \param hat The espp::gamepad::Hat value to set.
  constexpr void set_hat_switch(espp::gamepad::Hat hat) {
    switch (hat) {
    case espp::gamepad::Hat::UP:
      set_dpad(true, false, false, false);
      break;
    case espp::gamepad::Hat::UP_RIGHT:
      set_dpad(true, false, false, true);
      break;
    case espp::gamepad::Hat::RIGHT:
      set_dpad(false, false, false, true);
      break;
    case espp::gamepad::Hat::DOWN_RIGHT:
      set_dpad(false, true, false, true);
      break;
    case espp::gamepad::Hat::DOWN:
      set_dpad(false, true, false, false);
      break;
    case espp::gamepad::Hat::DOWN_LEFT:
      set_dpad(false, true, true, false);
      break;
    case espp::gamepad::Hat::LEFT:
      set_dpad(false, false, true, false);
      break;
    case espp::gamepad::Hat::UP_LEFT:
      set_dpad(true, false, true, false);
      break;
    case espp::gamepad::Hat::CENTERED:
    default:
      set_dpad(false, false, false, false);
      break;
    }
  }

  /// Set the hat switch (d-pad) value
  /// @param hat Hat enum / direction to set
  constexpr void set_hat(espp::gamepad::Hat hat) { set_hat_switch(hat); }

  constexpr void set_button_a(bool pressed) { btn_a = pressed; }
  constexpr void set_button_b(bool pressed) { btn_b = pressed; }
  constexpr void set_button_x(bool pressed) { btn_x = pressed; }
  constexpr void set_button_y(bool pressed) { btn_y = pressed; }

  constexpr void set_button_l1(bool pressed) { btn_l = pressed; }
  constexpr void set_button_r1(bool pressed) { btn_r = pressed; }
  constexpr void set_button_l2(bool pressed) { btn_zl = pressed; }
  constexpr void set_button_r2(bool pressed) { btn_zr = pressed; }
  constexpr void set_button_l3(bool pressed) { btn_thumb_l = pressed; }
  constexpr void set_button_r3(bool pressed) { btn_thumb_r = pressed; }

  constexpr void set_button_l(bool pressed) { btn_l = pressed; }
  constexpr void set_button_r(bool pressed) { btn_r = pressed; }
  constexpr void set_button_zl(bool pressed) { btn_zl = pressed; }
  constexpr void set_button_zr(bool pressed) { btn_zr = pressed; }
  constexpr void set_button_thumb_l(bool pressed) { btn_thumb_l = pressed; }
  constexpr void set_button_thumb_r(bool pressed) { btn_thumb_r = pressed; }

  constexpr void set_button_home(bool pressed) { btn_home = pressed; }
  constexpr void set_button_capture(bool pressed) { btn_capture = pressed; }
  constexpr void set_button_plus(bool pressed) { btn_plus = pressed; }
  constexpr void set_button_minus(bool pressed) { btn_minus = pressed; }

  constexpr bool get_button_a() const { return btn_a; }
  constexpr bool get_button_b() const { return btn_b; }
  constexpr bool get_button_x() const { return btn_x; }
  constexpr bool get_button_y() const { return btn_y; }

  constexpr bool get_button_l1() const { return btn_l; }
  constexpr bool get_button_r1() const { return btn_r; }
  constexpr bool get_button_l2() const { return btn_zl; }
  constexpr bool get_button_r2() const { return btn_zr; }
  constexpr bool get_button_l3() const { return btn_thumb_l; }
  constexpr bool get_button_r3() const { return btn_thumb_r; }

  constexpr bool get_button_l() const { return btn_l; }
  constexpr bool get_button_r() const { return btn_r; }
  constexpr bool get_button_zl() const { return btn_zl; }
  constexpr bool get_button_zr() const { return btn_zr; }
  constexpr bool get_button_thumb_l() const { return btn_thumb_l; }
  constexpr bool get_button_thumb_r() const { return btn_thumb_r; }

  constexpr bool get_button_home() const { return btn_home; }
  constexpr bool get_button_capture() const { return btn_capture; }
  constexpr bool get_button_plus() const { return btn_plus; }
  constexpr bool get_button_minus() const { return btn_minus; }

  /// Set the buttons
  /// @param buttons The struct containing the button values
  template <SwitchProButtonStruct T> constexpr void set_buttons(const T &buttons) {
    this->set_button_a(buttons.a);
    this->set_button_b(buttons.b);
    this->set_button_x(buttons.x);
    this->set_button_y(buttons.y);
    this->set_button_l(buttons.l);
    this->set_button_r(buttons.r);
    this->set_button_zl(buttons.zl);
    this->set_button_zr(buttons.zr);
    this->set_button_thumb_l(buttons.thumb_l);
    this->set_button_thumb_r(buttons.thumb_r);
    this->set_button_home(buttons.home);
    this->set_button_capture(buttons.capture);
    this->set_button_plus(buttons.plus);
    this->set_button_minus(buttons.minus);
    this->set_dpad(buttons.dpad_up, buttons.dpad_down, buttons.dpad_left, buttons.dpad_right);
  }

  /// Get the button values
  /// @param t The struct to fill with the button values
  template <SwitchProButtonStruct T> constexpr void get_buttons(T &t) const {
    t.a = this->get_button_a();
    t.b = this->get_button_b();
    t.x = this->get_button_x();
    t.y = this->get_button_y();
    t.l = this->get_button_l();
    t.r = this->get_button_r();
    t.zl = this->get_button_zl();
    t.zr = this->get_button_zr();
    t.thumb_l = this->get_button_thumb_l();
    t.thumb_r = this->get_button_thumb_r();
    t.home = this->get_button_home();
    t.capture = this->get_button_capture();
    t.plus = this->get_button_plus();
    t.minus = this->get_button_minus();
    // NOTE: do this in case the members are bitfields
    bool up, down, left, right;
    this->get_dpad(up, down, left, right);
    t.dpad_up = up;
    t.dpad_down = down;
    t.dpad_left = left;
    t.dpad_right = right;
  }

  /// Get the dpad value
  /// @param up up dpad value
  /// @param down down dpad value
  /// @param left left dpad value
  /// @param right right dpad value
  constexpr void get_dpad(bool &up, bool &down, bool &left, bool &right) const {
    up = dpad_up;
    down = dpad_down;
    left = dpad_left;
    right = dpad_right;
  }

  /// Set the button value
  /// \param button_index The button for which you want to set the value.
  ///        Should be between 1 and 24, inclusive.
  /// \param value The true/false value you want to se the button to.
  constexpr void set_button(int button_index, bool value) {
    if (button_index < 1 || button_index > 24) {
      return;
    }
    // set the appropriate bit in the raw_input_report starting at byte 2 (0-indexed)
    // and bit 0 (0-indexed)
    int byte_index = 2 + (button_index - 1) / 8;
    int bit_index = (button_index - 1) % 8;
    uint8_t current_byte = raw_input_report[byte_index];
    if (value) {
      raw_input_report[byte_index] = current_byte | (1 << bit_index);
    } else {
      raw_input_report[byte_index] = current_byte & ~(1 << bit_index);
    }
  }

  /// Get the button value
  /// \param button_index The button for which you want to get the value.
  ///        Should be between 1 and 24, inclusive.
  /// \return The true/false value of the button.
  constexpr bool get_button(int button_index) const {
    if (button_index < 1 || button_index > 24) {
      return false;
    }
    // get the appropriate bit in the raw_input_report starting at byte 2 (0-indexed)
    // and bit 0 (0-indexed)
    int byte_index = 2 + (button_index - 1) / 8;
    int bit_index = (button_index - 1) % 8;
    return (raw_input_report[byte_index] & (1 << bit_index)) != 0;
  }

  /// Set the trigger axis value
  /// \param index The index of the trigger axis to set. Should be between 0 and 1, inclusive.
  /// \param value The value to set the trigger axis to.
  /// \note The value should be in the range [0, 1].
  constexpr void set_trigger_axis(size_t index, float value) {
    set_trigger_axis(index, value > 0.5);
  }

  /// Set the trigger pressed value
  /// \param index The index of the trigger axis to set. Should be between 0 and 1, inclusive.
  /// \param pressed Whether the trigger is pressed or not
  constexpr void set_trigger_axis(size_t index, bool pressed) {
    if (index == 0) {
      // brake / zl
      btn_zl = pressed;
    } else if (index == 1) {
      // accelerator / zr
      btn_zr = pressed;
    }
  }

  /// Get the input report as a vector of bytes
  /// \return The input report as a vector of bytes.
  /// \note The report id is not included in the returned vector.
  constexpr auto get_report() const {
    // the first byte is the id, which we don't want
    size_t offset = 1;
    auto report_data = this->data() + offset;
    auto report_size = num_data_bytes;
    return std::vector<uint8_t>(report_data, report_data + report_size);
  }

  /// Set the output report data from a vector of bytes
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    // the first byte is the id, which we don't want
    size_t offset = 1;
    // copy the data into our data array
    std::copy(data.begin(), data.end(), this->data() + offset);
  }

  /// Get the report descriptor as a hid::rdf::descriptor
  /// \return The report descriptor as a hid::rdf::descriptor.
  /// \note This is an incomplete descriptor, you will need to add it to a
  ///      collection::application descriptor to create a complete report descriptor.
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    // clang-format off
      return descriptor(
                        conditional_report_id<REPORT_ID>(),
                        usage_page<generic_desktop>(),
                        usage_page<button>(),
                        usage_limits(button(1), button(0x0A)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(0x0A),
                        unit::base<unit::code::NONE, 0>::exponent_item(),
                        unit::base<unit::code::NONE, 0>::unit_item(),
                        input::absolute_variable(),
                        usage_page<button>(),
                        usage_limits(button(0x0B), button(0x0E)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(4),
                        input::absolute_variable(),
                        report_size(1),
                        report_count(2),
                        input::absolute_constant(),
                        hid::rdf::short_item<4>(local::tag::USAGE, 0x010001),
                        collection::physical(
                                             hid::rdf::short_item<4>(local::tag::USAGE, 0x010030),
                                             hid::rdf::short_item<4>(local::tag::USAGE, 0x010031),
                                             hid::rdf::short_item<4>(local::tag::USAGE, 0x010032),
                                             hid::rdf::short_item<4>(local::tag::USAGE, 0x010035),
                                             logical_limits<1, 4>(0, 65535),
                                             report_size(16),
                                             report_count(4),
                                             input::absolute_variable()
                                             ),

                        hid::rdf::short_item<4>(local::tag::USAGE, 0x010039),
                        logical_limits<1, 1>(0, 7),
                        physical_limits<1, 2>(0, 315),
                        unit::unit<1>(unit::code::DEGREE),
                        report_size(4),
                        report_count(1),
                        input::absolute_variable(),
                        usage_page<button>(),
                        usage_limits(button(0x0F), button(0x12)),
                        logical_limits<1, 1>(0, 1),
                        report_size(1),
                        report_count(4),
                        input::absolute_variable(),
                        report_size(8),
                        report_count(52),
                        input::absolute_constant()
                        );
    // clang-format on
  }

  friend fmt::formatter<SwitchProGamepadInputReport<REPORT_ID>>;
}; // class SwitchProGamepadInputReport

/// HID Switch Pro Input Vendor Report
/// This class implements a HID Switch Pro Input Vendor Report. It supports 63
/// bytes of data, which can be set and retrieved. It also provides a method to
/// get the report descriptor.
template <uint8_t REPORT_ID, uint8_t VENDOR_USAGE>
struct SwitchProInputVendorReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
  uint8_t data[63] = {0};
  constexpr SwitchProInputVendorReport() = default;

  /// Set the data for the input report
  /// \param data The data to set the input report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    std::copy(data.begin(), data.end(), this->data);
  }

  /// Get the data for the input report
  /// \return The data for the input report
  constexpr auto get_data() const { return std::vector<uint8_t>(data, data + sizeof(data)); }

  /// Get the descriptor for the input report
  /// \return The descriptor for the input report
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<1>(local::tag::USAGE, VENDOR_USAGE), report_size(8),
                      report_count(63), input::absolute_constant());
  }
}; // struct SwitchProInputVendorReport

/// HID Switch Pro Output Vendor Report
/// This class implements a HID Switch Pro Output Vendor Report. It supports 63
/// bytes of data, which can be set and retrieved. It also provides a method to
/// get the report descriptor.
template <uint8_t REPORT_ID, uint8_t VENDOR_USAGE>
struct SwitchProOutputVendorReport
    : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
  uint8_t data[63] = {0};
  constexpr SwitchProOutputVendorReport() = default;

  /// Set the data for the output report
  /// \param data The data to set the output report to.
  constexpr void set_data(const std::vector<uint8_t> &data) {
    std::copy(data.begin(), data.end(), this->data);
  }

  /// Get the data for the output report
  /// \return The data for the output report
  constexpr auto get_data() const { return std::vector<uint8_t>(data, data + sizeof(data)); }

  /// Get the descriptor for the output report
  /// \return The descriptor for the output report
  static constexpr auto get_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    return descriptor(conditional_report_id<REPORT_ID>(),
                      hid::rdf::short_item<1>(local::tag::USAGE, VENDOR_USAGE), report_size(8),
                      report_count(63), output::absolute_constant(main::VOLATILE));
  }
}; // struct SwitchProOutputVendorReport

/// Get the descriptor for the Switch Pro controller
/// \return The descriptor for the Switch Pro controller
/// \note This is the complete descriptor for the Switch Pro controller, and
///       matches byte-for-byte the descriptor provided by the actual Switch
///       Pro controller.
[[maybe_unused]] static constexpr auto switch_pro_descriptor() {
  using namespace hid::page;
  using namespace hid::rdf;

  // clang-format off
    return descriptor(
                      usage_page<generic_desktop>(),
                      logical_min<1>(0),
                      usage(generic_desktop::JOYSTICK),
                      collection::application(
                                              SwitchProGamepadInputReport<>::get_descriptor(),
                                              usage_page<switch_vendor>(),
                                              SwitchProInputVendorReport<33, 0x01>::get_descriptor(),
                                              SwitchProInputVendorReport<0x81, 0x02>::get_descriptor(),
                                              SwitchProOutputVendorReport<0x01, 0x03>::get_descriptor(),
                                              SwitchProOutputVendorReport<0x10, 0x04>::get_descriptor(),
                                              SwitchProOutputVendorReport<0x80, 0x05>::get_descriptor(),
                                              SwitchProOutputVendorReport<0x82, 0x06>::get_descriptor()
                                              )
                      );
  // clang-format on
} // switch_pro_descriptor

} // namespace espp

#include "hid-rp-switch-pro-formatters.hpp"

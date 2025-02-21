#pragma once

#include <algorithm>

#include "hid-rp.hpp"

namespace hid::page {
  enum class switch_vendor : std::uint16_t;
  template <> struct info<switch_vendor> {
    constexpr static page_id_t page_id = 0xFF00;
    constexpr static usage_id_t max_usage_id = 0xFFFF;
    constexpr static const char *name = "Switch Vendor Page";
  };
} // namespace hid::page

namespace espp {

  /// HID Switch Pro Gamepad Input Report
  ///
  /// This class implements a HID Switch Pro Gamepad Input Report. It supports 15
  /// buttons, a d-pad, 4 joystick axes and two trigger buttons. It supports
  /// setting the buttons, d-pad, joysticks, and triggers, as well as serializing
  /// the input report and getting the report descriptor.
  ///
  /// \section hid_rp_ex1 HID-RP Example
  /// \snippet hid_rp_example.cpp hid rp example
  template <uint8_t REPORT_ID = 0x30>
  class SwitchProGamepadInputReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
  protected:
    using JOYSTICK_TYPE = std::int16_t;
    static constexpr JOYSTICK_TYPE joystick_min = 0;
    static constexpr JOYSTICK_TYPE joystick_max = 4096;
    static constexpr JOYSTICK_TYPE joystick_center = (joystick_min + joystick_max) / 2;
    static constexpr size_t joystick_value_range = joystick_max - joystick_min;
    static constexpr JOYSTICK_TYPE joystick_range = joystick_value_range / 2;

    union {
      // See
      // https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering/blob/master/bluetooth_hid_notes.md#standard-input-report-format
      // for more information.
      struct {
        // Byte 0: Timer which increments very fast.
        //         Can be used to estimate excess latency
        uint8_t timer;
        // Byte 1:
        // - Battery level.
        //     8=full, 6=medium, 4=low, 2=critical, 0=empty. Bit 0 is charging status.
        // - Connection info.
        //     Bit 0 is USB powered, bits 1-2 are connection info.
        uint8_t battery_level : 4;
        uint8_t connection_info : 4;
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
        //
        // decode:
        //     uint8_t *data = packet + (left ? 5 : 8);
        //     uint16_t stick_horizontal = data[0] | ((data[1] & 0xF) << 8);
        //     uint16_t stick_vertical = (data[1] >> 4) | (data[2] << 4);
        // encode:
        //     uint8_t *data = packet + (left ? 5 : 8);
        //     data[0] = stick_horizontal & 0xFF;
        //     data[1] = (stick_horizontal >> 8) | ((stick_vertical & 0xF) << 4);
        //     data[2] = stick_vertical >> 4;
        uint8_t analog[6];
        // Byte 11: Vibrator input report.
        //          Decides if next vibration pattern should be sent.
        uint8_t vibrator_input_report;
      } __attribute__((packed));
      uint8_t raw_report[12];
    };
    // for report IDs 0x30, 0x31, 0x32, 0x33, this is 6-axis data. 3 frames of 2
    // groups of 2 int16LE each. group is ACC followed by GYRO.
    //
    // for report ID 0x21, this is subcommand reply data. max len 35. In this
    // report, there is additional 2 bytes of ACK/reply between raw_report and
    // imuData.
    //
    // for report ID 0x23, this is NFC/IR MCU FW update input report (max len 37)
    //
    // for report id 0x31, there are aditional 313 bytes of NFC/IR data input
    // after this.
    uint8_t imuData[36];
    // make this report the same length as the other switch pro input/output
    // reports.
    uint8_t padBytes[15];

    static constexpr size_t num_data_bytes = 63;

  public:
    /// Construct a new Gamepad Input Report object
    constexpr SwitchProGamepadInputReport() = default;

    /// Reset the gamepad inputs
    constexpr void reset() {
      std::fill(raw_report, raw_report + sizeof(raw_report), 0);
      set_left_joystick(0, 0);
      set_right_joystick(0, 0);
      set_battery_level(75);
      set_battery_charging(false);
      set_usb_powered(false);
    }

    constexpr uint8_t report_id() { return 0x30; }

    constexpr void set_timer(uint64_t time_us) {
      // this is the timer which is supposed to increment very quickly and can be
      // used to estimate excess latency
      uint32_t time_ms = time_us / 1000 / 4;
      timer = static_cast<uint8_t>(time_ms & 0xff);
    }

    /// Set the left joystick X and Y axis values
    /// @param lx left joystick x axis value, in the range [-1, 1]
    /// @param ly left joystick y axis value, in the range [-1, 1]
    constexpr void set_left_joystick(float lx, float ly) {
      // decode:
      //     uint8_t *data = packet + (left ? 5 : 8);
      //     uint16_t stick_horizontal = data[0] | ((data[1] & 0xF) << 8);
      //     uint16_t stick_vertical = (data[1] >> 4) | (data[2] << 4);
      // encode:
      //     uint8_t *data = packet + (left ? 5 : 8);
      //     data[0] = stick_horizontal & 0xFF;
      //     data[1] = (stick_horizontal >> 8) | ((stick_vertical & 0xF) << 4);
      //     data[2] = stick_vertical >> 4;

      JOYSTICK_TYPE x = std::clamp(static_cast<JOYSTICK_TYPE>(lx * joystick_range + joystick_center),
                                   joystick_min, joystick_max);
      JOYSTICK_TYPE y = std::clamp(static_cast<JOYSTICK_TYPE>(ly * joystick_range + joystick_center),
                                   joystick_min, joystick_max);

      analog[0] = x & 0xFF;
      analog[1] = (x >> 8) | ((y & 0xF) << 4);
      analog[2] = y >> 4;
    }

    /// Set the right joystick X and Y axis values
    /// @param rx right joystick x axis value, in the range [-1, 1]
    /// @param ry right joystick y axis value, in the range [-1, 1]
    constexpr void set_right_joystick(float rx, float ry) {
      // decode:
      //     uint8_t *data = packet + (left ? 5 : 8);
      //     uint16_t stick_horizontal = data[0] | ((data[1] & 0xF) << 8);
      //     uint16_t stick_vertical = (data[1] >> 4) | (data[2] << 4);
      // encode:
      //     uint8_t *data = packet + (left ? 5 : 8);
      //     data[0] = stick_horizontal & 0xFF;
      //     data[1] = (stick_horizontal >> 8) | ((stick_vertical & 0xF) << 4);
      //     data[2] = stick_vertical >> 4;

      JOYSTICK_TYPE x = std::clamp(static_cast<JOYSTICK_TYPE>(rx * joystick_range + joystick_center),
                                   joystick_min, joystick_max);
      JOYSTICK_TYPE y = std::clamp(static_cast<JOYSTICK_TYPE>(ry * joystick_range + joystick_center),
                                   joystick_min, joystick_max);

      analog[3] = x & 0xFF;
      analog[4] = (x >> 8) | ((y & 0xF) << 4);
      analog[5] = y >> 4;
    }

    /// Set the brake trigger value
    /// @param pressed Whether the brake trigger is pressed or not
    constexpr void set_brake(bool pressed) { set_trigger_axis(0, pressed); }

    /// Set the brake trigger value
    /// @param value The value to set the brake trigger to.
    ///        Should be in the range [0, 1].
    constexpr void set_brake(float value){ set_trigger_axis(0, value); }

    /// Set the accelerator trigger value
    /// @param pressed Whether the accelerator trigger is pressed or not
    constexpr void set_accelerator(bool pressed) { set_trigger_axis(1, pressed); }

    /// Set the accelerator trigger value
    /// @param value The value to set the accelerator trigger to.
    ///        Should be in the range [0, 1].
    constexpr void set_accelerator(float value){ set_trigger_axis(1, value); }

    /// Set the battery level
    /// @param level battery level, in the range [0, 100]
    constexpr void set_battery_level(float level) {
      // battery level is only 3 bits, so we need to convert from [0, 100] to [0,
      // 8] and then shift it by 1 to make it fit in the upper 3 bits
      battery_level = (int)(std::clamp(level / 100.0f * 8, 0.0f, 8.0f)) << 1;
    }

    constexpr void set_battery_charging(bool charging) {
      // charging is the least significant bit of the battery level
      // so we just need to set the least significant bit of the battery level
      battery_level = (battery_level & 0xE) | (charging ? 1 : 0);
    }

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

    constexpr void set_button_home(bool pressed) { btn_home = pressed; }
    constexpr void set_button_capture(bool pressed) { btn_capture = pressed; }
    constexpr void set_button_plus(bool pressed) { btn_plus = pressed; }
    constexpr void set_button_minus(bool pressed) { btn_minus = pressed; }

    /// Set the button value
    /// \param button_index The button for which you want to set the value.
    ///        Should be between 1 and 24, inclusive.
    /// \param value The true/false value you want to se the button to.
    constexpr void set_button(int button_index, bool value) {
      if (button_index < 1 || button_index > 24) {
        return;
      }
      // set the appropriate bit in the raw_report starting at byte 2 (0-indexed)
      // and bit 0 (0-indexed)
      int byte_index = 2 + (button_index - 1) / 8;
      int bit_index = (button_index - 1) % 8;
      uint8_t current_byte = raw_report[byte_index];
      if (value) {
        raw_report[byte_index] = current_byte | (1 << bit_index);
      } else {
        raw_report[byte_index] = current_byte & ~(1 << bit_index);
      }
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
    constexpr auto get_report() {
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
    static constexpr auto get_descriptor() {
      using namespace hid::page;
      using namespace hid::rdf;

      // clang-format off
      return descriptor(
                        conditional_report_id<0x30>(),
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
  template<uint8_t REPORT_ID, uint8_t VENDOR_USAGE>
  struct SwitchProInputVendorReport : public hid::report::base<hid::report::type::INPUT, REPORT_ID> {
    uint8_t data[63];
    constexpr SwitchProInputVendorReport() = default;

    /// Set the data for the input report
    /// \param data The data to set the input report to.
    constexpr void set_data(const std::vector<uint8_t> &data) {
      std::copy(data.begin(), data.end(), this->data);
    }

    /// Get the data for the input report
    /// \return The data for the input report
    constexpr auto get_data() {
      return std::vector<uint8_t>(data, data + sizeof(data));
    }

    /// Get the descriptor for the input report
    /// \return The descriptor for the input report
    static constexpr auto get_descriptor() {
      using namespace hid::page;
      using namespace hid::rdf;

      return descriptor(
                        conditional_report_id<REPORT_ID>(),
                        hid::rdf::short_item<1>(local::tag::USAGE, VENDOR_USAGE),
                        report_size(8),
                        report_count(63),
                        input::absolute_constant()
                        );
    }
  }; // struct SwitchProInputVendorReport

  /// HID Switch Pro Output Vendor Report
  /// This class implements a HID Switch Pro Output Vendor Report. It supports 63
  /// bytes of data, which can be set and retrieved. It also provides a method to
  /// get the report descriptor.
  template<uint8_t REPORT_ID, uint8_t VENDOR_USAGE>
  struct SwitchProOutputVendorReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
    uint8_t data[63];
    constexpr SwitchProOutputVendorReport() = default;

    /// Set the data for the output report
    /// \param data The data to set the output report to.
    constexpr void set_data(const std::vector<uint8_t> &data) {
      std::copy(data.begin(), data.end(), this->data);
    }

    /// Get the data for the output report
    /// \return The data for the output report
    constexpr auto get_data() {
      return std::vector<uint8_t>(data, data + sizeof(data));
    }

    /// Get the descriptor for the output report
    /// \return The descriptor for the output report
    static constexpr auto get_descriptor() {
      using namespace hid::page;
      using namespace hid::rdf;

      return descriptor(
                        conditional_report_id<REPORT_ID>(),
                        hid::rdf::short_item<1>(local::tag::USAGE, VENDOR_USAGE),
                        report_size(8),
                        report_count(63),
                        output::absolute_constant(main::VOLATILE)
                        );
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

#pragma once

#include <algorithm>

#include "hid-rp-gamepad.hpp"

namespace espp {

  /// HID Xbox Gamepad Input Report
  ///
  /// This class implements a HID Xbox Gamepad Report. It supports 15 buttons, a
  /// d-pad, 4 joystick axes, 2 trigger axes as well as consumer record button.
  ///
  /// \section hid_rp_ex1 HID-RP Example
  /// \snippet hid_rp_example.cpp hid rp example
  template <uint8_t REPORT_ID = 1>
  class XboxGamepadInputReport : public espp::GamepadInputReport<15, std::uint16_t, std::uint16_t, 0, 65535, 0, 1023, REPORT_ID> {
  };

  /// HID Xbox Rumble Output Report
  ///
  /// This class implements a HID Rumble Output Report providing rumble effect on
  /// the left and right motors, as well as serializing the output report and
  /// getting the report descriptor.
  template <uint8_t REPORT_ID = 3>
  class XboxRumbleOutputReport : public hid::report::base<hid::report::type::OUTPUT, REPORT_ID> {
  protected:
    struct {
      std::uint8_t enabled : 4; // which motors are enabled
      std::uint8_t : 4;
    };

    std::array<std::uint8_t, 4> magnitude{0}; // [0, 100] magnitude of the effect on each motor
    std::uint8_t duration{0};                 // [0, 255] duration of the effect
    std::uint8_t start_delay{0};              // [0, 255] delay before the effect starts
    std::uint8_t loop_count{0};               // [0, 255] number of times to loop the effect

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
    constexpr void set_enabled(std::uint8_t new_enabled) { enabled = new_enabled; }

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
  }; // XboxBatteryInputReport

  /// Get the report descriptor for the Xbox Gamepad
  /// \return The report descriptor for the Xbox Gamepad
  /// \note This is the complete report descriptor for the Xbox Gamepad.
  [[maybe_unused]] static constexpr auto xbox_descriptor() {
    using namespace hid::page;
    using namespace hid::rdf;

    auto gamepad_descriptor = XboxGamepadInputReport<>::get_descriptor();
    auto rumble_descriptor = XboxRumbleOutputReport<>::get_descriptor();
    auto battery_descriptor = XboxBatteryInputReport<>::get_descriptor();

    return descriptor(
                      usage_page<generic_desktop>(),
                      usage(generic_desktop::GAMEPAD),
                      collection::application(
                                              gamepad_descriptor,
                                              rumble_descriptor,
                                              battery_descriptor
                                              )
                      );
  } // xbox_descriptor

} // namespace espp


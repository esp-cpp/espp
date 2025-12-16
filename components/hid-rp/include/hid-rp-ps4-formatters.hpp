#pragma once

#include "format.hpp"

template <uint8_t REPORT_ID>
struct fmt::formatter<espp::PS4DualShock4GamepadInputReport<REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::PS4DualShock4GamepadInputReport<REPORT_ID> &report,
              FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "PS4DualShock4GamepadInputReport {{");
    fmt::format_to(out, "left_stick: ({}, {}), ", report.get_left_joystick_x(),
                   report.get_left_joystick_y());
    fmt::format_to(out, "right_stick: ({}, {}), ", report.get_right_joystick_x(),
                   report.get_right_joystick_y());
    fmt::format_to(out, "triggers: (L2={}, R2={}), ", report.get_l2_trigger(),
                   report.get_r2_trigger());
    fmt::format_to(out, "hat: {}, ", static_cast<int>(report.get_hat()));
    fmt::format_to(out, "buttons: [");
    fmt::format_to(out, "Square={}, Cross={}, Circle={}, Triangle={}, ", report.get_button_square(),
                   report.get_button_cross(), report.get_button_circle(),
                   report.get_button_triangle());
    fmt::format_to(out, "L1={}, R1={}, L2={}, R2={}, ", report.get_button_l1(),
                   report.get_button_r1(), report.get_button_l2(), report.get_button_r2());
    fmt::format_to(out, "Share={}, Options={}, L3={}, R3={}, ", report.get_button_share(),
                   report.get_button_options(), report.get_button_l3(), report.get_button_r3());
    fmt::format_to(out, "PS={}, Touchpad={}], ", report.get_button_home(),
                   report.get_button_touchpad());
    fmt::format_to(out, "battery: {}%, charging: {}, ", report.get_battery_level(),
                   report.get_battery_charging());
    auto gyro = report.get_gyroscope();
    auto accel = report.get_accelerometer();
    fmt::format_to(out, "gyro: ({}, {}, {}), ", gyro.X, gyro.Y, gyro.Z);
    fmt::format_to(out, "accel: ({}, {}, {}), ", accel.X, accel.Y, accel.Z);
    fmt::format_to(out, "timestamp: {}, counter: {}", report.get_timestamp(), report.get_counter());
    return fmt::format_to(out, "}}");
  }
};

template <uint8_t REPORT_ID> struct fmt::formatter<espp::PS4DualShock4OutputReport<REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::PS4DualShock4OutputReport<REPORT_ID> &report, FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "PS4DualShock4OutputReport {{");
    fmt::format_to(out, "data: [{::#02X}]", report.data);
    return fmt::format_to(out, "}}");
  }
};

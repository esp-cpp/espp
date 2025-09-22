#pragma once

#include "format.hpp"

template <uint8_t REPORT_ID> struct fmt::formatter<espp::XboxGamepadInputReport<REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::XboxGamepadInputReport<REPORT_ID> &report, FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "XboxGamepadInputReport {{");
    fmt::format_to(out, "joystick_axes: [");
    for (size_t i = 0; i < 4; i++) {
      fmt::format_to(out, "{}", report.joystick_axes[i]);
      if (i < 3) {
        fmt::format_to(out, ", ");
      }
    }
    fmt::format_to(out, "], trigger_axes: [");
    for (size_t i = 0; i < 2; i++) {
      fmt::format_to(out, "{}", report.trigger_axes[i]);
      if (i < 1) {
        fmt::format_to(out, ", ");
      }
    }
    fmt::format_to(out, "], hat_switch: {}, buttons: [", report.hat_switch);
    std::bitset<report.button_count> buttons;
    for (size_t i = 1; i <= report.button_count; i++) {
      buttons.set(i - 1, report.buttons.test(hid::page::button(i)));
    }
    fmt::format_to(out, "{}", buttons);
    fmt::format_to(out, "], consumer_record: {}", (bool)report.consumer_record);
    return fmt::format_to(out, "}}");
  }
};

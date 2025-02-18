#pragma once

#include "format.hpp"

template <size_t BUTTON_COUNT, typename JOYSTICK_TYPE, typename TRIGGER_TYPE,
          JOYSTICK_TYPE JOYSTICK_MIN, JOYSTICK_TYPE JOYSTICK_MAX, TRIGGER_TYPE TRIGGER_MIN,
          TRIGGER_TYPE TRIGGER_MAX, uint8_t REPORT_ID>
struct fmt::formatter<
    espp::GamepadInputReport<BUTTON_COUNT, JOYSTICK_TYPE, TRIGGER_TYPE, JOYSTICK_MIN, JOYSTICK_MAX,
                             TRIGGER_MIN, TRIGGER_MAX, REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto
  format(const espp::GamepadInputReport<BUTTON_COUNT, JOYSTICK_TYPE, TRIGGER_TYPE, JOYSTICK_MIN,
                                        JOYSTICK_MAX, TRIGGER_MIN, TRIGGER_MAX, REPORT_ID> &report,
         FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "GamepadInputReport<{}> {{", BUTTON_COUNT);
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
    std::bitset<BUTTON_COUNT> buttons;
    for (size_t i = 1; i <= BUTTON_COUNT; i++) {
      buttons.set(i - 1, report.buttons.test(hid::page::button(i)));
    }
    fmt::format_to(out, "{}", buttons);
    fmt::format_to(out, "], consumer_record: {}", (bool)report.consumer_record);
    return fmt::format_to(out, "}}");
  }
};

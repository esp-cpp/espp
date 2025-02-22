#pragma once

#include "format.hpp"

template <uint8_t REPORT_ID> struct fmt::formatter<espp::SwitchProGamepadInputReport<REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::SwitchProGamepadInputReport<REPORT_ID> &report,
              FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "SwitchProGamepadInputReport {{");
    fmt::format_to(out, "analog: {::#04x}", report.analog);
    uint32_t buttons =
        report.raw_report[4] << 16 | report.raw_report[3] << 8 | report.raw_report[2];
    fmt::format_to(out, "], buttons: [{:#026b}]", buttons);
    return fmt::format_to(out, "}}");
  }
};

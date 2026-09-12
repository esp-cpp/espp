#pragma once

#include <bitset>
#include <cstddef>
#include <cstdint>

#include "format.hpp"
#include "hid-rp-3dconnexion.hpp"

template <std::int16_t LOGICAL_MIN, std::int16_t LOGICAL_MAX, uint8_t REPORT_ID>
struct fmt::formatter<espp::SpaceMouseTranslationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto
  format(const espp::SpaceMouseTranslationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID> &report,
         FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "SpaceMouseTranslationInputReport {{x: {}, y: {}, z: {}}}",
                          report.axes[0], report.axes[1], report.axes[2]);
  }
};

template <std::int16_t LOGICAL_MIN, std::int16_t LOGICAL_MAX, uint8_t REPORT_ID>
struct fmt::formatter<espp::SpaceMouseRotationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto
  format(const espp::SpaceMouseRotationInputReport<LOGICAL_MIN, LOGICAL_MAX, REPORT_ID> &report,
         FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "SpaceMouseRotationInputReport {{rx: {}, ry: {}, rz: {}}}",
                          report.axes[0], report.axes[1], report.axes[2]);
  }
};

template <std::size_t BUTTON_COUNT, uint8_t REPORT_ID>
struct fmt::formatter<espp::SpaceMouseButtonsInputReport<BUTTON_COUNT, REPORT_ID>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const espp::SpaceMouseButtonsInputReport<BUTTON_COUNT, REPORT_ID> &report,
              FormatContext &ctx) const {
    auto out = ctx.out();
    fmt::format_to(out, "SpaceMouseButtonsInputReport<{}> {{buttons: [", BUTTON_COUNT);
    std::bitset<BUTTON_COUNT> buttons;
    for (size_t i = 1; i <= BUTTON_COUNT; i++) {
      buttons.set(i - 1, report.buttons.test(hid::page::button(i)));
    }
    fmt::format_to(out, "{}", buttons);
    return fmt::format_to(out, "]}}");
  }
};

#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// Rgb
template <> struct fmt::formatter<espp::Rgb> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Rgb const &rgb, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", rgb.r, rgb.g, rgb.b);
  }
};

// for allowing easy serialization/printing of the
// Rgb
template <> struct fmt::formatter<espp::Hsv> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Hsv const &hsv, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", hsv.h, hsv.s, hsv.v);
  }
};

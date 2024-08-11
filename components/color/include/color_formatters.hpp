#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// Rgb
template <> struct fmt::formatter<espp::Rgb> {
  // Presentation format: 'f' - floating [0,1] (default), 'd' - integer [0,255], 'x' - hex integer.
  char presentation = 'f';

  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    // Parse the presentation format and store it in the formatter:
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'f' || *it == 'd' || *it == 'x'))
      presentation = *it++;

    // TODO: Check if reached the end of the range:
    // if (it != end && *it != '}') throw format_error("invalid format");

    // Return an iterator past the end of the parsed range:
    return it;
  }

  template <typename FormatContext> auto format(espp::Rgb const &rgb, FormatContext &ctx) const {
    switch (presentation) {
    case 'f':
      return fmt::format_to(ctx.out(), "({}, {}, {})", rgb.r, rgb.g, rgb.b);
    case 'd':
      return fmt::format_to(ctx.out(), "({}, {}, {})", static_cast<int>(rgb.r * 255),
                            static_cast<int>(rgb.g * 255), static_cast<int>(rgb.b * 255));
    case 'x':
      return fmt::format_to(ctx.out(), "{:#08X}", rgb.hex());
    default:
      // shouldn't get here!
      return fmt::format_to(ctx.out(), "({}, {}, {})", rgb.r, rgb.g, rgb.b);
    }
  }
};

// for allowing easy serialization/printing of the
// Hsv
template <> struct fmt::formatter<espp::Hsv> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Hsv const &hsv, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({}, {}, {})", hsv.h, hsv.s, hsv.v);
  }
};

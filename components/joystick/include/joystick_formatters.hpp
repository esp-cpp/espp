#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// Trigger class
template <> struct fmt::formatter<espp::Joystick> {
  // Presentation format: 'v' - value, 'r' - raw, 'b' - both.
  char presentation = 'v';

  // Parses format specifications of the form ['v' | 'r' | 'b'].
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    // Parse the presentation format and store it in the formatter:
    auto it = ctx.begin(), end = ctx.end();
    if (it != end && (*it == 'v' || *it == 'r' || *it == 'b'))
      presentation = *it++;

    // TODO: Check if reached the end of the range:
    // if (it != end && *it != '}') throw format_error("invalid format");

    // Return an iterator past the end of the parsed range:
    return it;
  }

  template <typename FormatContext> auto format(espp::Joystick const &j, FormatContext &ctx) const {
    switch (presentation) {
    case 'v':
      return fmt::format_to(ctx.out(), "{}", j.position_);
    case 'r':
      return fmt::format_to(ctx.out(), "{}", j.raw_);
    case 'b':
      return fmt::format_to(ctx.out(), "({} -> {})", j.raw_, j.position_);
    default:
      // shouldn't get here!
      return fmt::format_to(ctx.out(), "{}", j.position_);
    }
  }
};

#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::Vector2d<type>
template <typename Value> struct fmt::formatter<espp::Vector2d<Value>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Vector2d<Value> const &v, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "({},{})", v.x(), v.y());
  }
};

#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::SimpleLowpassFilter
template <> struct fmt::formatter<espp::SimpleLowpassFilter> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::SimpleLowpassFilter const &f, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "SimpleLowpassFilter - {}", f.time_constant_);
  }
};

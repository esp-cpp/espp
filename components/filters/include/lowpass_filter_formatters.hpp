#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::LowpassFilter
template <> struct fmt::formatter<espp::LowpassFilter> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::LowpassFilter const &f, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "Lowpass - {}", f.coeffs_);
  }
};

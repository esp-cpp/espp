#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::ButterworthFilter
template <size_t ORDER, class Impl> struct fmt::formatter<espp::ButterworthFilter<ORDER, Impl>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::ButterworthFilter<ORDER, Impl> const &f, FormatContext &ctx) const {
    auto &&out = ctx.out();
    fmt::format_to(out, "Butterworth - [");
    if constexpr (ORDER > 0) {
      fmt::format_to(out, "[{}]", f.sections_[0]);
    }
    for (int i = 1; i < (ORDER + 1) / 2; i++) {
      fmt::format_to(out, ", [{}]", f.sections_[i]);
    }
    return fmt::format_to(out, "]");
  }
};

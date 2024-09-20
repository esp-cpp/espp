#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::SosFilter
template <size_t N, class SectionImpl> struct fmt::formatter<espp::SosFilter<N, SectionImpl>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::SosFilter<N, SectionImpl> const &f, FormatContext &ctx) const {
    auto &&out = ctx.out();
    format_to(out, "SoS - [");
    if constexpr (N > 0) {
      format_to(out, "[{}]", f.sections_[0]);
    }
    for (int i = 1; i < N; i++) {
      format_to(out, ", [{}]", f.sections_[i]);
    }
    return fmt::format_to(out, "]");
  }
};

#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::BiquadFilterDf1
template <> struct fmt::formatter<espp::BiquadFilterDf1> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::BiquadFilterDf1 const &bqf, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "DF1 - B: {}, A: {}", bqf.b_, bqf.a_);
  }
};

// for allowing easy serialization/printing of the
// espp::BiquadFilterDf2
template <> struct fmt::formatter<espp::BiquadFilterDf2> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::BiquadFilterDf2 const &bqf, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "DF2 - B: {}, A: {}", bqf.b_, bqf.a_);
  }
};

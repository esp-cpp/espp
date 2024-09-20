#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::TransferFunction
template <size_t N> struct fmt::formatter<espp::TransferFunction<N>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::TransferFunction<N> const &tf, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "TransferFunction({}) - B: {}, A: {}", N, tf.b, tf.a);
  }
};

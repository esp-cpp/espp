#pragma once

#include "format.hpp"
#include <array>

namespace espp {
/**
 * @brief Transfer function representing the A and B coefficients.
 */
template <size_t N> struct TransferFunction {
  TransferFunction() = default;

  TransferFunction(const std::array<float, N> &b, const std::array<float, N> &a) : b(b), a(a) {}

  std::array<float, N> b = {}; /**< B coefficients. */
  std::array<float, N> a = {}; /**< A coefficients. */
};
} // namespace espp

// for allowing easy serialization/printing of the
// espp::TransferFunction
template <size_t N> struct fmt::formatter<espp::TransferFunction<N>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::TransferFunction<N> const &tf, FormatContext &ctx) {
    return fmt::format_to(ctx.out(), "TransferFunction({}) - B: {}, A: {}", N, tf.b, tf.a);
  }
};

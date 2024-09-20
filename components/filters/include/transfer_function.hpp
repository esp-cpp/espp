#pragma once

#include "format.hpp"
#include <array>

namespace espp {
/**
 * @brief Transfer function representing the A and B coefficients.
 */
template <size_t N> struct TransferFunction {
  TransferFunction() = default;

  TransferFunction(const std::array<float, N> &b, const std::array<float, N> &a)
      : b(b)
      , a(a) {}

  std::array<float, N> b = {}; /**< B coefficients. */
  std::array<float, N> a = {}; /**< A coefficients. */
};
} // namespace espp

#include "transfer_function_formatters.hpp"

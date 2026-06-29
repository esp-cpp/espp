#pragma once

#include "format.hpp"

/// @brief espp namespace
namespace espp {
/// @brief espp::detail namespace
namespace detail {
/// @brief Sensor Direction Configuration
enum class SensorDirection {
  CLOCKWISE = 1, //!< The sensor is mounted clockwise (so the positive phase direction leads to
                 //!< positive angle increase).
  COUNTER_CLOCKWISE = -1, //!< The sensor is mounted counter-clockwise (so the positive phase
                          //!< direction leads to negative angle increase).
  UNKNOWN = 0             //!< The direction is unknown.
};
} // namespace detail
} // namespace espp

// for printing SensorDirection using libfmt
template <> struct fmt::formatter<espp::detail::SensorDirection> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(espp::detail::SensorDirection dir, FormatContext &ctx) const {
    switch (dir) {
    case espp::detail::SensorDirection::CLOCKWISE:
      return fmt::format_to(ctx.out(), "CLOCKWISE");
    case espp::detail::SensorDirection::COUNTER_CLOCKWISE:
      return fmt::format_to(ctx.out(), "COUNTER_CLOCKWISE");
    case espp::detail::SensorDirection::UNKNOWN:
      return fmt::format_to(ctx.out(), "UNKNOWN");
    }
    return fmt::format_to(ctx.out(), "UNKNOWN");
  }
};

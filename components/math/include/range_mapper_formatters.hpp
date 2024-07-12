#pragma once

#include "fmt/format.h"

// NOTE: right now it seems we cannot use the generic version that works in
//       vector2d.hpp because it results in 'template class without a name'
//       errors...

template <> struct fmt::formatter<espp::FloatRangeMapper::Config> : fmt::formatter<std::string> {
  auto format(const espp::FloatRangeMapper::Config &config, format_context &ctx) const {
    return fmt::format_to(ctx.out(), "FloatRangeMapper[{},{},{},{},{},{},{},{}]", config.center,
                          config.center_deadband, config.minimum, config.maximum,
                          config.range_deadband, config.output_center, config.output_range,
                          config.invert_output);
  }
};
template <> struct fmt::formatter<espp::IntRangeMapper::Config> : fmt::formatter<std::string> {
  auto format(const espp::IntRangeMapper::Config &config, format_context &ctx) const {
    return fmt::format_to(ctx.out(), "IntRangeMapper[{},{},{},{},{},{},{},{}]", config.center,
                          config.center_deadband, config.minimum, config.maximum,
                          config.range_deadband, config.output_center, config.output_range,
                          config.invert_output);
  }
};

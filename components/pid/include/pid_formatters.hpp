#pragma once

#include "format.hpp"

// for allowing easy serialization/printing of the
// espp::Pid
template <> struct fmt::formatter<espp::Pid> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext> auto format(espp::Pid const &pid, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{}, {}", pid.get_error(), pid.get_integrator());
  }
};

// for allowing easy serialization/printing of the
// espp::Pid::Config
template <> struct fmt::formatter<espp::Pid::Config> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::Pid::Config const &cfg, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "{}, {}, {}, {}, {}, {}, {}", cfg.kp, cfg.ki, cfg.kd,
                          cfg.integrator_min, cfg.integrator_max, cfg.output_min, cfg.output_max);
  }
};

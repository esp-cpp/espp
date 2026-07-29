#pragma once

#include "format.hpp"

// fmt::formatter for espp::TrajectoryPlanner::MotionCommand
template <> struct fmt::formatter<espp::TrajectoryPlanner::MotionCommand> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::TrajectoryPlanner::MotionCommand const &cmd, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "v={:.3f} m/s, ω={:.3f} rad/s", cmd.linear_velocity,
                          cmd.angular_velocity);
  }
};

// fmt::formatter for espp::TrajectoryPlanner::Config
template <> struct fmt::formatter<espp::TrajectoryPlanner::Config> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::TrajectoryPlanner::Config const &cfg, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "max_v={:.3f} m/s, max_w={:.3f} rad/s, max_a_v={:.3f} m/s², max_a_w={:.3f} rad/s², "
        "max_j_v={:.3f} m/s³, max_j_w={:.3f} rad/s³, envelope={}, max_a_c={:.3f} m/s², "
        "max_decel_v={:.3f} m/s², max_decel_w={:.3f} rad/s²",
        cfg.max_linear_velocity, cfg.max_angular_velocity, cfg.max_linear_acceleration,
        cfg.max_angular_acceleration, cfg.max_linear_jerk, cfg.max_angular_jerk,
        cfg.enforce_motion_envelope, cfg.max_centripetal_acceleration, cfg.max_linear_deceleration,
        cfg.max_angular_deceleration);
  }
};

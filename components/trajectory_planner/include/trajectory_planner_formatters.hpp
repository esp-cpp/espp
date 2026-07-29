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
    return fmt::format_to(ctx.out(),
                          "TrajectoryPlanner::Config{{\n"
                          "  max_linear_velocity:        {:.3f} m/s\n"
                          "  max_angular_velocity:       {:.3f} rad/s\n"
                          "  max_linear_acceleration:    {:.3f} m/s²\n"
                          "  max_angular_acceleration:   {:.3f} rad/s²\n"
                          "  max_linear_deceleration:    {:.3f} m/s²\n"
                          "  max_angular_deceleration:   {:.3f} rad/s²\n"
                          "  max_linear_jerk:            {:.3f} m/s³\n"
                          "  max_angular_jerk:           {:.3f} rad/s³\n"
                          "  enforce_motion_envelope:    {}\n"
                          "  max_centripetal_accel:      {:.3f} m/s²\n"
                          "}}",
                          cfg.max_linear_velocity, cfg.max_angular_velocity,
                          cfg.max_linear_acceleration, cfg.max_angular_acceleration,
                          cfg.max_linear_deceleration, cfg.max_angular_deceleration,
                          cfg.max_linear_jerk, cfg.max_angular_jerk, cfg.enforce_motion_envelope,
                          cfg.max_centripetal_acceleration);
  }
};

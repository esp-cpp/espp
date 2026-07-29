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

// fmt::formatter for espp::TrajectoryPlanner::MotionProfile
template <> struct fmt::formatter<espp::TrajectoryPlanner::MotionProfile> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::TrajectoryPlanner::MotionProfile const &p, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "a_v={:.3f} m/s², a_w={:.3f} rad/s², j_v={:.3f} m/s³, j_w={:.3f} rad/s³",
                          p.max_linear_acceleration, p.max_angular_acceleration, p.max_linear_jerk,
                          p.max_angular_jerk);
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
                          "  max_linear_velocity:   {:.3f} m/s\n"
                          "  max_angular_velocity:  {:.3f} rad/s\n"
                          "  driving_profile:       {}\n"
                          "  stopping_profile:      {}\n"
                          "  enforce_envelope:      {}\n"
                          "  max_centripetal_accel: {:.3f} m/s²\n"
                          "}}",
                          cfg.max_linear_velocity, cfg.max_angular_velocity, cfg.driving_profile,
                          cfg.stopping_profile, cfg.enforce_motion_envelope,
                          cfg.max_centripetal_acceleration);
  }
};

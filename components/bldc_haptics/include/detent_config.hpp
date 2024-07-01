/// @file Defines the detent configuration

#pragma once

#include "format.hpp"
#include <vector>

namespace espp::detail {
/// @brief Configuration for the detents
/// @note max_position < min_position indicates no bounds
struct DetentConfig {
  float position_width;  ///< Width of the positions, in radians
  float min_position{0}; ///< Minimum position of motor, there will be an end stop at this position;
                         ///< There will be (max-min+1) positions between min and max (inclusive).
  float max_position{
      5}; ///< Maximum position of motor, there will be an end stop at this position; There will be
          ///< (max-min+1) positions between min and max (inclusive). max < min indicates no bounds
  std::vector<int> detent_positions{}; ///< Positions of the detents, an integer value between min
                                       ///< and max position. Optional, if empty / not specified, no
                                       ///< detents will be used.
  float detent_strength{1};            ///< Strength of the detents
  float end_strength{1};               ///< Strength of the end detents
  float snap_point{1.1}; ///< Position of the snap point, in radians, should be >= 0.5 for stability
  float snap_point_bias{0}; ///< Bias for the snap point, in radians, should be >= 0 for stability
  float dead_zone_percent{0.2f}; ///< Percent of the dead zone to use for the detent
  float dead_zone_abs_max_radians{
      M_PI / 180.0f}; ///< Absolute maximum of the dead zone to use for the detent in radians
};

/// @brief Equality operator for DetentConfig
/// @param lhs Left hand side of the equality
/// @param rhs Right hand side of the equality
/// @return True if the two DetentConfigs are equal
inline bool operator==(const DetentConfig &lhs, const DetentConfig &rhs) {
  bool vectors_equal = lhs.detent_positions.size() == rhs.detent_positions.size() &&
                       std::equal(lhs.detent_positions.begin(), lhs.detent_positions.end(),
                                  rhs.detent_positions.begin());
  return lhs.position_width == rhs.position_width && lhs.min_position == rhs.min_position &&
         lhs.max_position == rhs.max_position && vectors_equal &&
         lhs.detent_strength == rhs.detent_strength && lhs.end_strength == rhs.end_strength &&
         lhs.snap_point == rhs.snap_point && lhs.snap_point_bias == rhs.snap_point_bias;
}

/// @brief Unbounded motion, no detents
static const DetentConfig UNBOUNDED_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = -1, // max < min indicates no bounds
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief Bounded motion, no detents
static const DetentConfig BOUNDED_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 10,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief Bounded motion with multiple revolutions, no detents, with end stops
static const DetentConfig MULTI_REV_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 72,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief On-off with strong detents
static const DetentConfig ON_OFF_STRONG_DETENTS = {
    .position_width = 60.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 1,
    .detent_strength = 1,
    .end_strength = 1,
    .snap_point = 0.55, // Note the snap point is slightly past the midpoint (0.5); compare to
                        // normal detents which use a snap point *past* the next value (i.e. > 1)
    .snap_point_bias = 0,
};

/// @brief Bounded motion with strong position detents spaced 9 degrees apart
/// (coarse), with end stops
static const DetentConfig COARSE_VALUES_STRONG_DETENTS = {
    .position_width = 8.225f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 31,
    .detent_strength = 2,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief Bounded motion with no detents spaced 1 degree apart (fine), with end
/// stops
static const DetentConfig FINE_VALUES_NO_DETENTS = {
    .position_width = 1.0f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 255,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief Bounded motion with position detents spaced 1 degree apart (fine),
/// with end stops
static const DetentConfig FINE_VALUES_WITH_DETENTS = {
    .position_width = 1.0f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 255,
    .detent_strength = 1,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

/// @brief Bounded motion with position detents, end stops, and explicit
/// magnetic detents.
static const DetentConfig MAGNETIC_DETENTS = {
    .position_width = 7.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 31,
    .detent_positions = {2, 10, 21, 22},
    .detent_strength = 2.5,
    .end_strength = 1,
    .snap_point = 0.7,
    .snap_point_bias = 0,
};

/// @brief Bounded motion for a return to center rotary encoder with positions
static const DetentConfig RETURN_TO_CENTER_WITH_DETENTS = {
    .position_width = 60.0 * M_PI / 180.0,
    .min_position = -6,
    .max_position = 6,
    .detent_strength = 1,
    .end_strength = 1,
    .snap_point = 0.55,
    .snap_point_bias = 0.4,
};

} // namespace espp::detail

// for allowing easy serialization/printing of the
// DetentConfig
template <> struct fmt::formatter<espp::detail::DetentConfig> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) const {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(espp::detail::DetentConfig const &detent_config, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(),
                          "DetentConfig:\n"
                          "\tposition_width: {} radians ({} degrees)\n"
                          "\tmin_position: {}\n"
                          "\tmax_position: {}\n"
                          "\trange of motion: {:.2f} to {:.2f} degrees\n"
                          "\tdetent_positions: {}\n"
                          "\tdetent_strength: {}\n"
                          "\tend_strength: {}\n"
                          "\tsnap_point: {}\n"
                          "\tsnap_point_bias: {}",
                          detent_config.position_width, detent_config.position_width * 180.0 / M_PI,
                          detent_config.min_position, detent_config.max_position,
                          detent_config.min_position * detent_config.position_width * 180.0 / M_PI,
                          detent_config.max_position * detent_config.position_width * 180.0 / M_PI,
                          detent_config.detent_positions, detent_config.detent_strength,
                          detent_config.end_strength, detent_config.snap_point,
                          detent_config.snap_point_bias);
  }
};

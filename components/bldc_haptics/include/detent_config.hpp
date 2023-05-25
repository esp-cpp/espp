#pragma once

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

/*
static const DetentConfig UNBOUNDED_NO_DETENTS;
static const DetentConfig BOUNDED_NO_DETENTS;
static const DetentConfig MULTI_REV_NO_DETENTS;
static const DetentConfig COARSE_VALUES_STRONG_DETENTS;
static const DetentConfig FINE_VALUES_NO_DETENTS;
static const DetentConfig FINE_VALUES_WITH_DETENTS;
static const DetentConfig MAGNETIC_DETENTS;
static const DetentConfig RETURN_TO_CENTER_WITH_DETENTS;
*/
static const DetentConfig UNBOUNDED_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = -1, // max < min indicates no bounds
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

static const DetentConfig BOUNDED_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 10,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

static const DetentConfig MULTI_REV_NO_DETENTS = {
    .position_width = 10.0 * M_PI / 180.0,
    .min_position = 0,
    .max_position = 72,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

static const DetentConfig COARSE_VALUES_STRONG_DETENTS = {
    .position_width = 8.225f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 31,
    .detent_strength = 2,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

static const DetentConfig FINE_VALUES_NO_DETENTS = {
    .position_width = 1.0f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 255,
    .detent_strength = 0,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

static const DetentConfig FINE_VALUES_WITH_DETENTS = {
    .position_width = 1.0f * M_PI / 180.0,
    .min_position = 0,
    .max_position = 255,
    .detent_strength = 1,
    .end_strength = 1,
    .snap_point = 1.1,
    .snap_point_bias = 0,
};

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

static const DetentConfig RETURN_TO_CENTER_WITH_DETENTS = {
    .position_width = 0,
    .min_position = -6,
    .max_position = 6,
    .detent_strength = 1,
    .end_strength = 1,
    .snap_point = 0.55,
    .snap_point_bias = 0.4,
};

} // namespace espp::detail

#pragma once

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

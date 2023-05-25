#pragma once

#include <chrono>

namespace espp::detail {
/// @brief Configuration for the haptic feedback
struct HapticConfig {
  float strength;                        ///< Strength of the haptic feedback
  std::chrono::duration<float> duration; ///< Duration of the haptic feedback
};
} // namespace espp::detail

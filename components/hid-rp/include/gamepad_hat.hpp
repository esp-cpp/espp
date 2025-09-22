#pragma once

#include <cstdint>

namespace espp {
namespace gamepad {
/// Possible Hat switch directions
enum class Hat : std::uint8_t {
  CENTERED = 0x0f, ///< Centered, no direction pressed.
  UP = 1,
  UP_RIGHT,
  RIGHT,
  DOWN_RIGHT,
  DOWN,
  DOWN_LEFT,
  LEFT,
  UP_LEFT
};
} // namespace gamepad
} // namespace espp

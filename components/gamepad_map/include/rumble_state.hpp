#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

namespace espp {

/**
 * @brief How strongly to drive a gamepad's rumble motors.
 *
 * @details The output-direction counterpart of GamepadState: a host's rumble
 *          request is decoded into it (a Switch's HD rumble, an X-Input
 *          rumble packet, an emulator's rumble pak...), and it is encoded into
 *          whatever drives the motors (a controller's rumble output report, a
 *          haptic driver). Levels are 0 (off) .. 255 (full).
 *
 *          Strong is the large, low-frequency motor (in the left grip of a
 *          hand-held controller), Weak the small, high-frequency one (right
 *          grip). Controllers with impulse triggers (the Xbox Wireless
 *          controller) also have a motor behind each trigger.
 */
struct RumbleState {
  /// A rumble motor.
  enum class Motor : uint8_t {
    Strong,       ///< large, low-frequency motor (left grip)
    Weak,         ///< small, high-frequency motor (right grip)
    LeftTrigger,  ///< impulse-trigger motor behind the left trigger
    RightTrigger, ///< impulse-trigger motor behind the right trigger
    Count         ///< number of motors (not a motor)
  };

  static constexpr size_t MOTOR_COUNT = static_cast<size_t>(Motor::Count);
  static constexpr uint8_t LEVEL_MAX = 255; ///< full strength

  std::array<uint8_t, MOTOR_COUNT> levels{}; ///< indexed by Motor

  /// @brief The level of a motor (0 for an invalid motor).
  constexpr uint8_t level(Motor m) const {
    return m < Motor::Count ? levels[static_cast<uint8_t>(m)] : 0;
  }
  /// @brief Set a motor's level (ignored for an invalid motor).
  constexpr void set(Motor m, uint8_t value) {
    if (m < Motor::Count)
      levels[static_cast<uint8_t>(m)] = value;
  }
  /// @brief Whether any motor is on.
  constexpr bool any() const {
    return std::any_of(levels.begin(), levels.end(), [](uint8_t l) { return l != 0; });
  }
  /// @brief All motors off.
  constexpr void clear() { levels.fill(0); }

  constexpr bool operator==(const RumbleState &) const = default;
};

} // namespace espp

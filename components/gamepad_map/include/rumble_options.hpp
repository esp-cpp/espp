#pragma once

#include <algorithm>
#include <cstdint>
#include <string>
#include <type_traits>

#include "rumble_state.hpp"

namespace espp {

/**
 * @brief How to play a host's rumble on one controller type: off, a strength
 *        (0..200 %), and whether the strong and weak motors trade places.
 *
 * @details Plain data. GamepadProfiles stores one per profile plus one for
 *          controllers without a profile, and GamepadMapService sends it field
 *          for field ([flags u8][strength_percent u8]). RumbleOptions{} is
 *          neutral: apply() returns its input.
 *
 *          Apply it exactly ONCE, where the host's request becomes the
 *          controller's motor command. Re-apply it to the stored raw request
 *          whenever the request or the options change, never to an
 *          already-applied state: scaling compounds and a swap undoes itself.
 *          An all-zero result is a stop to SEND. "Off" means "send stops",
 *          not "send nothing".
 *
 *          apply() scales every motor's level by strength_percent / 100,
 *          rounding half up, and saturates each motor on its own at
 *          RumbleState::LEVEL_MAX (so above 100 % strong effects play at full
 *          strength). SWAP_MOTORS trades the Strong and Weak levels; the
 *          trigger motors are scaled but never swapped. A level of 0 stays 0
 *          for every option value, so apply() never turns on a motor the host
 *          left off. It is pure, constexpr and does not allocate.
 *
 *          A new flag bit is a compatible addition (no schema bump) only if a
 *          clear bit means today's behaviour. Firmware that does not know a bit
 *          ignores it in apply(), rejects it in valid(), and clears it when it
 *          loads a stored file. A console must learn from a capability bit
 *          that a device honours a new flag.
 *
 * \section rumble_options_ex1 Rumble Options Example
 * \snippet gamepad_map_example.cpp rumble options example
 */
struct RumbleOptions {
  static constexpr uint8_t RUMBLE_OFF = 0x01;  ///< flag: every request plays as a stop
  static constexpr uint8_t SWAP_MOTORS = 0x02; ///< flag: Strong plays Weak's level and vice versa
  static constexpr uint8_t KNOWN_FLAGS =
      RUMBLE_OFF | SWAP_MOTORS;                            ///< every flag this firmware knows
  static constexpr uint8_t DEFAULT_STRENGTH_PERCENT = 100; ///< as the host asks
  static constexpr uint8_t MAX_STRENGTH_PERCENT = 200;     ///< largest valid strength

  uint8_t flags{0};                                   ///< RUMBLE_OFF | SWAP_MOTORS
  uint8_t strength_percent{DEFAULT_STRENGTH_PERCENT}; ///< 0..200: every motor's level x this / 100

  /// @brief Whether every request plays as a stop (RUMBLE_OFF).
  constexpr bool is_off() const { return flags & RUMBLE_OFF; }
  /// @brief Whether the Strong and Weak motors trade places (SWAP_MOTORS).
  constexpr bool swaps_motors() const { return flags & SWAP_MOTORS; }
  /// @brief Whether apply() changes nothing (unknown flag bits are ignored).
  constexpr bool is_neutral() const {
    return (flags & KNOWN_FLAGS) == 0 && strength_percent == DEFAULT_STRENGTH_PERCENT;
  }

  /// @brief Whether the options are in range; `why` (if given) says what is wrong:
  ///        "unknown rumble flags" / "strength out of range (0..200 %)".
  bool valid(std::string *why = nullptr) const {
    auto fail = [why](const char *reason) {
      if (why)
        *why = reason;
      return false;
    };
    if (flags & ~KNOWN_FLAGS)
      return fail("unknown rumble flags");
    if (strength_percent > MAX_STRENGTH_PERCENT)
      return fail("strength out of range (0..200 %)");
    return true;
  }

  /**
   * @brief The levels to play for a host's request (see the class notes).
   * @param request The host's request, as it asked for it (never an
   *        already-applied state).
   * @return All zero when off; otherwise every motor scaled by the strength
   *         (clamped to MAX_STRENGTH_PERCENT), Strong and Weak swapped when
   *         SWAP_MOTORS is set. Unknown flag bits are ignored.
   */
  constexpr RumbleState apply(const RumbleState &request) const {
    using Motor = RumbleState::Motor;
    if (is_off())
      return RumbleState{};
    const uint8_t pct = std::min(strength_percent, MAX_STRENGTH_PERCENT);
    const bool swap = swaps_motors();
    RumbleState out;
    out.set(Motor::Strong, scale(request.level(swap ? Motor::Weak : Motor::Strong), pct));
    out.set(Motor::Weak, scale(request.level(swap ? Motor::Strong : Motor::Weak), pct));
    out.set(Motor::LeftTrigger, scale(request.level(Motor::LeftTrigger), pct));
    out.set(Motor::RightTrigger, scale(request.level(Motor::RightTrigger), pct));
    return out;
  }

  constexpr bool operator==(const RumbleOptions &) const = default;

private:
  static constexpr uint8_t scale(uint8_t level, uint8_t percent) {
    const uint32_t v = (uint32_t{level} * percent + 50) / 100; // round half up
    return static_cast<uint8_t>(std::min<uint32_t>(v, RumbleState::LEVEL_MAX));
  }
};

static_assert(std::is_aggregate_v<RumbleOptions> && std::is_trivially_copyable_v<RumbleOptions>);

} // namespace espp

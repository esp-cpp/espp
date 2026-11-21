#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace espp {

/**
 * @brief The state of a gamepad, by control position.
 *
 * @details The common currency of the gamepad_map component: inputs are
 *          decoded into it (from a HID report, a USB/BLE controller, GPIO
 *          buttons...), a GamepadMap rewrites it, and the result is encoded
 *          into whatever comes next (a USB controller report, an emulator's
 *          input).
 *
 *          Face buttons are named by **position**, not label, because the
 *          labels disagree between controller families: south is Xbox A /
 *          Nintendo B / PlayStation Cross, east is Xbox B / Nintendo A /
 *          PlayStation Circle, west is Xbox X / Nintendo Y / Square and north
 *          is Xbox Y / Nintendo X / Triangle.
 *
 *          Sticks are -32767..32767 with X growing to the right and Y growing
 *          **downwards** (the HID convention); triggers are 0..32767.
 */
struct GamepadState {
  /// A digital control.
  enum class Button : uint8_t {
    South,   ///< Xbox A, Nintendo B, PlayStation Cross
    East,    ///< Xbox B, Nintendo A, PlayStation Circle
    West,    ///< Xbox X, Nintendo Y, PlayStation Square
    North,   ///< Xbox Y, Nintendo X, PlayStation Triangle
    L1,      ///< left bumper / L
    R1,      ///< right bumper / R
    L2,      ///< left trigger as a button / ZL
    R2,      ///< right trigger as a button / ZR
    L3,      ///< left stick click
    R3,      ///< right stick click
    Select,  ///< View / Back / Minus / Share
    Start,   ///< Menu / Plus / Options
    Home,    ///< Guide / Home / PS
    Capture, ///< Share (Xbox Series) / Capture (Nintendo)
    Up,      ///< d-pad up
    Down,    ///< d-pad down
    Left,    ///< d-pad left
    Right,   ///< d-pad right
    Paddle1, ///< extra / back buttons (paddles, SL/SR...)
    Paddle2,
    Paddle3,
    Paddle4,
    Count ///< number of buttons (not a button)
  };

  /// An analog control.
  enum class Axis : uint8_t {
    LeftX,        ///< left stick X, -32767 (left) .. 32767 (right)
    LeftY,        ///< left stick Y, -32767 (up) .. 32767 (down)
    RightX,       ///< right stick X
    RightY,       ///< right stick Y
    LeftTrigger,  ///< 0 (released) .. 32767 (fully pressed)
    RightTrigger, ///< 0 (released) .. 32767 (fully pressed)
    Count         ///< number of axes (not an axis)
  };

  static constexpr size_t BUTTON_COUNT = static_cast<size_t>(Button::Count);
  static constexpr size_t AXIS_COUNT = static_cast<size_t>(Axis::Count);
  static constexpr int16_t AXIS_MAX = 32767; ///< full deflection of a stick or trigger

  uint32_t buttons{0};                    ///< bit n = Button n pressed
  std::array<int16_t, AXIS_COUNT> axes{}; ///< indexed by Axis

  /// @brief Whether a button is pressed.
  constexpr bool pressed(Button b) const {
    return b < Button::Count && (buttons >> static_cast<uint8_t>(b)) & 1u;
  }
  /// @brief Press or release a button.
  constexpr void set(Button b, bool down) {
    if (b >= Button::Count)
      return;
    const uint32_t bit = 1u << static_cast<uint8_t>(b);
    buttons = down ? (buttons | bit) : (buttons & ~bit);
  }
  /// @brief The value of an axis (0 for an invalid axis).
  constexpr int16_t axis(Axis a) const {
    return a < Axis::Count ? axes[static_cast<uint8_t>(a)] : 0;
  }
  /// @brief Set an axis, clamped to its range (see is_trigger()).
  constexpr void set(Axis a, int32_t value) {
    if (a >= Axis::Count)
      return;
    axes[static_cast<uint8_t>(a)] = clamp(a, value);
  }

  /// @brief Whether an axis is a trigger (0..32767) rather than a stick axis.
  static constexpr bool is_trigger(Axis a) {
    return a == Axis::LeftTrigger || a == Axis::RightTrigger;
  }
  /// @brief Clamp a value to an axis' range.
  static constexpr int16_t clamp(Axis a, int32_t value) {
    const int32_t lo = is_trigger(a) ? 0 : -AXIS_MAX;
    return static_cast<int16_t>(value < lo ? lo : value > AXIS_MAX ? AXIS_MAX : value);
  }

  /// @brief Short, stable name of a button ("south", "l1", "up"...), for logs and UIs.
  static constexpr const char *name(Button b) {
    constexpr const char *names[] = {"south",   "east",    "west",    "north",  "l1",     "r1",
                                     "l2",      "r2",      "l3",      "r3",     "select", "start",
                                     "home",    "capture", "up",      "down",   "left",   "right",
                                     "paddle1", "paddle2", "paddle3", "paddle4"};
    static_assert(sizeof(names) / sizeof(names[0]) == BUTTON_COUNT);
    return b < Button::Count ? names[static_cast<uint8_t>(b)] : "?";
  }
  /// @brief Short, stable name of an axis ("left_x", "right_trigger"...).
  static constexpr const char *name(Axis a) {
    constexpr const char *names[] = {"left_x",  "left_y",       "right_x",
                                     "right_y", "left_trigger", "right_trigger"};
    static_assert(sizeof(names) / sizeof(names[0]) == AXIS_COUNT);
    return a < Axis::Count ? names[static_cast<uint8_t>(a)] : "?";
  }

  bool operator==(const GamepadState &) const = default;
};

} // namespace espp

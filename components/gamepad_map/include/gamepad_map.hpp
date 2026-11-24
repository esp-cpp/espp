#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "gamepad_state.hpp"

namespace espp {

/**
 * @brief One rule of a GamepadMap: where one control's input goes.
 *
 * @details Four kinds of rule cover the remaps controllers need:
 *          - **button -> button**: the source button presses the target
 *            (swap A/B, move a function to a paddle);
 *          - **axis -> button**: the target is pressed while the source axis
 *            is past a threshold (a stick direction as the d-pad, an analog
 *            trigger as ZL/ZR). A positive threshold presses at values >= it,
 *            a negative one at values <= it (stick up / left);
 *          - **button -> axis**: while the source button is pressed it adds a
 *            value to the target axis (the d-pad as the left stick, a button
 *            as a full trigger);
 *          - **axis -> axis**: the source axis, scaled by a percentage (a
 *            negative scale inverts), is added to the target (swap or invert
 *            sticks, a stick as a trigger). A trigger inverted onto a trigger
 *            reads from full: (32767 - value) x |scale|, so it rests pressed.
 *
 *          A control that is the source of a rule no longer reaches its own
 *          position (it is "moved", not copied) unless the rule sets
 *          KEEP_SOURCE; a rule whose target is NONE just removes its source.
 *          Several rules may feed one target: buttons are OR-ed, axis
 *          contributions are summed and clamped.
 *
 *          The rules of a map form a table applied to the controller's input
 *          all at once, not one after the other: every rule reads the input
 *          as the controller reported it, never another rule's output. So a
 *          control that is the source of two rules feeds both targets
 *          (`swap(South, East).swap(East, North)` makes East press South AND
 *          North), and `invert(LeftY).swap_sticks()` sends the left stick's Y
 *          inverted to itself and as is to the right stick. Give each source
 *          the one rule (or, with KEEP_SOURCE, the rules) it should have.
 *
 *          The struct is plain data (it is stored with PersistentData and
 *          sent over the wire as is); build rules with the static helpers.
 *          Stored as is inside GamepadProfiles' file: a layout change needs a
 *          new schema version (see GamepadProfileV1).
 */
struct GamepadMapRule {
  using Button = GamepadState::Button;
  using Axis = GamepadState::Axis;

  /// What the source and target are.
  enum class Kind : uint8_t {
    ButtonToButton, ///< param unused
    AxisToButton,   ///< param = threshold (sign = direction), never 0
    ButtonToAxis,   ///< param = value added to the target while pressed
    AxisToAxis,     ///< param = scale in percent, -400..400 (negative inverts)
  };

  static constexpr uint8_t NONE = 0xFF;        ///< target: remove the source, send it nowhere
  static constexpr uint8_t KEEP_SOURCE = 0x01; ///< flag: the source also stays where it was
  static constexpr int16_t MAX_SCALE_PERCENT = 400;

  Kind kind{Kind::ButtonToButton};
  uint8_t source{0};    ///< a Button or an Axis (by kind)
  uint8_t target{NONE}; ///< a Button or an Axis (by kind), or NONE
  uint8_t flags{0};     ///< KEEP_SOURCE
  int16_t param{0};     ///< see Kind

  /// @brief Whether the source stays at its own position too.
  constexpr bool keeps_source() const { return flags & KEEP_SOURCE; }
  /// @brief Whether the source is an axis.
  constexpr bool source_is_axis() const {
    return kind == Kind::AxisToButton || kind == Kind::AxisToAxis;
  }
  /// @brief Whether the target is an axis.
  constexpr bool target_is_axis() const {
    return kind == Kind::ButtonToAxis || kind == Kind::AxisToAxis;
  }

  /// @brief `from` presses `to`.
  static constexpr GamepadMapRule button(Button from, Button to, bool keep_source = false) {
    return {Kind::ButtonToButton, static_cast<uint8_t>(from), static_cast<uint8_t>(to),
            keep_flag(keep_source), 0};
  }
  /// @brief `to` is pressed while `from` is at or past `threshold` (a negative
  ///        threshold: at or below it). 16384 is half way.
  static constexpr GamepadMapRule axis_to_button(Axis from, int16_t threshold, Button to,
                                                 bool keep_source = false) {
    return {Kind::AxisToButton, static_cast<uint8_t>(from), static_cast<uint8_t>(to),
            keep_flag(keep_source), threshold};
  }
  /// @brief While `from` is pressed, `value` is added to `to` (e.g. 32767 for a
  ///        full trigger, -32767 for the left stick fully left).
  static constexpr GamepadMapRule button_to_axis(Button from, Axis to, int16_t value,
                                                 bool keep_source = false) {
    return {Kind::ButtonToAxis, static_cast<uint8_t>(from), static_cast<uint8_t>(to),
            keep_flag(keep_source), value};
  }
  /// @brief `from` scaled by `scale_percent` is added to `to` (-100 inverts).
  static constexpr GamepadMapRule axis(Axis from, Axis to, int16_t scale_percent = 100,
                                       bool keep_source = false) {
    return {Kind::AxisToAxis, static_cast<uint8_t>(from), static_cast<uint8_t>(to),
            keep_flag(keep_source), scale_percent};
  }
  /// @brief Remove a button.
  static constexpr GamepadMapRule disable(Button b) {
    return {Kind::ButtonToButton, static_cast<uint8_t>(b), NONE, 0, 0};
  }
  /// @brief Remove an axis (it reads 0 / released).
  static constexpr GamepadMapRule disable(Axis a) {
    return {Kind::AxisToAxis, static_cast<uint8_t>(a), NONE, 0, 100};
  }

  /// @brief Whether the rule is well formed; `why` (if given) says what is wrong.
  bool valid(std::string *why = nullptr) const {
    auto fail = [why](const char *reason) {
      if (why)
        *why = reason;
      return false;
    };
    if (kind > Kind::AxisToAxis)
      return fail("unknown rule kind");
    if (flags & ~KEEP_SOURCE)
      return fail("unknown flags");
    const size_t sources = source_is_axis() ? GamepadState::AXIS_COUNT : GamepadState::BUTTON_COUNT;
    const size_t targets = target_is_axis() ? GamepadState::AXIS_COUNT : GamepadState::BUTTON_COUNT;
    if (source >= sources)
      return fail("source out of range");
    if (target != NONE && target >= targets)
      return fail("target out of range");
    if (kind == Kind::AxisToButton && param == 0)
      return fail("axis -> button needs a non-zero threshold");
    if (kind == Kind::AxisToButton && param < 0 &&
        GamepadState::is_trigger(static_cast<Axis>(source)))
      return fail("a trigger never goes below 0: its threshold must be positive");
    if (target == NONE && keeps_source())
      return fail("a rule that keeps its source needs a target");
    if (kind == Kind::AxisToAxis && (param < -MAX_SCALE_PERCENT || param > MAX_SCALE_PERCENT))
      return fail("axis -> axis scale out of range");
    return true;
  }

  constexpr bool operator==(const GamepadMapRule &) const = default;

private:
  static constexpr uint8_t keep_flag(bool keep) { return keep ? KEEP_SOURCE : 0; }
};

/**
 * @brief A button / axis mapping for a gamepad: radial stick dead zones plus a
 *        list of GamepadMapRule.
 *
 * @details apply() turns the state a controller reports into the state to
 *          send on: dead zones first (they belong to the physical sticks),
 *          then the rules; every control no rule takes as its source passes
 *          through unchanged, so an empty map is the identity. apply() does
 *          not allocate and touches each rule twice, so it is cheap enough to
 *          run on every report.
 *
 *          A GamepadMap is plain data: copy it, store it (see GamepadProfiles),
 *          send it. Use valid() on one that came from outside (a file, a web
 *          page) before using it. Stored as is inside GamepadProfiles' file: a
 *          layout change needs a new schema version (see GamepadProfileV1).
 *
 * \section gamepad_map_ex1 Gamepad Map Example
 * \snippet gamepad_map_example.cpp gamepad map example
 */
struct GamepadMap {
  using Button = GamepadState::Button;
  using Axis = GamepadState::Axis;
  using Rule = GamepadMapRule;

  static constexpr size_t MAX_RULES = 64;             ///< rules a valid map may hold
  static constexpr uint8_t MAX_DEADZONE_PERCENT = 90; ///< largest valid dead zone

  uint8_t left_deadzone_percent{0};  ///< radial dead zone of the left stick, 0..90
  uint8_t right_deadzone_percent{0}; ///< radial dead zone of the right stick, 0..90
  std::vector<GamepadMapRule> rules{};

  /// @brief Whether the map is well formed; `why` (if given) says what is wrong.
  bool valid(std::string *why = nullptr) const {
    if (left_deadzone_percent > MAX_DEADZONE_PERCENT ||
        right_deadzone_percent > MAX_DEADZONE_PERCENT) {
      if (why)
        *why = "dead zone out of range";
      return false;
    }
    if (rules.size() > MAX_RULES) {
      if (why)
        *why = "too many rules";
      return false;
    }
    for (size_t i = 0; i < rules.size(); i++) {
      std::string reason;
      if (!rules[i].valid(&reason)) {
        if (why)
          *why = "rule " + std::to_string(i) + ": " + reason;
        return false;
      }
    }
    return true;
  }

  /// @brief Whether the map changes nothing.
  bool is_identity() const {
    return left_deadzone_percent == 0 && right_deadzone_percent == 0 && rules.empty();
  }

  /// @brief Map `in` to `out` (they may be the same object). Rules that are
  ///        not valid() are skipped.
  void apply(const GamepadState &in_state, GamepadState &out) const {
    GamepadState in = in_state; // `out` may alias `in_state`
    apply_deadzone(in, Axis::LeftX, Axis::LeftY, left_deadzone_percent);
    apply_deadzone(in, Axis::RightX, Axis::RightY, right_deadzone_percent);

    // controls moved elsewhere do not pass through
    uint32_t moved_buttons = 0;
    uint32_t moved_axes = 0;
    for (const auto &r : rules) {
      if (r.keeps_source() || !usable(r))
        continue;
      (r.source_is_axis() ? moved_axes : moved_buttons) |= 1u << r.source;
    }
    std::array<int32_t, GamepadState::AXIS_COUNT> sum{};
    for (size_t a = 0; a < GamepadState::AXIS_COUNT; a++) {
      if (!(moved_axes & (1u << a)))
        sum[a] = in.axes[a];
    }
    uint32_t buttons = in.buttons & ~moved_buttons;

    for (const auto &r : rules) {
      if (r.target == Rule::NONE || !usable(r))
        continue;
      switch (r.kind) {
      case Rule::Kind::ButtonToButton:
        if ((in.buttons >> r.source) & 1u)
          buttons |= 1u << r.target;
        break;
      case Rule::Kind::AxisToButton: {
        const int16_t v = in.axes[r.source];
        if (r.param > 0 ? v >= r.param : v <= r.param)
          buttons |= 1u << r.target;
        break;
      }
      case Rule::Kind::ButtonToAxis:
        if ((in.buttons >> r.source) & 1u)
          sum[r.target] += r.param;
        break;
      case Rule::Kind::AxisToAxis: {
        const int32_t v = in.axes[r.source];
        const bool trigger_to_trigger = GamepadState::is_trigger(static_cast<Axis>(r.source)) &&
                                        GamepadState::is_trigger(static_cast<Axis>(r.target));
        // an inverted trigger onto a trigger reads from full instead of
        // going negative (which would clamp to "never pressed")
        sum[r.target] += trigger_to_trigger && r.param < 0
                             ? (GamepadState::AXIS_MAX - v) * -r.param / 100
                             : v * r.param / 100;
        break;
      }
      }
    }
    out.buttons = buttons;
    for (size_t a = 0; a < GamepadState::AXIS_COUNT; a++)
      out.axes[a] = GamepadState::clamp(static_cast<Axis>(a), sum[a]);
  }

  /// @brief The mapped copy of `in`.
  GamepadState apply(const GamepadState &in) const {
    GamepadState out;
    apply(in, out);
    return out;
  }

  // --- building maps ---

  /// @brief Add a rule (no check: see valid()). Returns *this for chaining.
  GamepadMap &add(const GamepadMapRule &rule) {
    rules.push_back(rule);
    return *this;
  }
  /// @brief Exchange two buttons (e.g. south/east for Nintendo A/B placement).
  GamepadMap &swap(Button a, Button b) { return add(Rule::button(a, b)).add(Rule::button(b, a)); }
  /// @brief Exchange two axes (e.g. the sticks, one axis at a time).
  GamepadMap &swap(Axis a, Axis b) { return add(Rule::axis(a, b)).add(Rule::axis(b, a)); }
  /// @brief Invert an axis (stick Y up-positive, say; a trigger then rests fully pressed).
  GamepadMap &invert(Axis a) { return add(Rule::axis(a, a, -100)); }
  /// @brief Exchange the left and right sticks.
  GamepadMap &swap_sticks() {
    return swap(Axis::LeftX, Axis::RightX).swap(Axis::LeftY, Axis::RightY);
  }
  /// @brief The left stick also drives the d-pad past `threshold` (default 50%).
  GamepadMap &left_stick_to_dpad(int16_t threshold = 16384, bool keep_stick = true) {
    return add(Rule::axis_to_button(Axis::LeftX, static_cast<int16_t>(-threshold), Button::Left,
                                    keep_stick))
        .add(Rule::axis_to_button(Axis::LeftX, threshold, Button::Right, keep_stick))
        .add(Rule::axis_to_button(Axis::LeftY, static_cast<int16_t>(-threshold), Button::Up,
                                  keep_stick))
        .add(Rule::axis_to_button(Axis::LeftY, threshold, Button::Down, keep_stick));
  }
  /// @brief The d-pad also drives the left stick to full deflection.
  GamepadMap &dpad_to_left_stick(bool keep_dpad = true) {
    return add(Rule::button_to_axis(Button::Left, Axis::LeftX, -GamepadState::AXIS_MAX, keep_dpad))
        .add(Rule::button_to_axis(Button::Right, Axis::LeftX, GamepadState::AXIS_MAX, keep_dpad))
        .add(Rule::button_to_axis(Button::Up, Axis::LeftY, -GamepadState::AXIS_MAX, keep_dpad))
        .add(Rule::button_to_axis(Button::Down, Axis::LeftY, GamepadState::AXIS_MAX, keep_dpad));
  }
  /// @brief The analog triggers also press L2 / R2 past `threshold` (default 40%).
  GamepadMap &triggers_to_buttons(int16_t threshold = 13107, bool keep_triggers = true) {
    return add(Rule::axis_to_button(Axis::LeftTrigger, threshold, Button::L2, keep_triggers))
        .add(Rule::axis_to_button(Axis::RightTrigger, threshold, Button::R2, keep_triggers));
  }
  /// @brief L2 / R2 also pull the analog triggers fully (for digital-trigger pads).
  GamepadMap &buttons_to_triggers(bool keep_buttons = true) {
    return add(Rule::button_to_axis(Button::L2, Axis::LeftTrigger, GamepadState::AXIS_MAX,
                                    keep_buttons))
        .add(Rule::button_to_axis(Button::R2, Axis::RightTrigger, GamepadState::AXIS_MAX,
                                  keep_buttons));
  }

  bool operator==(const GamepadMap &) const = default;

private:
  static bool usable(const GamepadMapRule &r) {
    // the cheap part of valid(): indices in range, so apply() never reads or
    // writes outside the state
    const size_t sources =
        r.source_is_axis() ? GamepadState::AXIS_COUNT : GamepadState::BUTTON_COUNT;
    const size_t targets =
        r.target_is_axis() ? GamepadState::AXIS_COUNT : GamepadState::BUTTON_COUNT;
    return r.kind <= GamepadMapRule::Kind::AxisToAxis && r.source < sources &&
           (r.target == GamepadMapRule::NONE || r.target < targets) &&
           !(r.kind == GamepadMapRule::Kind::AxisToButton && r.param == 0);
  }

  static void apply_deadzone(GamepadState &s, Axis x_axis, Axis y_axis, uint8_t percent) {
    if (percent == 0 || percent > MAX_DEADZONE_PERCENT)
      return;
    const float x = s.axis(x_axis), y = s.axis(y_axis);
    const float mag = std::sqrt(x * x + y * y);
    const float dead = GamepadState::AXIS_MAX * (percent / 100.0f);
    if (mag <= dead) {
      s.set(x_axis, 0);
      s.set(y_axis, 0);
      return;
    }
    // rescale what is left so the edge of the dead zone reads 0 and full
    // deflection still reads full; the magnitude is not capped (each axis is
    // clamped on its own), so a square-gate corner stays a full diagonal
    const float scale =
        ((mag - dead) / (GamepadState::AXIS_MAX - dead)) * GamepadState::AXIS_MAX / mag;
    s.set(x_axis, static_cast<int32_t>(std::lround(x * scale)));
    s.set(y_axis, static_cast<int32_t>(std::lround(y * scale)));
  }
};

} // namespace espp

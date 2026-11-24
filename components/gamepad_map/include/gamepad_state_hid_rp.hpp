#pragma once

#include "gamepad_state.hpp"
#include "hid-rp-report-map.hpp"
#include "hid-rp-rumble.hpp"
#include "rumble_state.hpp"

namespace espp {

/**
 * @brief A GamepadState from a decoded HID gamepad report
 *        (hid_rp::GamepadDecoder).
 *
 * @details Both name buttons by position and share the stick convention
 *          (-32767..32767, Y down), so this is a field-by-field copy. For a
 *          pass-through (a controller bridge, a remapper) decode with
 *          `GamepadDecoder::set_combine_controls(false)`, so the d-pad, L2/R2
 *          and Home are only what the controller reports for them and the map
 *          decides the rest.
 * @param r The decoded report.
 * @return The state.
 */
inline GamepadState to_gamepad_state(const hid_rp::GamepadReport &r) {
  using B = GamepadState::Button;
  using A = GamepadState::Axis;
  GamepadState s;
  s.set(B::South, r.south);
  s.set(B::East, r.east);
  s.set(B::West, r.west);
  s.set(B::North, r.north);
  s.set(B::L1, r.l1);
  s.set(B::R1, r.r1);
  s.set(B::L2, r.l2);
  s.set(B::R2, r.r2);
  s.set(B::L3, r.l3);
  s.set(B::R3, r.r3);
  s.set(B::Select, r.select);
  s.set(B::Start, r.start);
  s.set(B::Home, r.home);
  s.set(B::Capture, r.capture);
  s.set(B::Up, r.up);
  s.set(B::Down, r.down);
  s.set(B::Left, r.left);
  s.set(B::Right, r.right);
  s.set(A::LeftX, r.lx);
  s.set(A::LeftY, r.ly);
  s.set(A::RightX, r.rx);
  s.set(A::RightY, r.ry);
  s.set(A::LeftTrigger, r.lt);
  s.set(A::RightTrigger, r.rt);
  return s;
}

/**
 * @brief The levels to hand a hid_rp::RumbleEncoder for a RumbleState.
 * @param r The rumble state.
 * @return The same levels, motor by motor.
 */
inline hid_rp::RumbleLevels to_rumble_levels(const RumbleState &r) {
  using M = RumbleState::Motor;
  return {.strong = r.level(M::Strong),
          .weak = r.level(M::Weak),
          .left_trigger = r.level(M::LeftTrigger),
          .right_trigger = r.level(M::RightTrigger)};
}

/**
 * @brief A RumbleState from the motor levels of a Switch host's rumble packet
 *        (hid_rp::decode_switch_rumble()).
 *
 * @details The left Joy-Con / grip motor drives Strong and the right one
 *          Weak, as a hand-held controller places them.
 * @param r The decoded packet.
 * @return The state.
 */
inline RumbleState to_rumble_state(const hid_rp::SwitchRumble &r) {
  RumbleState s;
  s.set(RumbleState::Motor::Strong, r.left);
  s.set(RumbleState::Motor::Weak, r.right);
  return s;
}

} // namespace espp

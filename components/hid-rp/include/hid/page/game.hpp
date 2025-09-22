#ifndef __HID_PAGE_GAME_HPP_
#define __HID_PAGE_GAME_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class game : std::uint8_t;
template <> struct info<game> {
  constexpr static page_id_t page_id = 0x0005;
  constexpr static usage_id_t max_usage_id = 0x003a;
  constexpr static const char *name = "Game Controls";
};
enum class game : std::uint8_t {
  _3D_GAME_CONTROLLER = 0x0001,
  PINBALL_DEVICE = 0x0002,
  GUN_DEVICE = 0x0003,
  POINT_OF_VIEW = 0x0020,
  TURN_RIGHT_LEFT = 0x0021,
  PITCH_FORWARD_BACKWARD = 0x0022,
  ROLL_RIGHT_LEFT = 0x0023,
  MOVE_RIGHT_LEFT = 0x0024,
  MOVE_FORWARD_BACKWARD = 0x0025,
  MOVE_UP_DOWN = 0x0026,
  LEAN_RIGHT_LEFT = 0x0027,
  LEAN_FORWARD_BACKWARD = 0x0028,
  HEIGHT_OF_POV = 0x0029,
  FLIPPER = 0x002a,
  SECONDARY_FLIPPER = 0x002b,
  BUMP = 0x002c,
  NEW_GAME = 0x002d,
  SHOOT_BALL = 0x002e,
  PLAYER = 0x002f,
  GUN_BOLT = 0x0030,
  GUN_CLIP = 0x0031,
  GUN_SELECTOR = 0x0032,
  GUN_SINGLE_SHOT = 0x0033,
  GUN_BURST = 0x0034,
  GUN_AUTOMATIC = 0x0035,
  GUN_SAFETY = 0x0036,
  GAMEPAD_FIRE_JUMP = 0x0037,
  GAMEPAD_TRIGGER = 0x0039,
  FORM_FITTING_GAMEPAD = 0x003a,
};
} // namespace hid::page

#endif // __HID_PAGE_GAME_HPP_

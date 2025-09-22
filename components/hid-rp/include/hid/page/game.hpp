#ifndef __HID_PAGE_GAME_HPP_
#define __HID_PAGE_GAME_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class game : std::uint8_t;
template <> constexpr inline auto get_info<game>() {
  return info(
      0x0005, 0x003a, "Game Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "3D Game Controller";
        case 0x0002:
          return "Pinball Device";
        case 0x0003:
          return "Gun Device";
        case 0x0020:
          return "Point of View";
        case 0x0021:
          return "Turn Right/Left";
        case 0x0022:
          return "Pitch Forward/Backward";
        case 0x0023:
          return "Roll Right/Left";
        case 0x0024:
          return "Move Right/Left";
        case 0x0025:
          return "Move Forward/Backward";
        case 0x0026:
          return "Move Up/Down";
        case 0x0027:
          return "Lean Right/Left";
        case 0x0028:
          return "Lean Forward/Backward";
        case 0x0029:
          return "Height of POV";
        case 0x002a:
          return "Flipper";
        case 0x002b:
          return "Secondary Flipper";
        case 0x002c:
          return "Bump";
        case 0x002d:
          return "New Game";
        case 0x002e:
          return "Shoot Ball";
        case 0x002f:
          return "Player";
        case 0x0030:
          return "Gun Bolt";
        case 0x0031:
          return "Gun Clip";
        case 0x0032:
          return "Gun Selector";
        case 0x0033:
          return "Gun Single Shot";
        case 0x0034:
          return "Gun Burst";
        case 0x0035:
          return "Gun Automatic";
        case 0x0036:
          return "Gun Safety";
        case 0x0037:
          return "Gamepad Fire/Jump";
        case 0x0039:
          return "Gamepad Trigger";
        case 0x003a:
          return "Form-fitting Gamepad";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
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

#ifndef __HID_PAGE_BRAILLE_DISPLAY_HPP_
#define __HID_PAGE_BRAILLE_DISPLAY_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class braille_display : std::uint16_t;
template <> constexpr inline auto get_info<braille_display>() {
  return info(
      0x0041, 0x021e, "Braille Display",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Braille Display";
        case 0x0002:
          return "Braille Row";
        case 0x0003:
          return "8 Dot Braille Cell";
        case 0x0004:
          return "6 Dot Braille Cell";
        case 0x0005:
          return "Number of Braille Cells";
        case 0x0006:
          return "Screen Reader Control";
        case 0x0007:
          return "Screen Reader Identifier";
        case 0x00fa:
          return "Router Set 1";
        case 0x00fb:
          return "Router Set 2";
        case 0x00fc:
          return "Router Set 3";
        case 0x0100:
          return "Router Key";
        case 0x0101:
          return "Row Router Key";
        case 0x0200:
          return "Braille Buttons";
        case 0x0201:
          return "Braille Keyboard Dot 1";
        case 0x0202:
          return "Braille Keyboard Dot 2";
        case 0x0203:
          return "Braille Keyboard Dot 3";
        case 0x0204:
          return "Braille Keyboard Dot 4";
        case 0x0205:
          return "Braille Keyboard Dot 5";
        case 0x0206:
          return "Braille Keyboard Dot 6";
        case 0x0207:
          return "Braille Keyboard Dot 7";
        case 0x0208:
          return "Braille Keyboard Dot 8";
        case 0x0209:
          return "Braille Keyboard Space";
        case 0x020a:
          return "Braille Keyboard Left Space";
        case 0x020b:
          return "Braille Keyboard Right Space";
        case 0x020c:
          return "Braille Face Controls";
        case 0x020d:
          return "Braille Left Controls";
        case 0x020e:
          return "Braille Right Controls";
        case 0x020f:
          return "Braille Top Controls";
        case 0x0210:
          return "Braille Joystick Center";
        case 0x0211:
          return "Braille Joystick Up";
        case 0x0212:
          return "Braille Joystick Down";
        case 0x0213:
          return "Braille Joystick Left";
        case 0x0214:
          return "Braille Joystick Right";
        case 0x0215:
          return "Braille D-Pad Center";
        case 0x0216:
          return "Braille D-Pad Up";
        case 0x0217:
          return "Braille D-Pad Down";
        case 0x0218:
          return "Braille D-Pad Left";
        case 0x0219:
          return "Braille D-Pad Right";
        case 0x021a:
          return "Braille Pan Left";
        case 0x021b:
          return "Braille Pan Right";
        case 0x021c:
          return "Braille Rocker Up";
        case 0x021d:
          return "Braille Rocker Down";
        case 0x021e:
          return "Braille Rocker Press";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class braille_display : std::uint16_t {
  BRAILLE_DISPLAY = 0x0001,
  BRAILLE_ROW = 0x0002,
  _8_DOT_BRAILLE_CELL = 0x0003,
  _6_DOT_BRAILLE_CELL = 0x0004,
  NUMBER_OF_BRAILLE_CELLS = 0x0005,
  SCREEN_READER_CONTROL = 0x0006,
  SCREEN_READER_IDENTIFIER = 0x0007,
  ROUTER_SET_1 = 0x00fa,
  ROUTER_SET_2 = 0x00fb,
  ROUTER_SET_3 = 0x00fc,
  ROUTER_KEY = 0x0100,
  ROW_ROUTER_KEY = 0x0101,
  BRAILLE_BUTTONS = 0x0200,
  BRAILLE_KEYBOARD_DOT_1 = 0x0201,
  BRAILLE_KEYBOARD_DOT_2 = 0x0202,
  BRAILLE_KEYBOARD_DOT_3 = 0x0203,
  BRAILLE_KEYBOARD_DOT_4 = 0x0204,
  BRAILLE_KEYBOARD_DOT_5 = 0x0205,
  BRAILLE_KEYBOARD_DOT_6 = 0x0206,
  BRAILLE_KEYBOARD_DOT_7 = 0x0207,
  BRAILLE_KEYBOARD_DOT_8 = 0x0208,
  BRAILLE_KEYBOARD_SPACE = 0x0209,
  BRAILLE_KEYBOARD_LEFT_SPACE = 0x020a,
  BRAILLE_KEYBOARD_RIGHT_SPACE = 0x020b,
  BRAILLE_FACE_CONTROLS = 0x020c,
  BRAILLE_LEFT_CONTROLS = 0x020d,
  BRAILLE_RIGHT_CONTROLS = 0x020e,
  BRAILLE_TOP_CONTROLS = 0x020f,
  BRAILLE_JOYSTICK_CENTER = 0x0210,
  BRAILLE_JOYSTICK_UP = 0x0211,
  BRAILLE_JOYSTICK_DOWN = 0x0212,
  BRAILLE_JOYSTICK_LEFT = 0x0213,
  BRAILLE_JOYSTICK_RIGHT = 0x0214,
  BRAILLE_D_PAD_CENTER = 0x0215,
  BRAILLE_D_PAD_UP = 0x0216,
  BRAILLE_D_PAD_DOWN = 0x0217,
  BRAILLE_D_PAD_LEFT = 0x0218,
  BRAILLE_D_PAD_RIGHT = 0x0219,
  BRAILLE_PAN_LEFT = 0x021a,
  BRAILLE_PAN_RIGHT = 0x021b,
  BRAILLE_ROCKER_UP = 0x021c,
  BRAILLE_ROCKER_DOWN = 0x021d,
  BRAILLE_ROCKER_PRESS = 0x021e,
};
} // namespace hid::page

#endif // __HID_PAGE_BRAILLE_DISPLAY_HPP_

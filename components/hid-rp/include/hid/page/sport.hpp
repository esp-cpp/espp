#ifndef __HID_PAGE_SPORT_HPP_
#define __HID_PAGE_SPORT_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class sport : std::uint8_t;
template <> constexpr inline auto get_info<sport>() {
  return info(
      0x0004, 0x0063, "Sport Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Baseball Bat";
        case 0x0002:
          return "Golf Club";
        case 0x0003:
          return "Rowing Machine";
        case 0x0004:
          return "Treadmill";
        case 0x0030:
          return "Oar";
        case 0x0031:
          return "Slope";
        case 0x0032:
          return "Rate";
        case 0x0033:
          return "Stick Speed";
        case 0x0034:
          return "Stick Face Angle";
        case 0x0035:
          return "Stick Heel/Toe";
        case 0x0036:
          return "Stick Follow Through";
        case 0x0037:
          return "Stick Tempo";
        case 0x0038:
          return "Stick Type";
        case 0x0039:
          return "Stick Height";
        case 0x0050:
          return "Putter";
        case 0x0051:
          return "1 Iron";
        case 0x0052:
          return "2 Iron";
        case 0x0053:
          return "3 Iron";
        case 0x0054:
          return "4 Iron";
        case 0x0055:
          return "5 Iron";
        case 0x0056:
          return "6 Iron";
        case 0x0057:
          return "7 Iron";
        case 0x0058:
          return "8 Iron";
        case 0x0059:
          return "9 Iron";
        case 0x005a:
          return "10 Iron";
        case 0x005b:
          return "11 Iron";
        case 0x005c:
          return "Sand Wedge";
        case 0x005d:
          return "Loft Wedge";
        case 0x005e:
          return "Power Wedge";
        case 0x005f:
          return "1 Wood";
        case 0x0060:
          return "3 Wood";
        case 0x0061:
          return "5 Wood";
        case 0x0062:
          return "7 Wood";
        case 0x0063:
          return "9 Wood";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class sport : std::uint8_t {
  BASEBALL_BAT = 0x0001,
  GOLF_CLUB = 0x0002,
  ROWING_MACHINE = 0x0003,
  TREADMILL = 0x0004,
  OAR = 0x0030,
  SLOPE = 0x0031,
  RATE = 0x0032,
  STICK_SPEED = 0x0033,
  STICK_FACE_ANGLE = 0x0034,
  STICK_HEEL_TOE = 0x0035,
  STICK_FOLLOW_THROUGH = 0x0036,
  STICK_TEMPO = 0x0037,
  STICK_TYPE = 0x0038,
  STICK_HEIGHT = 0x0039,
  PUTTER = 0x0050,
  _1_IRON = 0x0051,
  _2_IRON = 0x0052,
  _3_IRON = 0x0053,
  _4_IRON = 0x0054,
  _5_IRON = 0x0055,
  _6_IRON = 0x0056,
  _7_IRON = 0x0057,
  _8_IRON = 0x0058,
  _9_IRON = 0x0059,
  _10_IRON = 0x005a,
  _11_IRON = 0x005b,
  SAND_WEDGE = 0x005c,
  LOFT_WEDGE = 0x005d,
  POWER_WEDGE = 0x005e,
  _1_WOOD = 0x005f,
  _3_WOOD = 0x0060,
  _5_WOOD = 0x0061,
  _7_WOOD = 0x0062,
  _9_WOOD = 0x0063,
};
} // namespace hid::page

#endif // __HID_PAGE_SPORT_HPP_

#ifndef __HID_PAGE_SPORT_HPP_
#define __HID_PAGE_SPORT_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class sport : std::uint8_t;
template <> struct info<sport> {
  constexpr static page_id_t page_id = 0x0004;
  constexpr static usage_id_t max_usage_id = 0x0063;
  constexpr static const char *name = "Sport Controls";
};
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

#ifndef __HID_PAGE_VESA_VIRTUAL_HPP_
#define __HID_PAGE_VESA_VIRTUAL_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class vesa_virtual : std::uint8_t;
template <> struct info<vesa_virtual> {
  constexpr static page_id_t page_id = 0x0082;
  constexpr static usage_id_t max_usage_id = 0x00d4;
  constexpr static const char *name = "VESA Virtual Controls";
};
enum class vesa_virtual : std::uint8_t {
  DEGAUSS = 0x0001,
  BRIGHTNESS = 0x0010,
  CONTRAST = 0x0012,
  RED_VIDEO_GAIN = 0x0016,
  GREEN_VIDEO_GAIN = 0x0018,
  BLUE_VIDEO_GAIN = 0x001a,
  FOCUS = 0x001c,
  HORIZONTAL_POSITION = 0x0020,
  HORIZONTAL_SIZE = 0x0022,
  HORIZONTAL_PINCUSHION = 0x0024,
  HORIZONTAL_PINCUSHION_BALANCE = 0x0026,
  HORIZONTAL_MISCONVERGENCE = 0x0028,
  HORIZONTAL_LINEARITY = 0x002a,
  HORIZONTAL_LINEARITY_BALANCE = 0x002c,
  VERTICAL_POSITION = 0x0030,
  VERTICAL_SIZE = 0x0032,
  VERTICAL_PINCUSHION = 0x0034,
  VERTICAL_PINCUSHION_BALANCE = 0x0036,
  VERTICAL_MISCONVERGENCE = 0x0038,
  VERTICAL_LINEARITY = 0x003a,
  VERTICAL_LINEARITY_BALANCE = 0x003c,
  PARALLELOGRAM_DISTORTION_KEY_BALANCE = 0x0040,
  TRAPEZOIDAL_DISTORTION_KEY = 0x0042,
  TILT_ROTATION = 0x0044,
  TOP_CORNER_DISTORTION_CONTROL = 0x0046,
  TOP_CORNER_DISTORTION_BALANCE = 0x0048,
  BOTTOM_CORNER_DISTORTION_CONTROL = 0x004a,
  BOTTOM_CORNER_DISTORTION_BALANCE = 0x004c,
  HORIZONTAL_MOIRE = 0x0056,
  VERTICAL_MOIRE = 0x0058,
  INPUT_LEVEL_SELECT = 0x005e,
  INPUT_SOURCE_SELECT = 0x0060,
  RED_VIDEO_BLACK_LEVEL = 0x006c,
  GREEN_VIDEO_BLACK_LEVEL = 0x006e,
  BLUE_VIDEO_BLACK_LEVEL = 0x0070,
  AUTO_SIZE_CENTER = 0x00a2,
  POLARITY_HORIZONTAL_SYNCHRONIZATION = 0x00a4,
  POLARITY_VERTICAL_SYNCHRONIZATION = 0x00a6,
  SYNCHRONIZATION_TYPE = 0x00a8,
  SCREEN_ORIENTATION = 0x00aa,
  HORIZONTAL_FREQUENCY = 0x00ac,
  VERTICAL_FREQUENCY = 0x00ae,
  SETTINGS = 0x00b0,
  ON_SCREEN_DISPLAY = 0x00ca,
  STEREOMODE = 0x00d4,
};
} // namespace hid::page

#endif // __HID_PAGE_VESA_VIRTUAL_HPP_

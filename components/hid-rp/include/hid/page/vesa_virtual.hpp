#ifndef __HID_PAGE_VESA_VIRTUAL_HPP_
#define __HID_PAGE_VESA_VIRTUAL_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class vesa_virtual : std::uint8_t;
template <> constexpr inline auto get_info<vesa_virtual>() {
  return info(
      0x0082, 0x00d4, "VESA Virtual Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Degauss";
        case 0x0010:
          return "Brightness";
        case 0x0012:
          return "Contrast";
        case 0x0016:
          return "Red Video Gain";
        case 0x0018:
          return "Green Video Gain";
        case 0x001a:
          return "Blue Video Gain";
        case 0x001c:
          return "Focus";
        case 0x0020:
          return "Horizontal Position";
        case 0x0022:
          return "Horizontal Size";
        case 0x0024:
          return "Horizontal Pincushion";
        case 0x0026:
          return "Horizontal Pincushion Balance";
        case 0x0028:
          return "Horizontal Misconvergence";
        case 0x002a:
          return "Horizontal Linearity";
        case 0x002c:
          return "Horizontal Linearity Balance";
        case 0x0030:
          return "Vertical Position";
        case 0x0032:
          return "Vertical Size";
        case 0x0034:
          return "Vertical Pincushion";
        case 0x0036:
          return "Vertical Pincushion Balance";
        case 0x0038:
          return "Vertical Misconvergence";
        case 0x003a:
          return "Vertical Linearity";
        case 0x003c:
          return "Vertical Linearity Balance";
        case 0x0040:
          return "Parallelogram Distortion (Key Balance)";
        case 0x0042:
          return "Trapezoidal Distortion (Key)";
        case 0x0044:
          return "Tilt (Rotation)";
        case 0x0046:
          return "Top Corner Distortion Control";
        case 0x0048:
          return "Top Corner Distortion Balance";
        case 0x004a:
          return "Bottom Corner Distortion Control";
        case 0x004c:
          return "Bottom Corner Distortion Balance";
        case 0x0056:
          return "Horizontal Moire";
        case 0x0058:
          return "Vertical Moire";
        case 0x005e:
          return "Input Level Select";
        case 0x0060:
          return "Input Source Select";
        case 0x006c:
          return "Red Video Black Level";
        case 0x006e:
          return "Green Video Black Level";
        case 0x0070:
          return "Blue Video Black Level";
        case 0x00a2:
          return "Auto Size Center";
        case 0x00a4:
          return "Polarity Horizontal Synchronization";
        case 0x00a6:
          return "Polarity Vertical Synchronization";
        case 0x00a8:
          return "Synchronization Type";
        case 0x00aa:
          return "Screen Orientation";
        case 0x00ac:
          return "Horizontal Frequency";
        case 0x00ae:
          return "Vertical Frequency";
        case 0x00b0:
          return "Settings";
        case 0x00ca:
          return "On Screen Display";
        case 0x00d4:
          return "StereoMode";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
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

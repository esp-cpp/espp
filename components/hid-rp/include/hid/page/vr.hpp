#ifndef __HID_PAGE_VR_HPP_
#define __HID_PAGE_VR_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class vr : std::uint8_t;
template <> constexpr inline auto get_info<vr>() {
  return info(
      0x0003, 0x0021, "VR Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Belt";
        case 0x0002:
          return "Body Suit";
        case 0x0003:
          return "Flexor";
        case 0x0004:
          return "Glove";
        case 0x0005:
          return "Head Tracker";
        case 0x0006:
          return "Head Mounted Display";
        case 0x0007:
          return "Hand Tracker";
        case 0x0008:
          return "Oculometer";
        case 0x0009:
          return "Vest";
        case 0x000a:
          return "Animatronic Device";
        case 0x0020:
          return "Stereo Enable";
        case 0x0021:
          return "Display Enable";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class vr : std::uint8_t {
  BELT = 0x0001,
  BODY_SUIT = 0x0002,
  FLEXOR = 0x0003,
  GLOVE = 0x0004,
  HEAD_TRACKER = 0x0005,
  HEAD_MOUNTED_DISPLAY = 0x0006,
  HAND_TRACKER = 0x0007,
  OCULOMETER = 0x0008,
  VEST = 0x0009,
  ANIMATRONIC_DEVICE = 0x000a,
  STEREO_ENABLE = 0x0020,
  DISPLAY_ENABLE = 0x0021,
};
} // namespace hid::page

#endif // __HID_PAGE_VR_HPP_

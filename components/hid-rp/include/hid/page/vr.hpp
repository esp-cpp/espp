#ifndef __HID_PAGE_VR_HPP_
#define __HID_PAGE_VR_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class vr : std::uint8_t;
template <> struct info<vr> {
  constexpr static page_id_t page_id = 0x0003;
  constexpr static usage_id_t max_usage_id = 0x0021;
  constexpr static const char *name = "VR Controls";
};
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

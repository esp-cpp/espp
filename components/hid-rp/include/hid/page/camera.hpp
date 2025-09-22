#ifndef __HID_PAGE_CAMERA_HPP_
#define __HID_PAGE_CAMERA_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class camera : std::uint8_t;
template <> struct info<camera> {
  constexpr static page_id_t page_id = 0x0090;
  constexpr static usage_id_t max_usage_id = 0x0021;
  constexpr static const char *name = "Camera Control";
};
enum class camera : std::uint8_t {
  CAMERA_AUTO_FOCUS = 0x0020,
  CAMERA_SHUTTER = 0x0021,
};
} // namespace hid::page

#endif // __HID_PAGE_CAMERA_HPP_

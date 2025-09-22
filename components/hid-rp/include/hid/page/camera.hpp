#ifndef __HID_PAGE_CAMERA_HPP_
#define __HID_PAGE_CAMERA_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class camera : std::uint8_t;
template <> constexpr inline auto get_info<camera>() {
  return info(
      0x0090, 0x0021, "Camera Control",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0020:
          return "Camera Auto-focus";
        case 0x0021:
          return "Camera Shutter";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class camera : std::uint8_t {
  CAMERA_AUTO_FOCUS = 0x0020,
  CAMERA_SHUTTER = 0x0021,
};
} // namespace hid::page

#endif // __HID_PAGE_CAMERA_HPP_

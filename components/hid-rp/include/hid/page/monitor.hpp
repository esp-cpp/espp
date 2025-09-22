#ifndef __HID_PAGE_MONITOR_HPP_
#define __HID_PAGE_MONITOR_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class monitor : std::uint8_t;
template <> constexpr inline auto get_info<monitor>() {
  return info(
      0x0080, 0x0004, "Monitor",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Monitor Control";
        case 0x0002:
          return "EDID Information";
        case 0x0003:
          return "VDIF Information";
        case 0x0004:
          return "VESA Version";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class monitor : std::uint8_t {
  MONITOR_CONTROL = 0x0001,
  EDID_INFORMATION = 0x0002,
  VDIF_INFORMATION = 0x0003,
  VESA_VERSION = 0x0004,
};
} // namespace hid::page

#endif // __HID_PAGE_MONITOR_HPP_

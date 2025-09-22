#ifndef __HID_PAGE_MONITOR_HPP_
#define __HID_PAGE_MONITOR_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class monitor : std::uint8_t;
template <> struct info<monitor> {
  constexpr static page_id_t page_id = 0x0080;
  constexpr static usage_id_t max_usage_id = 0x0004;
  constexpr static const char *name = "Monitor";
};
enum class monitor : std::uint8_t {
  MONITOR_CONTROL = 0x0001,
  EDID_INFORMATION = 0x0002,
  VDIF_INFORMATION = 0x0003,
  VESA_VERSION = 0x0004,
};
} // namespace hid::page

#endif // __HID_PAGE_MONITOR_HPP_

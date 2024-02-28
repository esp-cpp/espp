#ifndef __HID_PAGE_MONITOR_ENUMERATED_HPP_
#define __HID_PAGE_MONITOR_ENUMERATED_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class monitor_enumerated;
template <> struct info<monitor_enumerated> {
  constexpr static page_id_t page_id = 0x0081;
  constexpr static usage_id_t max_usage_id = 0x00ff;
  constexpr static const char *name = "Monitor Enumerated";
};
class monitor_enumerated {
public:
  constexpr operator usage_id_t() const { return id; }
  constexpr monitor_enumerated(std::uint8_t value)
      : id(value) {}
  std::uint8_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_MONITOR_ENUMERATED_HPP_

#ifndef __HID_PAGE_MONITOR_ENUMERATED_HPP_
#define __HID_PAGE_MONITOR_ENUMERATED_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class monitor_enumerated;
template <> constexpr inline auto get_info<monitor_enumerated>() {
  return info(0x0081, 0xffff, "Monitor Enumerated",
              [](hid::usage_id_t id) { return id ? "ENUM_{}" : nullptr; });
}
class monitor_enumerated {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr monitor_enumerated(std::uint16_t value)
      : id(value) {}
  std::uint16_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_MONITOR_ENUMERATED_HPP_

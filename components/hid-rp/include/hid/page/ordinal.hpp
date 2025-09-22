#ifndef __HID_PAGE_ORDINAL_HPP_
#define __HID_PAGE_ORDINAL_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class ordinal;
template <> constexpr inline auto get_info<ordinal>() {
  return info(0x000a, 0xffff, "Ordinal",
              [](hid::usage_id_t id) { return id ? "Instance {}" : nullptr; });
}
class ordinal {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr ordinal(std::uint16_t value)
      : id(value) {}
  std::uint16_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_ORDINAL_HPP_

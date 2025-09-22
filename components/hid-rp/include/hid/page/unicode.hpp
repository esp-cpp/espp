#ifndef __HID_PAGE_UNICODE_HPP_
#define __HID_PAGE_UNICODE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class unicode;
template <> constexpr inline auto get_info<unicode>() {
  return info(0x0010, 0x0000, "Unicode",
              [](hid::usage_id_t id) { return id ? "Unicode {}" : nullptr; });
}
class unicode {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr unicode(std::uint8_t value)
      : id(value) {}
  std::uint8_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_UNICODE_HPP_

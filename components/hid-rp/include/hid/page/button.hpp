#ifndef __HID_PAGE_BUTTON_HPP_
#define __HID_PAGE_BUTTON_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class button;
template <> constexpr inline auto get_info<button>() {
  return info(0x0009, 0xffff, "Button",
              [](hid::usage_id_t id) { return id ? "Button {}" : nullptr; });
}
class button {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr button(std::uint16_t value)
      : id(value) {}
  std::uint16_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_BUTTON_HPP_

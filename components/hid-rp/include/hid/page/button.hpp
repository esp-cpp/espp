#ifndef __HID_PAGE_BUTTON_HPP_
#define __HID_PAGE_BUTTON_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class button;
template <> struct info<button> {
  constexpr static page_id_t page_id = 0x0009;
  constexpr static usage_id_t max_usage_id = 0x00ff;
  constexpr static const char *name = "Button";
};
class button {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr button(std::uint8_t value)
      : id(value) {}
  std::uint8_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_BUTTON_HPP_

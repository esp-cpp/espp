#ifndef __HID_PAGE_ORDINAL_HPP_
#define __HID_PAGE_ORDINAL_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class ordinal;
template <> struct info<ordinal> {
  constexpr static page_id_t page_id = 0x000a;
  constexpr static usage_id_t max_usage_id = 0xffff;
  constexpr static const char *name = "Ordinal";
};
class ordinal {
public:
  constexpr operator usage_id_t() const { return id; }
  explicit constexpr ordinal(std::uint16_t value)
      : id(value) {}
  std::uint16_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_ORDINAL_HPP_

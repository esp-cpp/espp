#ifndef __HID_PAGE_UNICODE_HPP_
#define __HID_PAGE_UNICODE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
class unicode;
template <> struct info<unicode> {
  constexpr static page_id_t page_id = 0x0010;
  constexpr static usage_id_t max_usage_id = 0x0000;
  constexpr static const char *name = "Unicode";
};
class unicode {
public:
  constexpr operator usage_id_t() const { return id; }
  constexpr unicode(std::uint8_t value)
      : id(value) {}
  std::uint8_t id{};
};
} // namespace hid::page

#endif // __HID_PAGE_UNICODE_HPP_

#ifndef __HID_PAGE_FIDO_HPP_
#define __HID_PAGE_FIDO_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class fido : std::uint8_t;
template <> struct info<fido> {
  constexpr static page_id_t page_id = 0xf1d0;
  constexpr static usage_id_t max_usage_id = 0x0021;
  constexpr static const char *name = "FIDO Alliance";
};
enum class fido : std::uint8_t {
  U2F_AUTHENTICATOR_DEVICE = 0x0001,
  INPUT_REPORT_DATA = 0x0020,
  OUTPUT_REPORT_DATA = 0x0021,
};
} // namespace hid::page

#endif // __HID_PAGE_FIDO_HPP_

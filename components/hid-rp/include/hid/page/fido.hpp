#ifndef __HID_PAGE_FIDO_HPP_
#define __HID_PAGE_FIDO_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class fido : std::uint8_t;
template <> constexpr inline auto get_info<fido>() {
  return info(
      0xf1d0, 0x0021, "FIDO Alliance",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "U2F Authenticator Device";
        case 0x0020:
          return "Input Report Data";
        case 0x0021:
          return "Output Report Data";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class fido : std::uint8_t {
  U2F_AUTHENTICATOR_DEVICE = 0x0001,
  INPUT_REPORT_DATA = 0x0020,
  OUTPUT_REPORT_DATA = 0x0021,
};
} // namespace hid::page

#endif // __HID_PAGE_FIDO_HPP_

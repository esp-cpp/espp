#ifndef __HID_PAGE_MAGNETIC_STRIPE_READER_HPP_
#define __HID_PAGE_MAGNETIC_STRIPE_READER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class magnetic_stripe_reader : std::uint8_t;
template <> constexpr inline auto get_info<magnetic_stripe_reader>() {
  return info(
      0x008e, 0x0024, "Magnetic Stripe Reader",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "MSR Device Read-Only";
        case 0x0011:
          return "Track 1 Length";
        case 0x0012:
          return "Track 2 Length";
        case 0x0013:
          return "Track 3 Length";
        case 0x0014:
          return "Track JIS Length";
        case 0x0020:
          return "Track Data";
        case 0x0021:
          return "Track 1 Data";
        case 0x0022:
          return "Track 2 Data";
        case 0x0023:
          return "Track 3 Data";
        case 0x0024:
          return "Track JIS Data";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class magnetic_stripe_reader : std::uint8_t {
  MSR_DEVICE_READ_ONLY = 0x0001,
  TRACK_1_LENGTH = 0x0011,
  TRACK_2_LENGTH = 0x0012,
  TRACK_3_LENGTH = 0x0013,
  TRACK_JIS_LENGTH = 0x0014,
  TRACK_DATA = 0x0020,
  TRACK_1_DATA = 0x0021,
  TRACK_2_DATA = 0x0022,
  TRACK_3_DATA = 0x0023,
  TRACK_JIS_DATA = 0x0024,
};
} // namespace hid::page

#endif // __HID_PAGE_MAGNETIC_STRIPE_READER_HPP_

#ifndef __HID_PAGE_MAGNETIC_STRIPE_READER_HPP_
#define __HID_PAGE_MAGNETIC_STRIPE_READER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class magnetic_stripe_reader : std::uint8_t;
template <> struct info<magnetic_stripe_reader> {
  constexpr static page_id_t page_id = 0x008e;
  constexpr static usage_id_t max_usage_id = 0x0024;
  constexpr static const char *name = "Magnetic Stripe Reader";
};
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

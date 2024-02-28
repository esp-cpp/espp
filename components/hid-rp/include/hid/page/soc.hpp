#ifndef __HID_PAGE_SOC_HPP_
#define __HID_PAGE_SOC_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class soc : std::uint8_t;
template <> struct info<soc> {
  constexpr static page_id_t page_id = 0x0011;
  constexpr static usage_id_t max_usage_id = 0x000a;
  constexpr static const char *name = "SoC";
};
enum class soc : std::uint8_t {
  SOC_CONTROL = 0x0001,
  FIRMWARE_TRANSFER = 0x0002,
  FIRMWARE_FILE_ID = 0x0003,
  FILE_OFFSET_IN_BYTES = 0x0004,
  FILE_TRANSFER_SIZE_MAX_IN_BYTES = 0x0005,
  FILE_PAYLOAD = 0x0006,
  FILE_PAYLOAD_SIZE_IN_BYTES = 0x0007,
  FILE_PAYLOAD_CONTAINS_LAST_BYTES = 0x0008,
  FILE_TRANSFER_STOP = 0x0009,
  FILE_TRANSFER_TILL_END = 0x000a,
};
} // namespace hid::page

#endif // __HID_PAGE_SOC_HPP_

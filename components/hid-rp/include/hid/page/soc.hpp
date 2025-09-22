#ifndef __HID_PAGE_SOC_HPP_
#define __HID_PAGE_SOC_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class soc : std::uint8_t;
template <> constexpr inline auto get_info<soc>() {
  return info(
      0x0011, 0x000a, "SoC",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "SoC Control";
        case 0x0002:
          return "Firmware Transfer";
        case 0x0003:
          return "Firmware File Id";
        case 0x0004:
          return "File Offset In Bytes";
        case 0x0005:
          return "File Transfer Size Max In Bytes";
        case 0x0006:
          return "File Payload";
        case 0x0007:
          return "File Payload Size In Bytes";
        case 0x0008:
          return "File Payload Contains Last Bytes";
        case 0x0009:
          return "File Transfer Stop";
        case 0x000a:
          return "File Transfer Till End";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
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

#ifndef __HID_PAGE_GENERIC_DEVICE_HPP_
#define __HID_PAGE_GENERIC_DEVICE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class generic_device : std::uint8_t;
template <> struct info<generic_device> {
  constexpr static page_id_t page_id = 0x0006;
  constexpr static usage_id_t max_usage_id = 0x0041;
  constexpr static const char *name = "Generic Device Controls";
};
enum class generic_device : std::uint8_t {
  BACKGROUND_NONUSER_CONTROLS = 0x0001,
  BATTERY_STRENGTH = 0x0020,
  WIRELESS_CHANNEL = 0x0021,
  WIRELESS_ID = 0x0022,
  DISCOVER_WIRELESS_CONTROL = 0x0023,
  SECURITY_CODE_CHARACTER_ENTERED = 0x0024,
  SECURITY_CODE_CHARACTER_ERASED = 0x0025,
  SECURITY_CODE_CLEARED = 0x0026,
  SEQUENCE_ID = 0x0027,
  SEQUENCE_ID_RESET = 0x0028,
  RF_SIGNAL_STRENGTH = 0x0029,
  SOFTWARE_VERSION = 0x002a,
  PROTOCOL_VERSION = 0x002b,
  HARDWARE_VERSION = 0x002c,
  MAJOR = 0x002d,
  MINOR = 0x002e,
  REVISION = 0x002f,
  HANDEDNESS = 0x0030,
  EITHER_HAND = 0x0031,
  LEFT_HAND = 0x0032,
  RIGHT_HAND = 0x0033,
  BOTH_HANDS = 0x0034,
  GRIP_POSE_OFFSET = 0x0040,
  POINTER_POSE_OFFSET = 0x0041,
};
} // namespace hid::page

#endif // __HID_PAGE_GENERIC_DEVICE_HPP_

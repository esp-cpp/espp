#ifndef __HID_PAGE_GENERIC_DEVICE_HPP_
#define __HID_PAGE_GENERIC_DEVICE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class generic_device : std::uint8_t;
template <> constexpr inline auto get_info<generic_device>() {
  return info(
      0x0006, 0x0041, "Generic Device Controls",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Background/Nonuser Controls";
        case 0x0020:
          return "Battery Strength";
        case 0x0021:
          return "Wireless Channel";
        case 0x0022:
          return "Wireless ID";
        case 0x0023:
          return "Discover Wireless Control";
        case 0x0024:
          return "Security Code Character Entered";
        case 0x0025:
          return "Security Code Character Erased";
        case 0x0026:
          return "Security Code Cleared";
        case 0x0027:
          return "Sequence ID";
        case 0x0028:
          return "Sequence ID Reset";
        case 0x0029:
          return "RF Signal Strength";
        case 0x002a:
          return "Software Version";
        case 0x002b:
          return "Protocol Version";
        case 0x002c:
          return "Hardware Version";
        case 0x002d:
          return "Major";
        case 0x002e:
          return "Minor";
        case 0x002f:
          return "Revision";
        case 0x0030:
          return "Handedness";
        case 0x0031:
          return "Either Hand";
        case 0x0032:
          return "Left Hand";
        case 0x0033:
          return "Right Hand";
        case 0x0034:
          return "Both Hands";
        case 0x0040:
          return "Grip Pose Offset";
        case 0x0041:
          return "Pointer Pose Offset";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
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

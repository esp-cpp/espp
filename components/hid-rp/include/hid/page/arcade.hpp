#ifndef __HID_PAGE_ARCADE_HPP_
#define __HID_PAGE_ARCADE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class arcade : std::uint8_t;
template <> constexpr inline auto get_info<arcade>() {
  return info(
      0x0091, 0x004d, "Arcade",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "General Purpose IO Card";
        case 0x0002:
          return "Coin Door";
        case 0x0003:
          return "Watchdog Timer";
        case 0x0030:
          return "General Purpose Analog Input State";
        case 0x0031:
          return "General Purpose Digital Input State";
        case 0x0032:
          return "General Purpose Optical Input State";
        case 0x0033:
          return "General Purpose Digital Output State";
        case 0x0034:
          return "Number of Coin Doors";
        case 0x0035:
          return "Coin Drawer Drop Count";
        case 0x0036:
          return "Coin Drawer Start";
        case 0x0037:
          return "Coin Drawer Service";
        case 0x0038:
          return "Coin Drawer Tilt";
        case 0x0039:
          return "Coin Door Test";
        case 0x0040:
          return "Coin Door Lockout";
        case 0x0041:
          return "Watchdog Timeout";
        case 0x0042:
          return "Watchdog Action";
        case 0x0043:
          return "Watchdog Reboot";
        case 0x0044:
          return "Watchdog Restart";
        case 0x0045:
          return "Alarm Input";
        case 0x0046:
          return "Coin Door Counter";
        case 0x0047:
          return "I/O Direction Mapping";
        case 0x0048:
          return "Set I/O Direction Mapping";
        case 0x0049:
          return "Extended Optical Input State";
        case 0x004a:
          return "Pin Pad Input State";
        case 0x004b:
          return "Pin Pad Status";
        case 0x004c:
          return "Pin Pad Output";
        case 0x004d:
          return "Pin Pad Command";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class arcade : std::uint8_t {
  GENERAL_PURPOSE_IO_CARD = 0x0001,
  COIN_DOOR = 0x0002,
  WATCHDOG_TIMER = 0x0003,
  GENERAL_PURPOSE_ANALOG_INPUT_STATE = 0x0030,
  GENERAL_PURPOSE_DIGITAL_INPUT_STATE = 0x0031,
  GENERAL_PURPOSE_OPTICAL_INPUT_STATE = 0x0032,
  GENERAL_PURPOSE_DIGITAL_OUTPUT_STATE = 0x0033,
  NUMBER_OF_COIN_DOORS = 0x0034,
  COIN_DRAWER_DROP_COUNT = 0x0035,
  COIN_DRAWER_START = 0x0036,
  COIN_DRAWER_SERVICE = 0x0037,
  COIN_DRAWER_TILT = 0x0038,
  COIN_DOOR_TEST = 0x0039,
  COIN_DOOR_LOCKOUT = 0x0040,
  WATCHDOG_TIMEOUT = 0x0041,
  WATCHDOG_ACTION = 0x0042,
  WATCHDOG_REBOOT = 0x0043,
  WATCHDOG_RESTART = 0x0044,
  ALARM_INPUT = 0x0045,
  COIN_DOOR_COUNTER = 0x0046,
  I_O_DIRECTION_MAPPING = 0x0047,
  SET_I_O_DIRECTION_MAPPING = 0x0048,
  EXTENDED_OPTICAL_INPUT_STATE = 0x0049,
  PIN_PAD_INPUT_STATE = 0x004a,
  PIN_PAD_STATUS = 0x004b,
  PIN_PAD_OUTPUT = 0x004c,
  PIN_PAD_COMMAND = 0x004d,
};
} // namespace hid::page

#endif // __HID_PAGE_ARCADE_HPP_

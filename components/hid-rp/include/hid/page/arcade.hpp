#ifndef __HID_PAGE_ARCADE_HPP_
#define __HID_PAGE_ARCADE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class arcade : std::uint8_t;
template <> struct info<arcade> {
  constexpr static page_id_t page_id = 0x0091;
  constexpr static usage_id_t max_usage_id = 0x004d;
  constexpr static const char *name = "Arcade";
};
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

#ifndef __HID_PAGE_BATTERY_SYSTEM_HPP_
#define __HID_PAGE_BATTERY_SYSTEM_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class battery_system : std::uint8_t;
template <> struct info<battery_system> {
  constexpr static page_id_t page_id = 0x0085;
  constexpr static usage_id_t max_usage_id = 0x00f3;
  constexpr static const char *name = "Battery System";
};
enum class battery_system : std::uint8_t {
  SMART_BATTERY_BATTERY_MODE = 0x0001,
  SMART_BATTERY_BATTERY_STATUS = 0x0002,
  SMART_BATTERY_ALARM_WARNING = 0x0003,
  SMART_BATTERY_CHARGER_MODE = 0x0004,
  SMART_BATTERY_CHARGER_STATUS = 0x0005,
  SMART_BATTERY_CHARGER_SPEC_INFO = 0x0006,
  SMART_BATTERY_SELECTOR_STATE = 0x0007,
  SMART_BATTERY_SELECTOR_PRESETS = 0x0008,
  SMART_BATTERY_SELECTOR_INFO = 0x0009,
  OPTIONAL_MFG_FUNCTION_1 = 0x0010,
  OPTIONAL_MFG_FUNCTION_2 = 0x0011,
  OPTIONAL_MFG_FUNCTION_3 = 0x0012,
  OPTIONAL_MFG_FUNCTION_4 = 0x0013,
  OPTIONAL_MFG_FUNCTION_5 = 0x0014,
  CONNECTION_TO_SM_BUS = 0x0015,
  OUTPUT_CONNECTION = 0x0016,
  CHARGER_CONNECTION = 0x0017,
  BATTERY_INSERTION = 0x0018,
  USE_NEXT = 0x0019,
  OK_TO_USE = 0x001a,
  BATTERY_SUPPORTED = 0x001b,
  SELECTOR_REVISION = 0x001c,
  CHARGING_INDICATOR = 0x001d,
  MANUFACTURER_ACCESS = 0x0028,
  REMAINING_CAPACITY_LIMIT = 0x0029,
  REMAINING_TIME_LIMIT = 0x002a,
  AT_RATE = 0x002b,
  CAPACITY_MODE = 0x002c,
  BROADCAST_TO_CHARGER = 0x002d,
  PRIMARY_BATTERY = 0x002e,
  CHARGE_CONTROLLER = 0x002f,
  TERMINATE_CHARGE = 0x0040,
  TERMINATE_DISCHARGE = 0x0041,
  BELOW_REMAINING_CAPACITY_LIMIT = 0x0042,
  REMAINING_TIME_LIMIT_EXPIRED = 0x0043,
  CHARGING = 0x0044,
  DISCHARGING = 0x0045,
  FULLY_CHARGED = 0x0046,
  FULLY_DISCHARGED = 0x0047,
  CONDITIONING_FLAG = 0x0048,
  AT_RATE_OK = 0x0049,
  SMART_BATTERY_ERROR_CODE = 0x004a,
  NEED_REPLACEMENT = 0x004b,
  AT_RATE_TIME_TO_FULL = 0x0060,
  AT_RATE_TIME_TO_EMPTY = 0x0061,
  AVERAGE_CURRENT = 0x0062,
  MAX_ERROR = 0x0063,
  RELATIVE_STATE_OF_CHARGE = 0x0064,
  ABSOLUTE_STATE_OF_CHARGE = 0x0065,
  REMAINING_CAPACITY = 0x0066,
  FULL_CHARGE_CAPACITY = 0x0067,
  RUN_TIME_TO_EMPTY = 0x0068,
  AVERAGE_TIME_TO_EMPTY = 0x0069,
  AVERAGE_TIME_TO_FULL = 0x006a,
  CYCLE_COUNT = 0x006b,
  BATTERY_PACK_MODEL_LEVEL = 0x0080,
  INTERNAL_CHARGE_CONTROLLER = 0x0081,
  PRIMARY_BATTERY_SUPPORT = 0x0082,
  DESIGN_CAPACITY = 0x0083,
  SPECIFICATION_INFO = 0x0084,
  MANUFACTURE_DATE = 0x0085,
  SERIAL_NUMBER = 0x0086,
  IMANUFACTURER_NAME = 0x0087,
  IDEVICE_NAME = 0x0088,
  IDEVICE_CHEMISTRY = 0x0089,
  MANUFACTURER_DATA = 0x008a,
  RECHARGABLE = 0x008b,
  WARNING_CAPACITY_LIMIT = 0x008c,
  CAPACITY_GRANULARITY_1 = 0x008d,
  CAPACITY_GRANULARITY_2 = 0x008e,
  IOEM_INFORMATION = 0x008f,
  INHIBIT_CHARGE = 0x00c0,
  ENABLE_POLLING = 0x00c1,
  RESET_TO_ZERO = 0x00c2,
  AC_PRESENT = 0x00d0,
  BATTERY_PRESENT = 0x00d1,
  POWER_FAIL = 0x00d2,
  ALARM_INHIBITED = 0x00d3,
  THERMISTOR_UNDER_RANGE = 0x00d4,
  THERMISTOR_HOT = 0x00d5,
  THERMISTOR_COLD = 0x00d6,
  THERMISTOR_OVER_RANGE = 0x00d7,
  VOLTAGE_OUT_OF_RANGE = 0x00d8,
  CURRENT_OUT_OF_RANGE = 0x00d9,
  CURRENT_NOT_REGULATED = 0x00da,
  VOLTAGE_NOT_REGULATED = 0x00db,
  MASTER_MODE = 0x00dc,
  CHARGER_SELECTOR_SUPPORT = 0x00f0,
  CHARGER_SPEC = 0x00f1,
  LEVEL_2 = 0x00f2,
  LEVEL_3 = 0x00f3,
};
} // namespace hid::page

#endif // __HID_PAGE_BATTERY_SYSTEM_HPP_

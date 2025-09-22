#ifndef __HID_PAGE_POWER_HPP_
#define __HID_PAGE_POWER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class power : std::uint8_t;
template <> constexpr inline auto get_info<power>() {
  return info(
      0x0084, 0x00ff, "Power",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "iName";
        case 0x0002:
          return "Present Status";
        case 0x0003:
          return "Changed Status";
        case 0x0004:
          return "UPS";
        case 0x0005:
          return "Power Supply";
        case 0x0010:
          return "Battery System";
        case 0x0011:
          return "Battery System Id";
        case 0x0012:
          return "Battery";
        case 0x0013:
          return "Battery Id";
        case 0x0014:
          return "Charger";
        case 0x0015:
          return "Charger Id";
        case 0x0016:
          return "Power Converter";
        case 0x0017:
          return "Power Converter Id";
        case 0x0018:
          return "Outlet System";
        case 0x0019:
          return "Outlet System Id";
        case 0x001a:
          return "Input";
        case 0x001b:
          return "Input Id";
        case 0x001c:
          return "Output";
        case 0x001d:
          return "Output Id";
        case 0x001e:
          return "Flow";
        case 0x001f:
          return "Flow Id";
        case 0x0020:
          return "Outlet";
        case 0x0021:
          return "Outlet Id";
        case 0x0022:
          return "Gang";
        case 0x0023:
          return "Gang Id";
        case 0x0024:
          return "Power Summary";
        case 0x0025:
          return "Power Summary Id";
        case 0x0030:
          return "Voltage";
        case 0x0031:
          return "Current";
        case 0x0032:
          return "Frequency";
        case 0x0033:
          return "Apparent Power";
        case 0x0034:
          return "Active Power";
        case 0x0035:
          return "Percent Load";
        case 0x0036:
          return "Temperature";
        case 0x0037:
          return "Humidity";
        case 0x0038:
          return "Bad Count";
        case 0x0040:
          return "Config Voltage";
        case 0x0041:
          return "Config Current";
        case 0x0042:
          return "Config Frequency";
        case 0x0043:
          return "Config Apparent Power";
        case 0x0044:
          return "Config Active Power";
        case 0x0045:
          return "Config Percent Load";
        case 0x0046:
          return "Config Temperature";
        case 0x0047:
          return "Config Humidity";
        case 0x0050:
          return "Switch On Control";
        case 0x0051:
          return "Switch Off Control";
        case 0x0052:
          return "Toggle Control";
        case 0x0053:
          return "Low Voltage Transfer";
        case 0x0054:
          return "High Voltage Transfer";
        case 0x0055:
          return "Delay Before Reboot";
        case 0x0056:
          return "Delay Before Startup";
        case 0x0057:
          return "Delay Before Shutdown";
        case 0x0058:
          return "Test";
        case 0x0059:
          return "Module Reset";
        case 0x005a:
          return "Audible Alarm Control";
        case 0x0060:
          return "Present";
        case 0x0061:
          return "Good";
        case 0x0062:
          return "Internal Failure";
        case 0x0063:
          return "Voltage Out Of Range";
        case 0x0064:
          return "Frequency Out Of Range";
        case 0x0065:
          return "Overload";
        case 0x0066:
          return "Overcharged";
        case 0x0067:
          return "Over Temperature";
        case 0x0068:
          return "Shutdown Requested";
        case 0x0069:
          return "Shutdown Imminent";
        case 0x006b:
          return "Switch On/Off";
        case 0x006c:
          return "Switchable";
        case 0x006d:
          return "Used";
        case 0x006e:
          return "Boost";
        case 0x006f:
          return "Buck";
        case 0x0070:
          return "Initialized";
        case 0x0071:
          return "Tested";
        case 0x0072:
          return "Awaiting Power";
        case 0x0073:
          return "Communication Lost";
        case 0x00fd:
          return "iManufacturer";
        case 0x00fe:
          return "iProduct";
        case 0x00ff:
          return "iSerialNumber";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class power : std::uint8_t {
  INAME = 0x0001,
  PRESENT_STATUS = 0x0002,
  CHANGED_STATUS = 0x0003,
  UPS = 0x0004,
  POWER_SUPPLY = 0x0005,
  BATTERY_SYSTEM = 0x0010,
  BATTERY_SYSTEM_ID = 0x0011,
  BATTERY = 0x0012,
  BATTERY_ID = 0x0013,
  CHARGER = 0x0014,
  CHARGER_ID = 0x0015,
  POWER_CONVERTER = 0x0016,
  POWER_CONVERTER_ID = 0x0017,
  OUTLET_SYSTEM = 0x0018,
  OUTLET_SYSTEM_ID = 0x0019,
  INPUT = 0x001a,
  INPUT_ID = 0x001b,
  OUTPUT = 0x001c,
  OUTPUT_ID = 0x001d,
  FLOW = 0x001e,
  FLOW_ID = 0x001f,
  OUTLET = 0x0020,
  OUTLET_ID = 0x0021,
  GANG = 0x0022,
  GANG_ID = 0x0023,
  POWER_SUMMARY = 0x0024,
  POWER_SUMMARY_ID = 0x0025,
  VOLTAGE = 0x0030,
  CURRENT = 0x0031,
  FREQUENCY = 0x0032,
  APPARENT_POWER = 0x0033,
  ACTIVE_POWER = 0x0034,
  PERCENT_LOAD = 0x0035,
  TEMPERATURE = 0x0036,
  HUMIDITY = 0x0037,
  BAD_COUNT = 0x0038,
  CONFIG_VOLTAGE = 0x0040,
  CONFIG_CURRENT = 0x0041,
  CONFIG_FREQUENCY = 0x0042,
  CONFIG_APPARENT_POWER = 0x0043,
  CONFIG_ACTIVE_POWER = 0x0044,
  CONFIG_PERCENT_LOAD = 0x0045,
  CONFIG_TEMPERATURE = 0x0046,
  CONFIG_HUMIDITY = 0x0047,
  SWITCH_ON_CONTROL = 0x0050,
  SWITCH_OFF_CONTROL = 0x0051,
  TOGGLE_CONTROL = 0x0052,
  LOW_VOLTAGE_TRANSFER = 0x0053,
  HIGH_VOLTAGE_TRANSFER = 0x0054,
  DELAY_BEFORE_REBOOT = 0x0055,
  DELAY_BEFORE_STARTUP = 0x0056,
  DELAY_BEFORE_SHUTDOWN = 0x0057,
  TEST = 0x0058,
  MODULE_RESET = 0x0059,
  AUDIBLE_ALARM_CONTROL = 0x005a,
  PRESENT = 0x0060,
  GOOD = 0x0061,
  INTERNAL_FAILURE = 0x0062,
  VOLTAGE_OUT_OF_RANGE = 0x0063,
  FREQUENCY_OUT_OF_RANGE = 0x0064,
  OVERLOAD = 0x0065,
  OVERCHARGED = 0x0066,
  OVER_TEMPERATURE = 0x0067,
  SHUTDOWN_REQUESTED = 0x0068,
  SHUTDOWN_IMMINENT = 0x0069,
  SWITCH_ON_OFF = 0x006b,
  SWITCHABLE = 0x006c,
  USED = 0x006d,
  BOOST = 0x006e,
  BUCK = 0x006f,
  INITIALIZED = 0x0070,
  TESTED = 0x0071,
  AWAITING_POWER = 0x0072,
  COMMUNICATION_LOST = 0x0073,
  IMANUFACTURER = 0x00fd,
  IPRODUCT = 0x00fe,
  ISERIALNUMBER = 0x00ff,
};
} // namespace hid::page

#endif // __HID_PAGE_POWER_HPP_

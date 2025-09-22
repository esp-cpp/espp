#ifndef __HID_PAGE_SCALE_HPP_
#define __HID_PAGE_SCALE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class scale : std::uint8_t;
template <> constexpr inline auto get_info<scale>() {
  return info(
      0x008d, 0x0081, "Scales",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Scales";
        case 0x0020:
          return "Scale Device";
        case 0x0021:
          return "Scale Class";
        case 0x0022:
          return "Scale Class I Metric";
        case 0x0023:
          return "Scale Class II Metric";
        case 0x0024:
          return "Scale Class III Metric";
        case 0x0025:
          return "Scale Class IIIL Metric";
        case 0x0026:
          return "Scale Class IV Metric";
        case 0x0027:
          return "Scale Class III English";
        case 0x0028:
          return "Scale Class IIIL English";
        case 0x0029:
          return "Scale Class IV English";
        case 0x002a:
          return "Scale Class Generic";
        case 0x0030:
          return "Scale Attribute Report";
        case 0x0031:
          return "Scale Control Report";
        case 0x0032:
          return "Scale Data Report";
        case 0x0033:
          return "Scale Status Report";
        case 0x0034:
          return "Scale Weight Limit Report";
        case 0x0035:
          return "Scale Statistics Report";
        case 0x0040:
          return "Data Weight";
        case 0x0041:
          return "Data Scaling";
        case 0x0050:
          return "Weight Unit";
        case 0x0051:
          return "Weight Unit Milligram";
        case 0x0052:
          return "Weight Unit Gram";
        case 0x0053:
          return "Weight Unit Kilogram";
        case 0x0054:
          return "Weight Unit Carats";
        case 0x0055:
          return "Weight Unit Taels";
        case 0x0056:
          return "Weight Unit Grains";
        case 0x0057:
          return "Weight Unit Pennyweights";
        case 0x0058:
          return "Weight Unit Metric Ton";
        case 0x0059:
          return "Weight Unit Avoir Ton";
        case 0x005a:
          return "Weight Unit Troy Ounce";
        case 0x005b:
          return "Weight Unit Ounce";
        case 0x005c:
          return "Weight Unit Pound";
        case 0x0060:
          return "Calibration Count";
        case 0x0061:
          return "Re-Zero Count";
        case 0x0070:
          return "Scale Status";
        case 0x0071:
          return "Scale Status Fault";
        case 0x0072:
          return "Scale Status Stable at Center of Zero";
        case 0x0073:
          return "Scale Status In Motion";
        case 0x0074:
          return "Scale Status Weight Stable";
        case 0x0075:
          return "Scale Status Under Zero";
        case 0x0076:
          return "Scale Status Over Weight Limit";
        case 0x0077:
          return "Scale Status Requires Calibration";
        case 0x0078:
          return "Scale Status Requires Rezeroing";
        case 0x0080:
          return "Zero Scale";
        case 0x0081:
          return "Enforced Zero Return";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class scale : std::uint8_t {
  SCALES = 0x0001,
  SCALE_DEVICE = 0x0020,
  SCALE_CLASS = 0x0021,
  SCALE_CLASS_I_METRIC = 0x0022,
  SCALE_CLASS_II_METRIC = 0x0023,
  SCALE_CLASS_III_METRIC = 0x0024,
  SCALE_CLASS_IIIL_METRIC = 0x0025,
  SCALE_CLASS_IV_METRIC = 0x0026,
  SCALE_CLASS_III_ENGLISH = 0x0027,
  SCALE_CLASS_IIIL_ENGLISH = 0x0028,
  SCALE_CLASS_IV_ENGLISH = 0x0029,
  SCALE_CLASS_GENERIC = 0x002a,
  SCALE_ATTRIBUTE_REPORT = 0x0030,
  SCALE_CONTROL_REPORT = 0x0031,
  SCALE_DATA_REPORT = 0x0032,
  SCALE_STATUS_REPORT = 0x0033,
  SCALE_WEIGHT_LIMIT_REPORT = 0x0034,
  SCALE_STATISTICS_REPORT = 0x0035,
  DATA_WEIGHT = 0x0040,
  DATA_SCALING = 0x0041,
  WEIGHT_UNIT = 0x0050,
  WEIGHT_UNIT_MILLIGRAM = 0x0051,
  WEIGHT_UNIT_GRAM = 0x0052,
  WEIGHT_UNIT_KILOGRAM = 0x0053,
  WEIGHT_UNIT_CARATS = 0x0054,
  WEIGHT_UNIT_TAELS = 0x0055,
  WEIGHT_UNIT_GRAINS = 0x0056,
  WEIGHT_UNIT_PENNYWEIGHTS = 0x0057,
  WEIGHT_UNIT_METRIC_TON = 0x0058,
  WEIGHT_UNIT_AVOIR_TON = 0x0059,
  WEIGHT_UNIT_TROY_OUNCE = 0x005a,
  WEIGHT_UNIT_OUNCE = 0x005b,
  WEIGHT_UNIT_POUND = 0x005c,
  CALIBRATION_COUNT = 0x0060,
  RE_ZERO_COUNT = 0x0061,
  SCALE_STATUS = 0x0070,
  SCALE_STATUS_FAULT = 0x0071,
  SCALE_STATUS_STABLE_AT_CENTER_OF_ZERO = 0x0072,
  SCALE_STATUS_IN_MOTION = 0x0073,
  SCALE_STATUS_WEIGHT_STABLE = 0x0074,
  SCALE_STATUS_UNDER_ZERO = 0x0075,
  SCALE_STATUS_OVER_WEIGHT_LIMIT = 0x0076,
  SCALE_STATUS_REQUIRES_CALIBRATION = 0x0077,
  SCALE_STATUS_REQUIRES_REZEROING = 0x0078,
  ZERO_SCALE = 0x0080,
  ENFORCED_ZERO_RETURN = 0x0081,
};
} // namespace hid::page

#endif // __HID_PAGE_SCALE_HPP_

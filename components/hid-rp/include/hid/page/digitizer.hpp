#ifndef __HID_PAGE_DIGITIZER_HPP_
#define __HID_PAGE_DIGITIZER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class digitizer : std::uint8_t;
template <> constexpr inline auto get_info<digitizer>() {
  return info(
      0x000d, 0x00b0, "Digitizers",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Digitizer";
        case 0x0002:
          return "Pen";
        case 0x0003:
          return "Light Pen";
        case 0x0004:
          return "Touch Screen";
        case 0x0005:
          return "Touch Pad";
        case 0x0006:
          return "Whiteboard";
        case 0x0007:
          return "Coordinate Measuring Machine";
        case 0x0008:
          return "3D Digitizer";
        case 0x0009:
          return "Stereo Plotter";
        case 0x000a:
          return "Articulated Arm";
        case 0x000b:
          return "Armature";
        case 0x000c:
          return "Multiple Point Digitizer";
        case 0x000d:
          return "Free Space Wand";
        case 0x000e:
          return "Device Configuration";
        case 0x000f:
          return "Capacitive Heat Map Digitizer";
        case 0x0020:
          return "Stylus";
        case 0x0021:
          return "Puck";
        case 0x0022:
          return "Finger";
        case 0x0023:
          return "Device Settings";
        case 0x0024:
          return "Character Gesture";
        case 0x0030:
          return "Tip Pressure";
        case 0x0031:
          return "Barrel Pressure";
        case 0x0032:
          return "In Range";
        case 0x0033:
          return "Touch";
        case 0x0034:
          return "Untouch";
        case 0x0035:
          return "Tap";
        case 0x0036:
          return "Quality";
        case 0x0037:
          return "Data Valid";
        case 0x0038:
          return "Transducer Index";
        case 0x0039:
          return "Tablet Function Keys";
        case 0x003a:
          return "Program Change Keys";
        case 0x003b:
          return "Battery Strength";
        case 0x003c:
          return "Invert";
        case 0x003d:
          return "X Tilt";
        case 0x003e:
          return "Y Tilt";
        case 0x003f:
          return "Azimuth";
        case 0x0040:
          return "Altitude";
        case 0x0041:
          return "Twist";
        case 0x0042:
          return "Tip Switch";
        case 0x0043:
          return "Secondary Tip Switch";
        case 0x0044:
          return "Barrel Switch";
        case 0x0045:
          return "Eraser";
        case 0x0046:
          return "Tablet Pick";
        case 0x0047:
          return "Touch Valid";
        case 0x0048:
          return "Width";
        case 0x0049:
          return "Height";
        case 0x0051:
          return "Contact Identifier";
        case 0x0052:
          return "Device Mode";
        case 0x0053:
          return "Device Identifier";
        case 0x0054:
          return "Contact Count";
        case 0x0055:
          return "Contact Count Maximum";
        case 0x0056:
          return "Scan Time";
        case 0x0057:
          return "Surface Switch";
        case 0x0058:
          return "Button Switch";
        case 0x0059:
          return "Pad Type";
        case 0x005a:
          return "Secondary Barrel Switch";
        case 0x005b:
          return "Transducer Serial Number";
        case 0x005c:
          return "Preferred Color";
        case 0x005d:
          return "Preferred Color is Locked";
        case 0x005e:
          return "Preferred Line Width";
        case 0x005f:
          return "Preferred Line Width is Locked";
        case 0x0060:
          return "Latency Mode";
        case 0x0061:
          return "Gesture Character Quality";
        case 0x0062:
          return "Character Gesture Data Length";
        case 0x0063:
          return "Character Gesture Data";
        case 0x0064:
          return "Gesture Character Encoding";
        case 0x0065:
          return "UTF8 Character Gesture Encoding";
        case 0x0066:
          return "UTF16 Little Endian Character Gesture Encoding";
        case 0x0067:
          return "UTF16 Big Endian Character Gesture Encoding";
        case 0x0068:
          return "UTF32 Little Endian Character Gesture Encoding";
        case 0x0069:
          return "UTF32 Big Endian Character Gesture Encoding";
        case 0x006a:
          return "Capacitive Heat Map Protocol Vendor ID";
        case 0x006b:
          return "Capacitive Heat Map Protocol Version";
        case 0x006c:
          return "Capacitive Heat Map Frame Data";
        case 0x006d:
          return "Gesture Character Enable";
        case 0x006e:
          return "Transducer Serial Number Part 2";
        case 0x006f:
          return "No Preferred Color";
        case 0x0070:
          return "Preferred Line Style";
        case 0x0071:
          return "Preferred Line Style is Locked";
        case 0x0072:
          return "Ink";
        case 0x0073:
          return "Pencil";
        case 0x0074:
          return "Highlighter";
        case 0x0075:
          return "Chisel Marker";
        case 0x0076:
          return "Brush";
        case 0x0077:
          return "No Preference";
        case 0x0080:
          return "Digitizer Diagnostic";
        case 0x0081:
          return "Digitizer Error";
        case 0x0082:
          return "Err Normal Status";
        case 0x0083:
          return "Err Transducers Exceeded";
        case 0x0084:
          return "Err Full Trans Features Unavailable";
        case 0x0085:
          return "Err Charge Low";
        case 0x0090:
          return "Transducer Software Info";
        case 0x0091:
          return "Transducer Vendor Id";
        case 0x0092:
          return "Transducer Product Id";
        case 0x0093:
          return "Device Supported Protocols";
        case 0x0094:
          return "Transducer Supported Protocols";
        case 0x0095:
          return "No Protocol";
        case 0x0096:
          return "Wacom AES Protocol";
        case 0x0097:
          return "USI Protocol";
        case 0x0098:
          return "Microsoft Pen Protocol";
        case 0x00a0:
          return "Supported Report Rates";
        case 0x00a1:
          return "Report Rate";
        case 0x00a2:
          return "Transducer Connected";
        case 0x00a3:
          return "Switch Disabled";
        case 0x00a4:
          return "Switch Unimplemented";
        case 0x00a5:
          return "Transducer Switches";
        case 0x00a6:
          return "Transducer Index Selector";
        case 0x00b0:
          return "Button Press Threshold";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class digitizer : std::uint8_t {
  DIGITIZER = 0x0001,
  PEN = 0x0002,
  LIGHT_PEN = 0x0003,
  TOUCH_SCREEN = 0x0004,
  TOUCH_PAD = 0x0005,
  WHITEBOARD = 0x0006,
  COORDINATE_MEASURING_MACHINE = 0x0007,
  _3D_DIGITIZER = 0x0008,
  STEREO_PLOTTER = 0x0009,
  ARTICULATED_ARM = 0x000a,
  ARMATURE = 0x000b,
  MULTIPLE_POINT_DIGITIZER = 0x000c,
  FREE_SPACE_WAND = 0x000d,
  DEVICE_CONFIGURATION = 0x000e,
  CAPACITIVE_HEAT_MAP_DIGITIZER = 0x000f,
  STYLUS = 0x0020,
  PUCK = 0x0021,
  FINGER = 0x0022,
  DEVICE_SETTINGS = 0x0023,
  CHARACTER_GESTURE = 0x0024,
  TIP_PRESSURE = 0x0030,
  BARREL_PRESSURE = 0x0031,
  IN_RANGE = 0x0032,
  TOUCH = 0x0033,
  UNTOUCH = 0x0034,
  TAP = 0x0035,
  QUALITY = 0x0036,
  DATA_VALID = 0x0037,
  TRANSDUCER_INDEX = 0x0038,
  TABLET_FUNCTION_KEYS = 0x0039,
  PROGRAM_CHANGE_KEYS = 0x003a,
  BATTERY_STRENGTH = 0x003b,
  INVERT = 0x003c,
  X_TILT = 0x003d,
  Y_TILT = 0x003e,
  AZIMUTH = 0x003f,
  ALTITUDE = 0x0040,
  TWIST = 0x0041,
  TIP_SWITCH = 0x0042,
  SECONDARY_TIP_SWITCH = 0x0043,
  BARREL_SWITCH = 0x0044,
  ERASER = 0x0045,
  TABLET_PICK = 0x0046,
  TOUCH_VALID = 0x0047,
  WIDTH = 0x0048,
  HEIGHT = 0x0049,
  CONTACT_IDENTIFIER = 0x0051,
  DEVICE_MODE = 0x0052,
  DEVICE_IDENTIFIER = 0x0053,
  CONTACT_COUNT = 0x0054,
  CONTACT_COUNT_MAXIMUM = 0x0055,
  SCAN_TIME = 0x0056,
  SURFACE_SWITCH = 0x0057,
  BUTTON_SWITCH = 0x0058,
  PAD_TYPE = 0x0059,
  SECONDARY_BARREL_SWITCH = 0x005a,
  TRANSDUCER_SERIAL_NUMBER = 0x005b,
  PREFERRED_COLOR = 0x005c,
  PREFERRED_COLOR_IS_LOCKED = 0x005d,
  PREFERRED_LINE_WIDTH = 0x005e,
  PREFERRED_LINE_WIDTH_IS_LOCKED = 0x005f,
  LATENCY_MODE = 0x0060,
  GESTURE_CHARACTER_QUALITY = 0x0061,
  CHARACTER_GESTURE_DATA_LENGTH = 0x0062,
  CHARACTER_GESTURE_DATA = 0x0063,
  GESTURE_CHARACTER_ENCODING = 0x0064,
  UTF8_CHARACTER_GESTURE_ENCODING = 0x0065,
  UTF16_LITTLE_ENDIAN_CHARACTER_GESTURE_ENCODING = 0x0066,
  UTF16_BIG_ENDIAN_CHARACTER_GESTURE_ENCODING = 0x0067,
  UTF32_LITTLE_ENDIAN_CHARACTER_GESTURE_ENCODING = 0x0068,
  UTF32_BIG_ENDIAN_CHARACTER_GESTURE_ENCODING = 0x0069,
  CAPACITIVE_HEAT_MAP_PROTOCOL_VENDOR_ID = 0x006a,
  CAPACITIVE_HEAT_MAP_PROTOCOL_VERSION = 0x006b,
  CAPACITIVE_HEAT_MAP_FRAME_DATA = 0x006c,
  GESTURE_CHARACTER_ENABLE = 0x006d,
  TRANSDUCER_SERIAL_NUMBER_PART_2 = 0x006e,
  NO_PREFERRED_COLOR = 0x006f,
  PREFERRED_LINE_STYLE = 0x0070,
  PREFERRED_LINE_STYLE_IS_LOCKED = 0x0071,
  INK = 0x0072,
  PENCIL = 0x0073,
  HIGHLIGHTER = 0x0074,
  CHISEL_MARKER = 0x0075,
  BRUSH = 0x0076,
  NO_PREFERENCE = 0x0077,
  DIGITIZER_DIAGNOSTIC = 0x0080,
  DIGITIZER_ERROR = 0x0081,
  ERR_NORMAL_STATUS = 0x0082,
  ERR_TRANSDUCERS_EXCEEDED = 0x0083,
  ERR_FULL_TRANS_FEATURES_UNAVAILABLE = 0x0084,
  ERR_CHARGE_LOW = 0x0085,
  TRANSDUCER_SOFTWARE_INFO = 0x0090,
  TRANSDUCER_VENDOR_ID = 0x0091,
  TRANSDUCER_PRODUCT_ID = 0x0092,
  DEVICE_SUPPORTED_PROTOCOLS = 0x0093,
  TRANSDUCER_SUPPORTED_PROTOCOLS = 0x0094,
  NO_PROTOCOL = 0x0095,
  WACOM_AES_PROTOCOL = 0x0096,
  USI_PROTOCOL = 0x0097,
  MICROSOFT_PEN_PROTOCOL = 0x0098,
  SUPPORTED_REPORT_RATES = 0x00a0,
  REPORT_RATE = 0x00a1,
  TRANSDUCER_CONNECTED = 0x00a2,
  SWITCH_DISABLED = 0x00a3,
  SWITCH_UNIMPLEMENTED = 0x00a4,
  TRANSDUCER_SWITCHES = 0x00a5,
  TRANSDUCER_INDEX_SELECTOR = 0x00a6,
  BUTTON_PRESS_THRESHOLD = 0x00b0,
};
} // namespace hid::page

#endif // __HID_PAGE_DIGITIZER_HPP_

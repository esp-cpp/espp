#ifndef __HID_PAGE_GENERIC_DESKTOP_HPP_
#define __HID_PAGE_GENERIC_DESKTOP_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class generic_desktop : std::uint8_t;
template <> constexpr inline auto get_info<generic_desktop>() {
  return info(
      0x0001, 0x00e2, "Generic Desktop",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Pointer";
        case 0x0002:
          return "Mouse";
        case 0x0004:
          return "Joystick";
        case 0x0005:
          return "Gamepad";
        case 0x0006:
          return "Keyboard";
        case 0x0007:
          return "Keypad";
        case 0x0008:
          return "Multi-axis Controller";
        case 0x0009:
          return "Tablet PC System Controls";
        case 0x000a:
          return "Water Cooling Device";
        case 0x000b:
          return "Computer Chassis Device";
        case 0x000c:
          return "Wireless Radio Controls";
        case 0x000d:
          return "Portable Device Control";
        case 0x000e:
          return "System Multi-Axis Controller";
        case 0x000f:
          return "Spatial Controller";
        case 0x0010:
          return "Assistive Control";
        case 0x0011:
          return "Device Dock";
        case 0x0012:
          return "Dockable Device";
        case 0x0013:
          return "Call State Management Control";
        case 0x0030:
          return "X";
        case 0x0031:
          return "Y";
        case 0x0032:
          return "Z";
        case 0x0033:
          return "Rx";
        case 0x0034:
          return "Ry";
        case 0x0035:
          return "Rz";
        case 0x0036:
          return "Slider";
        case 0x0037:
          return "Dial";
        case 0x0038:
          return "Wheel";
        case 0x0039:
          return "Hat Switch";
        case 0x003a:
          return "Counted Buffer";
        case 0x003b:
          return "Byte Count";
        case 0x003c:
          return "Motion Wakeup";
        case 0x003d:
          return "Start";
        case 0x003e:
          return "Select";
        case 0x0040:
          return "Vx";
        case 0x0041:
          return "Vy";
        case 0x0042:
          return "Vz";
        case 0x0043:
          return "Vbrx";
        case 0x0044:
          return "Vbry";
        case 0x0045:
          return "Vbrz";
        case 0x0046:
          return "Vno";
        case 0x0047:
          return "Feature Notification";
        case 0x0048:
          return "Resolution Multiplier";
        case 0x0049:
          return "Qx";
        case 0x004a:
          return "Qy";
        case 0x004b:
          return "Qz";
        case 0x004c:
          return "Qw";
        case 0x0080:
          return "System Control";
        case 0x0081:
          return "System Power Down";
        case 0x0082:
          return "System Sleep";
        case 0x0083:
          return "System Wake Up";
        case 0x0084:
          return "System Context Menu";
        case 0x0085:
          return "System Main Menu";
        case 0x0086:
          return "System App Menu";
        case 0x0087:
          return "System Menu Help";
        case 0x0088:
          return "System Menu Exit";
        case 0x0089:
          return "System Menu Select";
        case 0x008a:
          return "System Menu Right";
        case 0x008b:
          return "System Menu Left";
        case 0x008c:
          return "System Menu Up";
        case 0x008d:
          return "System Menu Down";
        case 0x008e:
          return "System Cold Restart";
        case 0x008f:
          return "System Warm Restart";
        case 0x0090:
          return "D-pad Up";
        case 0x0091:
          return "D-pad Down";
        case 0x0092:
          return "D-pad Right";
        case 0x0093:
          return "D-pad Left";
        case 0x0094:
          return "Index Trigger";
        case 0x0095:
          return "Palm Trigger";
        case 0x0096:
          return "Thumbstick";
        case 0x0097:
          return "System Function Shift";
        case 0x0098:
          return "System Function Shift Lock";
        case 0x0099:
          return "System Function Shift Lock Indicator";
        case 0x009a:
          return "System Dismiss Notification";
        case 0x009b:
          return "System Do Not Disturb";
        case 0x00a0:
          return "System Dock";
        case 0x00a1:
          return "System Undock";
        case 0x00a2:
          return "System Setup";
        case 0x00a3:
          return "System Break";
        case 0x00a4:
          return "System Debugger Break";
        case 0x00a5:
          return "Application Break";
        case 0x00a6:
          return "Application Debugger Break";
        case 0x00a7:
          return "System Speaker Mute";
        case 0x00a8:
          return "System Hibernate";
        case 0x00a9:
          return "System Microphone Mute";
        case 0x00aa:
          return "System Accessibility Binding";
        case 0x00b0:
          return "System Display Invert";
        case 0x00b1:
          return "System Display Internal";
        case 0x00b2:
          return "System Display External";
        case 0x00b3:
          return "System Display Both";
        case 0x00b4:
          return "System Display Dual";
        case 0x00b5:
          return "System Display Toggle Int/Ext Mode";
        case 0x00b6:
          return "System Display Swap Primary/Secondary";
        case 0x00b7:
          return "System Display Toggle LCD Autoscale";
        case 0x00c0:
          return "Sensor Zone";
        case 0x00c1:
          return "RPM";
        case 0x00c2:
          return "Coolant Level";
        case 0x00c3:
          return "Coolant Critical Level";
        case 0x00c4:
          return "Coolant Pump";
        case 0x00c5:
          return "Chassis Enclosure";
        case 0x00c6:
          return "Wireless Radio Button";
        case 0x00c7:
          return "Wireless Radio LED";
        case 0x00c8:
          return "Wireless Radio Slider Switch";
        case 0x00c9:
          return "System Display Rotation Lock Button";
        case 0x00ca:
          return "System Display Rotation Lock Slider Switch";
        case 0x00cb:
          return "Control Enable";
        case 0x00d0:
          return "Dockable Device Unique ID";
        case 0x00d1:
          return "Dockable Device Vendor ID";
        case 0x00d2:
          return "Dockable Device Primary Usage Page";
        case 0x00d3:
          return "Dockable Device Primary Usage ID";
        case 0x00d4:
          return "Dockable Device Docking State";
        case 0x00d5:
          return "Dockable Device Display Occlusion";
        case 0x00d6:
          return "Dockable Device Object Type";
        case 0x00e0:
          return "Call Active LED";
        case 0x00e1:
          return "Call Mute Toggle";
        case 0x00e2:
          return "Call Mute LED";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class generic_desktop : std::uint8_t {
  POINTER = 0x0001,
  MOUSE = 0x0002,
  JOYSTICK = 0x0004,
  GAMEPAD = 0x0005,
  KEYBOARD = 0x0006,
  KEYPAD = 0x0007,
  MULTI_AXIS_CONTROLLER = 0x0008,
  TABLET_PC_SYSTEM_CONTROLS = 0x0009,
  WATER_COOLING_DEVICE = 0x000a,
  COMPUTER_CHASSIS_DEVICE = 0x000b,
  WIRELESS_RADIO_CONTROLS = 0x000c,
  PORTABLE_DEVICE_CONTROL = 0x000d,
  SYSTEM_MULTI_AXIS_CONTROLLER = 0x000e,
  SPATIAL_CONTROLLER = 0x000f,
  ASSISTIVE_CONTROL = 0x0010,
  DEVICE_DOCK = 0x0011,
  DOCKABLE_DEVICE = 0x0012,
  CALL_STATE_MANAGEMENT_CONTROL = 0x0013,
  X = 0x0030,
  Y = 0x0031,
  Z = 0x0032,
  RX = 0x0033,
  RY = 0x0034,
  RZ = 0x0035,
  SLIDER = 0x0036,
  DIAL = 0x0037,
  WHEEL = 0x0038,
  HAT_SWITCH = 0x0039,
  COUNTED_BUFFER = 0x003a,
  BYTE_COUNT = 0x003b,
  MOTION_WAKEUP = 0x003c,
  START = 0x003d,
  SELECT = 0x003e,
  VX = 0x0040,
  VY = 0x0041,
  VZ = 0x0042,
  VBRX = 0x0043,
  VBRY = 0x0044,
  VBRZ = 0x0045,
  VNO = 0x0046,
  FEATURE_NOTIFICATION = 0x0047,
  RESOLUTION_MULTIPLIER = 0x0048,
  QX = 0x0049,
  QY = 0x004a,
  QZ = 0x004b,
  QW = 0x004c,
  SYSTEM_CONTROL = 0x0080,
  SYSTEM_POWER_DOWN = 0x0081,
  SYSTEM_SLEEP = 0x0082,
  SYSTEM_WAKE_UP = 0x0083,
  SYSTEM_CONTEXT_MENU = 0x0084,
  SYSTEM_MAIN_MENU = 0x0085,
  SYSTEM_APP_MENU = 0x0086,
  SYSTEM_MENU_HELP = 0x0087,
  SYSTEM_MENU_EXIT = 0x0088,
  SYSTEM_MENU_SELECT = 0x0089,
  SYSTEM_MENU_RIGHT = 0x008a,
  SYSTEM_MENU_LEFT = 0x008b,
  SYSTEM_MENU_UP = 0x008c,
  SYSTEM_MENU_DOWN = 0x008d,
  SYSTEM_COLD_RESTART = 0x008e,
  SYSTEM_WARM_RESTART = 0x008f,
  D_PAD_UP = 0x0090,
  D_PAD_DOWN = 0x0091,
  D_PAD_RIGHT = 0x0092,
  D_PAD_LEFT = 0x0093,
  INDEX_TRIGGER = 0x0094,
  PALM_TRIGGER = 0x0095,
  THUMBSTICK = 0x0096,
  SYSTEM_FUNCTION_SHIFT = 0x0097,
  SYSTEM_FUNCTION_SHIFT_LOCK = 0x0098,
  SYSTEM_FUNCTION_SHIFT_LOCK_INDICATOR = 0x0099,
  SYSTEM_DISMISS_NOTIFICATION = 0x009a,
  SYSTEM_DO_NOT_DISTURB = 0x009b,
  SYSTEM_DOCK = 0x00a0,
  SYSTEM_UNDOCK = 0x00a1,
  SYSTEM_SETUP = 0x00a2,
  SYSTEM_BREAK = 0x00a3,
  SYSTEM_DEBUGGER_BREAK = 0x00a4,
  APPLICATION_BREAK = 0x00a5,
  APPLICATION_DEBUGGER_BREAK = 0x00a6,
  SYSTEM_SPEAKER_MUTE = 0x00a7,
  SYSTEM_HIBERNATE = 0x00a8,
  SYSTEM_MICROPHONE_MUTE = 0x00a9,
  SYSTEM_ACCESSIBILITY_BINDING = 0x00aa,
  SYSTEM_DISPLAY_INVERT = 0x00b0,
  SYSTEM_DISPLAY_INTERNAL = 0x00b1,
  SYSTEM_DISPLAY_EXTERNAL = 0x00b2,
  SYSTEM_DISPLAY_BOTH = 0x00b3,
  SYSTEM_DISPLAY_DUAL = 0x00b4,
  SYSTEM_DISPLAY_TOGGLE_INT_EXT_MODE = 0x00b5,
  SYSTEM_DISPLAY_SWAP_PRIMARY_SECONDARY = 0x00b6,
  SYSTEM_DISPLAY_TOGGLE_LCD_AUTOSCALE = 0x00b7,
  SENSOR_ZONE = 0x00c0,
  RPM = 0x00c1,
  COOLANT_LEVEL = 0x00c2,
  COOLANT_CRITICAL_LEVEL = 0x00c3,
  COOLANT_PUMP = 0x00c4,
  CHASSIS_ENCLOSURE = 0x00c5,
  WIRELESS_RADIO_BUTTON = 0x00c6,
  WIRELESS_RADIO_LED = 0x00c7,
  WIRELESS_RADIO_SLIDER_SWITCH = 0x00c8,
  SYSTEM_DISPLAY_ROTATION_LOCK_BUTTON = 0x00c9,
  SYSTEM_DISPLAY_ROTATION_LOCK_SLIDER_SWITCH = 0x00ca,
  CONTROL_ENABLE = 0x00cb,
  DOCKABLE_DEVICE_UNIQUE_ID = 0x00d0,
  DOCKABLE_DEVICE_VENDOR_ID = 0x00d1,
  DOCKABLE_DEVICE_PRIMARY_USAGE_PAGE = 0x00d2,
  DOCKABLE_DEVICE_PRIMARY_USAGE_ID = 0x00d3,
  DOCKABLE_DEVICE_DOCKING_STATE = 0x00d4,
  DOCKABLE_DEVICE_DISPLAY_OCCLUSION = 0x00d5,
  DOCKABLE_DEVICE_OBJECT_TYPE = 0x00d6,
  CALL_ACTIVE_LED = 0x00e0,
  CALL_MUTE_TOGGLE = 0x00e1,
  CALL_MUTE_LED = 0x00e2,
};
} // namespace hid::page

#endif // __HID_PAGE_GENERIC_DESKTOP_HPP_

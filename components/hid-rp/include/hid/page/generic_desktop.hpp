#ifndef __HID_PAGE_GENERIC_DESKTOP_HPP_
#define __HID_PAGE_GENERIC_DESKTOP_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class generic_desktop : std::uint8_t;
template <> struct info<generic_desktop> {
  constexpr static page_id_t page_id = 0x0001;
  constexpr static usage_id_t max_usage_id = 0x00e2;
  constexpr static const char *name = "Generic Desktop";
};
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

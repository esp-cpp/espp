#ifndef __HID_PAGE_HAPTICS_HPP_
#define __HID_PAGE_HAPTICS_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class haptics : std::uint16_t;
template <> constexpr inline auto get_info<haptics>() {
  return info(
      0x000e, 0x1011, "Haptics",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Simple Haptic Controller";
        case 0x0010:
          return "Waveform List";
        case 0x0011:
          return "Duration List";
        case 0x0020:
          return "Auto Trigger";
        case 0x0021:
          return "Manual Trigger";
        case 0x0022:
          return "Auto Trigger Associated Control";
        case 0x0023:
          return "Intensity";
        case 0x0024:
          return "Repeat Count";
        case 0x0025:
          return "Retrigger Period";
        case 0x0026:
          return "Waveform Vendor Page";
        case 0x0027:
          return "Waveform Vendor ID";
        case 0x0028:
          return "Waveform Cutoff Time";
        case 0x1001:
          return "Waveform None";
        case 0x1002:
          return "Waveform Stop";
        case 0x1003:
          return "Waveform Click";
        case 0x1004:
          return "Waveform Buzz Continuous";
        case 0x1005:
          return "Waveform Rumble Continuous";
        case 0x1006:
          return "Waveform Press";
        case 0x1007:
          return "Waveform Release";
        case 0x1008:
          return "Waveform Hover";
        case 0x1009:
          return "Waveform Success";
        case 0x100a:
          return "Waveform Error";
        case 0x100b:
          return "Waveform Ink Continuous";
        case 0x100c:
          return "Waveform Pencil Continuous";
        case 0x100d:
          return "Waveform Marker Continuous";
        case 0x100e:
          return "Waveform Chisel Marker Continuous";
        case 0x100f:
          return "Waveform Brush Continuous";
        case 0x1010:
          return "Waveform Eraser Continuous";
        case 0x1011:
          return "Waveform Sparkle Continuous";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class haptics : std::uint16_t {
  SIMPLE_HAPTIC_CONTROLLER = 0x0001,
  WAVEFORM_LIST = 0x0010,
  DURATION_LIST = 0x0011,
  AUTO_TRIGGER = 0x0020,
  MANUAL_TRIGGER = 0x0021,
  AUTO_TRIGGER_ASSOCIATED_CONTROL = 0x0022,
  INTENSITY = 0x0023,
  REPEAT_COUNT = 0x0024,
  RETRIGGER_PERIOD = 0x0025,
  WAVEFORM_VENDOR_PAGE = 0x0026,
  WAVEFORM_VENDOR_ID = 0x0027,
  WAVEFORM_CUTOFF_TIME = 0x0028,
  WAVEFORM_NONE = 0x1001,
  WAVEFORM_STOP = 0x1002,
  WAVEFORM_CLICK = 0x1003,
  WAVEFORM_BUZZ_CONTINUOUS = 0x1004,
  WAVEFORM_RUMBLE_CONTINUOUS = 0x1005,
  WAVEFORM_PRESS = 0x1006,
  WAVEFORM_RELEASE = 0x1007,
  WAVEFORM_HOVER = 0x1008,
  WAVEFORM_SUCCESS = 0x1009,
  WAVEFORM_ERROR = 0x100a,
  WAVEFORM_INK_CONTINUOUS = 0x100b,
  WAVEFORM_PENCIL_CONTINUOUS = 0x100c,
  WAVEFORM_MARKER_CONTINUOUS = 0x100d,
  WAVEFORM_CHISEL_MARKER_CONTINUOUS = 0x100e,
  WAVEFORM_BRUSH_CONTINUOUS = 0x100f,
  WAVEFORM_ERASER_CONTINUOUS = 0x1010,
  WAVEFORM_SPARKLE_CONTINUOUS = 0x1011,
};
} // namespace hid::page

#endif // __HID_PAGE_HAPTICS_HPP_

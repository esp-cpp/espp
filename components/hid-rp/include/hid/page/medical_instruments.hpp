#ifndef __HID_PAGE_MEDICAL_INSTRUMENTS_HPP_
#define __HID_PAGE_MEDICAL_INSTRUMENTS_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class medical_instruments : std::uint8_t;
template <> constexpr inline auto get_info<medical_instruments>() {
  return info(
      0x0040, 0x00a1, "Medical Instrument",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Medical Ultrasound";
        case 0x0020:
          return "VCR/Acquisition";
        case 0x0021:
          return "Freeze/Thaw";
        case 0x0022:
          return "Clip Store";
        case 0x0023:
          return "Update";
        case 0x0024:
          return "Next";
        case 0x0025:
          return "Save";
        case 0x0026:
          return "Print";
        case 0x0027:
          return "Microphone Enable";
        case 0x0040:
          return "Cine";
        case 0x0041:
          return "Transmit Power";
        case 0x0042:
          return "Volume";
        case 0x0043:
          return "Focus";
        case 0x0044:
          return "Depth";
        case 0x0060:
          return "Soft Step - Primary";
        case 0x0061:
          return "Soft Step - Secondary";
        case 0x0070:
          return "Depth Gain Compensation";
        case 0x0080:
          return "Zoom Select";
        case 0x0081:
          return "Zoom Adjust";
        case 0x0082:
          return "Spectral Doppler Mode Select";
        case 0x0083:
          return "Spectral Doppler Adjust";
        case 0x0084:
          return "Color Doppler Mode Select";
        case 0x0085:
          return "Color Doppler Adjust";
        case 0x0086:
          return "Motion Mode Select";
        case 0x0087:
          return "Motion Mode Adjust";
        case 0x0088:
          return "2-D Mode Select";
        case 0x0089:
          return "2-D Mode Adjust";
        case 0x00a0:
          return "Soft Control Select";
        case 0x00a1:
          return "Soft Control Adjust";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class medical_instruments : std::uint8_t {
  MEDICAL_ULTRASOUND = 0x0001,
  VCR_ACQUISITION = 0x0020,
  FREEZE_THAW = 0x0021,
  CLIP_STORE = 0x0022,
  UPDATE = 0x0023,
  NEXT = 0x0024,
  SAVE = 0x0025,
  PRINT = 0x0026,
  MICROPHONE_ENABLE = 0x0027,
  CINE = 0x0040,
  TRANSMIT_POWER = 0x0041,
  VOLUME = 0x0042,
  FOCUS = 0x0043,
  DEPTH = 0x0044,
  SOFT_STEP_PRIMARY = 0x0060,
  SOFT_STEP_SECONDARY = 0x0061,
  DEPTH_GAIN_COMPENSATION = 0x0070,
  ZOOM_SELECT = 0x0080,
  ZOOM_ADJUST = 0x0081,
  SPECTRAL_DOPPLER_MODE_SELECT = 0x0082,
  SPECTRAL_DOPPLER_ADJUST = 0x0083,
  COLOR_DOPPLER_MODE_SELECT = 0x0084,
  COLOR_DOPPLER_ADJUST = 0x0085,
  MOTION_MODE_SELECT = 0x0086,
  MOTION_MODE_ADJUST = 0x0087,
  _2_D_MODE_SELECT = 0x0088,
  _2_D_MODE_ADJUST = 0x0089,
  SOFT_CONTROL_SELECT = 0x00a0,
  SOFT_CONTROL_ADJUST = 0x00a1,
};
} // namespace hid::page

#endif // __HID_PAGE_MEDICAL_INSTRUMENTS_HPP_

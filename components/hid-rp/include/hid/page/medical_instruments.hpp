#ifndef __HID_PAGE_MEDICAL_INSTRUMENTS_HPP_
#define __HID_PAGE_MEDICAL_INSTRUMENTS_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class medical_instruments : std::uint8_t;
template <> struct info<medical_instruments> {
  constexpr static page_id_t page_id = 0x0040;
  constexpr static usage_id_t max_usage_id = 0x00a1;
  constexpr static const char *name = "Medical Instrument";
};
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

#ifndef __HID_PAGE_EYE_AND_HEAD_TRACKERS_HPP_
#define __HID_PAGE_EYE_AND_HEAD_TRACKERS_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class eye_and_head_trackers : std::uint16_t;
template <> struct info<eye_and_head_trackers> {
  constexpr static page_id_t page_id = 0x0012;
  constexpr static usage_id_t max_usage_id = 0x0400;
  constexpr static const char *name = "Eye and Head Trackers";
};
enum class eye_and_head_trackers : std::uint16_t {
  EYE_TRACKER = 0x0001,
  HEAD_TRACKER = 0x0002,
  TRACKING_DATA = 0x0010,
  CAPABILITIES = 0x0011,
  CONFIGURATION = 0x0012,
  STATUS = 0x0013,
  CONTROL = 0x0014,
  SENSOR_TIMESTAMP = 0x0020,
  POSITION_X = 0x0021,
  POSITION_Y = 0x0022,
  POSITION_Z = 0x0023,
  GAZE_POINT = 0x0024,
  LEFT_EYE_POSITION = 0x0025,
  RIGHT_EYE_POSITION = 0x0026,
  HEAD_POSITION = 0x0027,
  HEAD_DIRECTION_POINT = 0x0028,
  ROTATION_ABOUT_X_AXIS = 0x0029,
  ROTATION_ABOUT_Y_AXIS = 0x002a,
  ROTATION_ABOUT_Z_AXIS = 0x002b,
  TRACKER_QUALITY = 0x0100,
  MINIMUM_TRACKING_DISTANCE = 0x0101,
  OPTIMUM_TRACKING_DISTANCE = 0x0102,
  MAXIMUM_TRACKING_DISTANCE = 0x0103,
  MAXIMUM_SCREEN_PLANE_WIDTH = 0x0104,
  MAXIMUM_SCREEN_PLANE_HEIGHT = 0x0105,
  DISPLAY_MANUFACTURER_ID = 0x0200,
  DISPLAY_PRODUCT_ID = 0x0201,
  DISPLAY_SERIAL_NUMBER = 0x0202,
  DISPLAY_MANUFACTURER_DATE = 0x0203,
  CALIBRATED_SCREEN_WIDTH = 0x0204,
  CALIBRATED_SCREEN_HEIGHT = 0x0205,
  SAMPLING_FREQUENCY = 0x0300,
  CONFIGURATION_STATUS = 0x0301,
  DEVICE_MODE_REQUEST = 0x0400,
};
} // namespace hid::page

#endif // __HID_PAGE_EYE_AND_HEAD_TRACKERS_HPP_

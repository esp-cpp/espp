#pragma once

#include <cstdint>

namespace espp {
/// \brief The appearance of a BLE device
/// \note This is a subset of the Bluetooth SIG Assigned Numbers
/// \note See
///       https://www.bluetooth.com/specifications/assigned-numbers/generic-access-profile/
///       specifically, pages 29-30 of
///       https://www.bluetooth.com/wp-content/uploads/Files/Specification/HTML/Assigned_Numbers/out/en/Assigned_Numbers.pdf?v=1710445481092
enum class BleAppearance : uint16_t {
  UNKNOWN = 0,
  GENERIC_PHONE = 64,
  GENERIC_COMPUTER = 128,
  GENERIC_WATCH = 192,
  WATCH_SPORTS_WATCH = 193,
  GENERIC_CLOCK = 256,
  GENERIC_DISPLAY = 320,
  GENERIC_REMOTE_CONTROL = 384,
  GENERIC_EYE_GLASSES = 448,
  GENERIC_TAG = 512,
  GENERIC_KEYRING = 576,
  GENERIC_MEDIA_PLAYER = 640,
  GENERIC_BARCODE_SCANNER = 704,
  GENERIC_THERMOMETER = 768,
  THERMOMETER_EAR = 769,
  GENERIC_HEART_RATE_SENSOR = 832,
  HEART_RATE_SENSOR_HEART_RATE_BELT = 833,
  GENERIC_BLOOD_PRESSURE = 896,
  BLOOD_PRESSURE_ARM = 897,
  BLOOD_PRESSURE_WRIST = 898,
  HUMAN_INTERFACE_DEVICE = 960,
  KEYBOARD = 961,
  MOUSE = 962,
  JOYSTICK = 963,
  GAMEPAD = 964,
  DIGITIZER_TABLET = 965,
  CARD_READER = 966,
  DIGITAL_PEN = 967,
  BARCODE_SCANNER = 968,
};
} // namespace espp

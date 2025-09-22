#ifndef __HID_PAGE_LEDS_HPP_
#define __HID_PAGE_LEDS_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class leds : std::uint8_t;
template <> constexpr inline auto get_info<leds>() {
  return info(
      0x0008, 0x0068, "LED",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Num Lock";
        case 0x0002:
          return "Caps Lock";
        case 0x0003:
          return "Scroll Lock";
        case 0x0004:
          return "Compose";
        case 0x0005:
          return "Kana";
        case 0x0006:
          return "Power";
        case 0x0007:
          return "Shift";
        case 0x0008:
          return "Do Not Disturb";
        case 0x0009:
          return "Mute";
        case 0x000a:
          return "Tone Enable";
        case 0x000b:
          return "High Cut Filter";
        case 0x000c:
          return "Low Cut Filter";
        case 0x000d:
          return "Equalizer Enable";
        case 0x000e:
          return "Sound Field On";
        case 0x000f:
          return "Surround On";
        case 0x0010:
          return "Repeat";
        case 0x0011:
          return "Stereo";
        case 0x0012:
          return "Sampling Rate Detect";
        case 0x0013:
          return "Spinning";
        case 0x0014:
          return "CAV";
        case 0x0015:
          return "CLV";
        case 0x0016:
          return "Recording Format Detect";
        case 0x0017:
          return "Off-Hook";
        case 0x0018:
          return "Ring";
        case 0x0019:
          return "Message Waiting";
        case 0x001a:
          return "Data Mode";
        case 0x001b:
          return "Battery Operation";
        case 0x001c:
          return "Battery OK";
        case 0x001d:
          return "Battery Low";
        case 0x001e:
          return "Speaker";
        case 0x001f:
          return "Headset";
        case 0x0020:
          return "Hold";
        case 0x0021:
          return "Microphone";
        case 0x0022:
          return "Coverage";
        case 0x0023:
          return "Night Mode";
        case 0x0024:
          return "Send Calls";
        case 0x0025:
          return "Call Pickup";
        case 0x0026:
          return "Conference";
        case 0x0027:
          return "Stand-by";
        case 0x0028:
          return "Camera On";
        case 0x0029:
          return "Camera Off";
        case 0x002a:
          return "On-Line";
        case 0x002b:
          return "Off-Line";
        case 0x002c:
          return "Busy";
        case 0x002d:
          return "Ready";
        case 0x002e:
          return "Paper-Out";
        case 0x002f:
          return "Paper-Jam";
        case 0x0030:
          return "Remote";
        case 0x0031:
          return "Forward";
        case 0x0032:
          return "Reverse";
        case 0x0033:
          return "Stop";
        case 0x0034:
          return "Rewind";
        case 0x0035:
          return "Fast Forward";
        case 0x0036:
          return "Play";
        case 0x0037:
          return "Pause";
        case 0x0038:
          return "Record";
        case 0x0039:
          return "Error";
        case 0x003a:
          return "Usage Selected Indicator";
        case 0x003b:
          return "Usage In Use Indicator";
        case 0x003c:
          return "Usage Multi Mode Indicator";
        case 0x003d:
          return "Indicator On";
        case 0x003e:
          return "Indicator Flash";
        case 0x003f:
          return "Indicator Slow Blink";
        case 0x0040:
          return "Indicator Fast Blink";
        case 0x0041:
          return "Indicator Off";
        case 0x0042:
          return "Flash On Time";
        case 0x0043:
          return "Slow Blink On Time";
        case 0x0044:
          return "Slow Blink Off Time";
        case 0x0045:
          return "Fast Blink On Time";
        case 0x0046:
          return "Fast Blink Off Time";
        case 0x0047:
          return "Usage Indicator Color";
        case 0x0048:
          return "Indicator Red";
        case 0x0049:
          return "Indicator Green";
        case 0x004a:
          return "Indicator Amber";
        case 0x004b:
          return "Generic Indicator";
        case 0x004c:
          return "System Suspend";
        case 0x004d:
          return "External Power Connected";
        case 0x004e:
          return "Indicator Blue";
        case 0x004f:
          return "Indicator Orange";
        case 0x0050:
          return "Good Status";
        case 0x0051:
          return "Warning Status";
        case 0x0052:
          return "RGB LED";
        case 0x0053:
          return "Red LED Channel";
        case 0x0054:
          return "Blue LED Channel";
        case 0x0055:
          return "Green LED Channel";
        case 0x0056:
          return "LED Intensity";
        case 0x0057:
          return "System Microphone Mute";
        case 0x0060:
          return "Player Indicator";
        case 0x0061:
          return "Player 1";
        case 0x0062:
          return "Player 2";
        case 0x0063:
          return "Player 3";
        case 0x0064:
          return "Player 4";
        case 0x0065:
          return "Player 5";
        case 0x0066:
          return "Player 6";
        case 0x0067:
          return "Player 7";
        case 0x0068:
          return "Player 8";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class leds : std::uint8_t {
  NUM_LOCK = 0x0001,
  CAPS_LOCK = 0x0002,
  SCROLL_LOCK = 0x0003,
  COMPOSE = 0x0004,
  KANA = 0x0005,
  POWER = 0x0006,
  SHIFT = 0x0007,
  DO_NOT_DISTURB = 0x0008,
  MUTE = 0x0009,
  TONE_ENABLE = 0x000a,
  HIGH_CUT_FILTER = 0x000b,
  LOW_CUT_FILTER = 0x000c,
  EQUALIZER_ENABLE = 0x000d,
  SOUND_FIELD_ON = 0x000e,
  SURROUND_ON = 0x000f,
  REPEAT = 0x0010,
  STEREO = 0x0011,
  SAMPLING_RATE_DETECT = 0x0012,
  SPINNING = 0x0013,
  CAV = 0x0014,
  CLV = 0x0015,
  RECORDING_FORMAT_DETECT = 0x0016,
  OFF_HOOK = 0x0017,
  RING = 0x0018,
  MESSAGE_WAITING = 0x0019,
  DATA_MODE = 0x001a,
  BATTERY_OPERATION = 0x001b,
  BATTERY_OK = 0x001c,
  BATTERY_LOW = 0x001d,
  SPEAKER = 0x001e,
  HEADSET = 0x001f,
  HOLD = 0x0020,
  MICROPHONE = 0x0021,
  COVERAGE = 0x0022,
  NIGHT_MODE = 0x0023,
  SEND_CALLS = 0x0024,
  CALL_PICKUP = 0x0025,
  CONFERENCE = 0x0026,
  STAND_BY = 0x0027,
  CAMERA_ON = 0x0028,
  CAMERA_OFF = 0x0029,
  ON_LINE = 0x002a,
  OFF_LINE = 0x002b,
  BUSY = 0x002c,
  READY = 0x002d,
  PAPER_OUT = 0x002e,
  PAPER_JAM = 0x002f,
  REMOTE = 0x0030,
  FORWARD = 0x0031,
  REVERSE = 0x0032,
  STOP = 0x0033,
  REWIND = 0x0034,
  FAST_FORWARD = 0x0035,
  PLAY = 0x0036,
  PAUSE = 0x0037,
  RECORD = 0x0038,
  ERROR = 0x0039,
  USAGE_SELECTED_INDICATOR = 0x003a,
  USAGE_IN_USE_INDICATOR = 0x003b,
  USAGE_MULTI_MODE_INDICATOR = 0x003c,
  INDICATOR_ON = 0x003d,
  INDICATOR_FLASH = 0x003e,
  INDICATOR_SLOW_BLINK = 0x003f,
  INDICATOR_FAST_BLINK = 0x0040,
  INDICATOR_OFF = 0x0041,
  FLASH_ON_TIME = 0x0042,
  SLOW_BLINK_ON_TIME = 0x0043,
  SLOW_BLINK_OFF_TIME = 0x0044,
  FAST_BLINK_ON_TIME = 0x0045,
  FAST_BLINK_OFF_TIME = 0x0046,
  USAGE_INDICATOR_COLOR = 0x0047,
  INDICATOR_RED = 0x0048,
  INDICATOR_GREEN = 0x0049,
  INDICATOR_AMBER = 0x004a,
  GENERIC_INDICATOR = 0x004b,
  SYSTEM_SUSPEND = 0x004c,
  EXTERNAL_POWER_CONNECTED = 0x004d,
  INDICATOR_BLUE = 0x004e,
  INDICATOR_ORANGE = 0x004f,
  GOOD_STATUS = 0x0050,
  WARNING_STATUS = 0x0051,
  RGB_LED = 0x0052,
  RED_LED_CHANNEL = 0x0053,
  BLUE_LED_CHANNEL = 0x0054,
  GREEN_LED_CHANNEL = 0x0055,
  LED_INTENSITY = 0x0056,
  SYSTEM_MICROPHONE_MUTE = 0x0057,
  PLAYER_INDICATOR = 0x0060,
  PLAYER_1 = 0x0061,
  PLAYER_2 = 0x0062,
  PLAYER_3 = 0x0063,
  PLAYER_4 = 0x0064,
  PLAYER_5 = 0x0065,
  PLAYER_6 = 0x0066,
  PLAYER_7 = 0x0067,
  PLAYER_8 = 0x0068,
};
} // namespace hid::page

#endif // __HID_PAGE_LEDS_HPP_

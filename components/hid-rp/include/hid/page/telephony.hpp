#ifndef __HID_PAGE_TELEPHONY_HPP_
#define __HID_PAGE_TELEPHONY_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class telephony : std::uint16_t;
template <> constexpr inline auto get_info<telephony>() {
  return info(
      0x000b, 0x014b, "Telephony Device",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Phone";
        case 0x0002:
          return "Answering Machine";
        case 0x0003:
          return "Message Controls";
        case 0x0004:
          return "Handset";
        case 0x0005:
          return "Headset";
        case 0x0006:
          return "Telephony Key Pad";
        case 0x0007:
          return "Programmable Button";
        case 0x0020:
          return "Hook Switch";
        case 0x0021:
          return "Flash";
        case 0x0022:
          return "Feature";
        case 0x0023:
          return "Hold";
        case 0x0024:
          return "Redial";
        case 0x0025:
          return "Transfer";
        case 0x0026:
          return "Drop";
        case 0x0027:
          return "Park";
        case 0x0028:
          return "Forward Calls";
        case 0x0029:
          return "Alternate Function";
        case 0x002a:
          return "Line";
        case 0x002b:
          return "Speaker Phone";
        case 0x002c:
          return "Conference";
        case 0x002d:
          return "Ring Enable";
        case 0x002e:
          return "Ring Select";
        case 0x002f:
          return "Phone Mute";
        case 0x0030:
          return "Caller ID";
        case 0x0031:
          return "Send";
        case 0x0050:
          return "Speed Dial";
        case 0x0051:
          return "Store Number";
        case 0x0052:
          return "Recall Number";
        case 0x0053:
          return "Phone Directory";
        case 0x0070:
          return "Voice Mail";
        case 0x0071:
          return "Screen Calls";
        case 0x0072:
          return "Do Not Disturb";
        case 0x0073:
          return "Message";
        case 0x0074:
          return "Answer On/Off";
        case 0x0090:
          return "Inside Dial Tone";
        case 0x0091:
          return "Outside Dial Tone";
        case 0x0092:
          return "Inside Ring Tone";
        case 0x0093:
          return "Outside Ring Tone";
        case 0x0094:
          return "Priority Ring Tone";
        case 0x0095:
          return "Inside Ringback";
        case 0x0096:
          return "Priority Ringback";
        case 0x0097:
          return "Line Busy Tone";
        case 0x0098:
          return "Reorder Tone";
        case 0x0099:
          return "Call Waiting Tone";
        case 0x009a:
          return "Confirmation Tone 1";
        case 0x009b:
          return "Confirmation Tone 2";
        case 0x009c:
          return "Tones Off";
        case 0x009d:
          return "Outside Ringback";
        case 0x009e:
          return "Ringer";
        case 0x00b0:
          return "Phone Key 0";
        case 0x00b1:
          return "Phone Key 1";
        case 0x00b2:
          return "Phone Key 2";
        case 0x00b3:
          return "Phone Key 3";
        case 0x00b4:
          return "Phone Key 4";
        case 0x00b5:
          return "Phone Key 5";
        case 0x00b6:
          return "Phone Key 6";
        case 0x00b7:
          return "Phone Key 7";
        case 0x00b8:
          return "Phone Key 8";
        case 0x00b9:
          return "Phone Key 9";
        case 0x00ba:
          return "Phone Key Star";
        case 0x00bb:
          return "Phone Key Pound";
        case 0x00bc:
          return "Phone Key A";
        case 0x00bd:
          return "Phone Key B";
        case 0x00be:
          return "Phone Key C";
        case 0x00bf:
          return "Phone Key D";
        case 0x00c0:
          return "Phone Call History Key";
        case 0x00c1:
          return "Phone Caller ID Key";
        case 0x00c2:
          return "Phone Settings Key";
        case 0x00f0:
          return "Host Control";
        case 0x00f1:
          return "Host Available";
        case 0x00f2:
          return "Host Call Active";
        case 0x00f3:
          return "Activate Handset Audio";
        case 0x00f4:
          return "Ring Type";
        case 0x00f5:
          return "Re-dialable Phone Number";
        case 0x00f8:
          return "Stop Ring Tone";
        case 0x00f9:
          return "PSTN Ring Tone";
        case 0x00fa:
          return "Host Ring Tone";
        case 0x00fb:
          return "Alert Sound Error";
        case 0x00fc:
          return "Alert Sound Confirm";
        case 0x00fd:
          return "Alert Sound Notification";
        case 0x00fe:
          return "Silent Ring";
        case 0x0108:
          return "Email Message Waiting";
        case 0x0109:
          return "Voicemail Message Waiting";
        case 0x010a:
          return "Host Hold";
        case 0x0110:
          return "Incoming Call History Count";
        case 0x0111:
          return "Outgoing Call History Count";
        case 0x0112:
          return "Incoming Call History";
        case 0x0113:
          return "Outgoing Call History";
        case 0x0114:
          return "Phone Locale";
        case 0x0140:
          return "Phone Time Second";
        case 0x0141:
          return "Phone Time Minute";
        case 0x0142:
          return "Phone Time Hour";
        case 0x0143:
          return "Phone Date Day";
        case 0x0144:
          return "Phone Date Month";
        case 0x0145:
          return "Phone Date Year";
        case 0x0146:
          return "Handset Nickname";
        case 0x0147:
          return "Address Book ID";
        case 0x014a:
          return "Call Duration";
        case 0x014b:
          return "Dual Mode Phone";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class telephony : std::uint16_t {
  PHONE = 0x0001,
  ANSWERING_MACHINE = 0x0002,
  MESSAGE_CONTROLS = 0x0003,
  HANDSET = 0x0004,
  HEADSET = 0x0005,
  TELEPHONY_KEY_PAD = 0x0006,
  PROGRAMMABLE_BUTTON = 0x0007,
  HOOK_SWITCH = 0x0020,
  FLASH = 0x0021,
  FEATURE = 0x0022,
  HOLD = 0x0023,
  REDIAL = 0x0024,
  TRANSFER = 0x0025,
  DROP = 0x0026,
  PARK = 0x0027,
  FORWARD_CALLS = 0x0028,
  ALTERNATE_FUNCTION = 0x0029,
  LINE = 0x002a,
  SPEAKER_PHONE = 0x002b,
  CONFERENCE = 0x002c,
  RING_ENABLE = 0x002d,
  RING_SELECT = 0x002e,
  PHONE_MUTE = 0x002f,
  CALLER_ID = 0x0030,
  SEND = 0x0031,
  SPEED_DIAL = 0x0050,
  STORE_NUMBER = 0x0051,
  RECALL_NUMBER = 0x0052,
  PHONE_DIRECTORY = 0x0053,
  VOICE_MAIL = 0x0070,
  SCREEN_CALLS = 0x0071,
  DO_NOT_DISTURB = 0x0072,
  MESSAGE = 0x0073,
  ANSWER_ON_OFF = 0x0074,
  INSIDE_DIAL_TONE = 0x0090,
  OUTSIDE_DIAL_TONE = 0x0091,
  INSIDE_RING_TONE = 0x0092,
  OUTSIDE_RING_TONE = 0x0093,
  PRIORITY_RING_TONE = 0x0094,
  INSIDE_RINGBACK = 0x0095,
  PRIORITY_RINGBACK = 0x0096,
  LINE_BUSY_TONE = 0x0097,
  REORDER_TONE = 0x0098,
  CALL_WAITING_TONE = 0x0099,
  CONFIRMATION_TONE_1 = 0x009a,
  CONFIRMATION_TONE_2 = 0x009b,
  TONES_OFF = 0x009c,
  OUTSIDE_RINGBACK = 0x009d,
  RINGER = 0x009e,
  PHONE_KEY_0 = 0x00b0,
  PHONE_KEY_1 = 0x00b1,
  PHONE_KEY_2 = 0x00b2,
  PHONE_KEY_3 = 0x00b3,
  PHONE_KEY_4 = 0x00b4,
  PHONE_KEY_5 = 0x00b5,
  PHONE_KEY_6 = 0x00b6,
  PHONE_KEY_7 = 0x00b7,
  PHONE_KEY_8 = 0x00b8,
  PHONE_KEY_9 = 0x00b9,
  PHONE_KEY_STAR = 0x00ba,
  PHONE_KEY_POUND = 0x00bb,
  PHONE_KEY_A = 0x00bc,
  PHONE_KEY_B = 0x00bd,
  PHONE_KEY_C = 0x00be,
  PHONE_KEY_D = 0x00bf,
  PHONE_CALL_HISTORY_KEY = 0x00c0,
  PHONE_CALLER_ID_KEY = 0x00c1,
  PHONE_SETTINGS_KEY = 0x00c2,
  HOST_CONTROL = 0x00f0,
  HOST_AVAILABLE = 0x00f1,
  HOST_CALL_ACTIVE = 0x00f2,
  ACTIVATE_HANDSET_AUDIO = 0x00f3,
  RING_TYPE = 0x00f4,
  RE_DIALABLE_PHONE_NUMBER = 0x00f5,
  STOP_RING_TONE = 0x00f8,
  PSTN_RING_TONE = 0x00f9,
  HOST_RING_TONE = 0x00fa,
  ALERT_SOUND_ERROR = 0x00fb,
  ALERT_SOUND_CONFIRM = 0x00fc,
  ALERT_SOUND_NOTIFICATION = 0x00fd,
  SILENT_RING = 0x00fe,
  EMAIL_MESSAGE_WAITING = 0x0108,
  VOICEMAIL_MESSAGE_WAITING = 0x0109,
  HOST_HOLD = 0x010a,
  INCOMING_CALL_HISTORY_COUNT = 0x0110,
  OUTGOING_CALL_HISTORY_COUNT = 0x0111,
  INCOMING_CALL_HISTORY = 0x0112,
  OUTGOING_CALL_HISTORY = 0x0113,
  PHONE_LOCALE = 0x0114,
  PHONE_TIME_SECOND = 0x0140,
  PHONE_TIME_MINUTE = 0x0141,
  PHONE_TIME_HOUR = 0x0142,
  PHONE_DATE_DAY = 0x0143,
  PHONE_DATE_MONTH = 0x0144,
  PHONE_DATE_YEAR = 0x0145,
  HANDSET_NICKNAME = 0x0146,
  ADDRESS_BOOK_ID = 0x0147,
  CALL_DURATION = 0x014a,
  DUAL_MODE_PHONE = 0x014b,
};
} // namespace hid::page

#endif // __HID_PAGE_TELEPHONY_HPP_

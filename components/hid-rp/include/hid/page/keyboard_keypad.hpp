#ifndef __HID_PAGE_KEYBOARD_KEYPAD_HPP_
#define __HID_PAGE_KEYBOARD_KEYPAD_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class keyboard_keypad : std::uint8_t;
template <> constexpr inline auto get_info<keyboard_keypad>() {
  return info(
      0x0007, 0x00e7, "Keyboard/Keypad",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Error RollOver";
        case 0x0002:
          return "POST Fail";
        case 0x0003:
          return "Error Undefined";
        case 0x0004:
          return "Keyboard A";
        case 0x0005:
          return "Keyboard B";
        case 0x0006:
          return "Keyboard C";
        case 0x0007:
          return "Keyboard D";
        case 0x0008:
          return "Keyboard E";
        case 0x0009:
          return "Keyboard F";
        case 0x000a:
          return "Keyboard G";
        case 0x000b:
          return "Keyboard H";
        case 0x000c:
          return "Keyboard I";
        case 0x000d:
          return "Keyboard J";
        case 0x000e:
          return "Keyboard K";
        case 0x000f:
          return "Keyboard L";
        case 0x0010:
          return "Keyboard M";
        case 0x0011:
          return "Keyboard N";
        case 0x0012:
          return "Keyboard O";
        case 0x0013:
          return "Keyboard P";
        case 0x0014:
          return "Keyboard Q";
        case 0x0015:
          return "Keyboard R";
        case 0x0016:
          return "Keyboard S";
        case 0x0017:
          return "Keyboard T";
        case 0x0018:
          return "Keyboard U";
        case 0x0019:
          return "Keyboard V";
        case 0x001a:
          return "Keyboard W";
        case 0x001b:
          return "Keyboard X";
        case 0x001c:
          return "Keyboard Y";
        case 0x001d:
          return "Keyboard Z";
        case 0x001e:
          return "Keyboard 1 ! (Bang)";
        case 0x001f:
          return "Keyboard 2 @ (At)";
        case 0x0020:
          return "Keyboard 3 # (Hash)";
        case 0x0021:
          return "Keyboard 4 $ (Dollar)";
        case 0x0022:
          return "Keyboard 5 % (Percent)";
        case 0x0023:
          return "Keyboard 6 ^ (Caret)";
        case 0x0024:
          return "Keyboard 7 & (Ampersand)";
        case 0x0025:
          return "Keyboard 8 * (Star)";
        case 0x0026:
          return "Keyboard 9 ( (Left Bracket)";
        case 0x0027:
          return "Keyboard 0 ) (Right Bracket)";
        case 0x0028:
          return "Keyboard Enter";
        case 0x0029:
          return "Keyboard Escape";
        case 0x002a:
          return "Keyboard Backspace";
        case 0x002b:
          return "Keyboard Tab";
        case 0x002c:
          return "Keyboard Space";
        case 0x002d:
          return "Keyboard - _ (Dash Underscore)";
        case 0x002e:
          return "Keyboard = + (Equals Plus)";
        case 0x002f:
          return "Keyboard [ { (Left Brace)";
        case 0x0030:
          return "Keyboard ] } (Right Brace)";
        case 0x0031:
          return "Keyboard \\ | (Backslash Pipe)";
        case 0x0032:
          return "Keyboard # ~ (Non-US Hash Tilde)";
        case 0x0033:
          return "Keyboard ; : (Semicolon Colon)";
        case 0x0034:
          return "Keyboard ' \" (Left Apos Double)";
        case 0x0035:
          return "Keyboard ` ´ (Grave Accent Tilde)";
        case 0x0036:
          return "Keyboard , < (Comma Less)";
        case 0x0037:
          return "Keyboard . > (Period Greater)";
        case 0x0038:
          return "Keyboard / ? (ForwardSlash QuestionMark)";
        case 0x0039:
          return "Keyboard Caps Lock";
        case 0x003a:
          return "Keyboard F1";
        case 0x003b:
          return "Keyboard F2";
        case 0x003c:
          return "Keyboard F3";
        case 0x003d:
          return "Keyboard F4";
        case 0x003e:
          return "Keyboard F5";
        case 0x003f:
          return "Keyboard F6";
        case 0x0040:
          return "Keyboard F7";
        case 0x0041:
          return "Keyboard F8";
        case 0x0042:
          return "Keyboard F9";
        case 0x0043:
          return "Keyboard F10";
        case 0x0044:
          return "Keyboard F11";
        case 0x0045:
          return "Keyboard F12";
        case 0x0046:
          return "Keyboard PrintScreen";
        case 0x0047:
          return "Keyboard Scroll Lock";
        case 0x0048:
          return "Keyboard Pause";
        case 0x0049:
          return "Keyboard Insert";
        case 0x004a:
          return "Keyboard Home";
        case 0x004b:
          return "Keyboard PageUp";
        case 0x004c:
          return "Keyboard Delete";
        case 0x004d:
          return "Keyboard End";
        case 0x004e:
          return "Keyboard PageDown";
        case 0x004f:
          return "Keyboard RightArrow";
        case 0x0050:
          return "Keyboard LeftArrow";
        case 0x0051:
          return "Keyboard DownArrow";
        case 0x0052:
          return "Keyboard UpArrow";
        case 0x0053:
          return "Keypad Num Lock and Clear";
        case 0x0054:
          return "Keypad / (ForwardSlash)";
        case 0x0055:
          return "Keypad * (Star)";
        case 0x0056:
          return "Keypad - (Dash)";
        case 0x0057:
          return "Keypad + (Plus)";
        case 0x0058:
          return "Keypad Enter";
        case 0x0059:
          return "Keypad 1 End";
        case 0x005a:
          return "Keypad 2 Down Arrow";
        case 0x005b:
          return "Keypad 3 PageDn";
        case 0x005c:
          return "Keypad 4 Left Arrow";
        case 0x005d:
          return "Keypad 5";
        case 0x005e:
          return "Keypad 6 Right Arrow";
        case 0x005f:
          return "Keypad 7 Home";
        case 0x0060:
          return "Keypad 8 Up Arrow";
        case 0x0061:
          return "Keypad 9 PageUp";
        case 0x0062:
          return "Keypad 0 Insert";
        case 0x0063:
          return "Keypad . Delete";
        case 0x0064:
          return "Keyboard \\ | (Non-US Backslash Pipe)";
        case 0x0065:
          return "Keyboard Application";
        case 0x0066:
          return "Keyboard Power";
        case 0x0067:
          return "Keypad = (Equals)";
        case 0x0068:
          return "Keyboard F13";
        case 0x0069:
          return "Keyboard F14";
        case 0x006a:
          return "Keyboard F15";
        case 0x006b:
          return "Keyboard F16";
        case 0x006c:
          return "Keyboard F17";
        case 0x006d:
          return "Keyboard F18";
        case 0x006e:
          return "Keyboard F19";
        case 0x006f:
          return "Keyboard F20";
        case 0x0070:
          return "Keyboard F21";
        case 0x0071:
          return "Keyboard F22";
        case 0x0072:
          return "Keyboard F23";
        case 0x0073:
          return "Keyboard F24";
        case 0x0074:
          return "Keyboard Execute";
        case 0x0075:
          return "Keyboard Help";
        case 0x0076:
          return "Keyboard Menu";
        case 0x0077:
          return "Keyboard Select";
        case 0x0078:
          return "Keyboard Stop";
        case 0x0079:
          return "Keyboard Again";
        case 0x007a:
          return "Keyboard Undo";
        case 0x007b:
          return "Keyboard Cut";
        case 0x007c:
          return "Keyboard Copy";
        case 0x007d:
          return "Keyboard Paste";
        case 0x007e:
          return "Keyboard Find";
        case 0x007f:
          return "Keyboard Mute";
        case 0x0080:
          return "Keyboard Volume Up";
        case 0x0081:
          return "Keyboard Volume Down";
        case 0x0082:
          return "Keyboard Locking Caps Lock";
        case 0x0083:
          return "Keyboard Locking Num Lock";
        case 0x0084:
          return "Keyboard Locking Scroll Lock";
        case 0x0085:
          return "Keypad Comma";
        case 0x0086:
          return "Keypad Equal Sign";
        case 0x0087:
          return "Keyboard International1";
        case 0x0088:
          return "Keyboard International2";
        case 0x0089:
          return "Keyboard International3";
        case 0x008a:
          return "Keyboard International4";
        case 0x008b:
          return "Keyboard International5";
        case 0x008c:
          return "Keyboard International6";
        case 0x008d:
          return "Keyboard International7";
        case 0x008e:
          return "Keyboard International8";
        case 0x008f:
          return "Keyboard International9";
        case 0x0090:
          return "Keyboard LANG1";
        case 0x0091:
          return "Keyboard LANG2";
        case 0x0092:
          return "Keyboard LANG3";
        case 0x0093:
          return "Keyboard LANG4";
        case 0x0094:
          return "Keyboard LANG5";
        case 0x0095:
          return "Keyboard LANG6";
        case 0x0096:
          return "Keyboard LANG7";
        case 0x0097:
          return "Keyboard LANG8";
        case 0x0098:
          return "Keyboard LANG9";
        case 0x0099:
          return "Keyboard Alternate Erase";
        case 0x009a:
          return "Keyboard SysReq Attention";
        case 0x009b:
          return "Keyboard Cancel";
        case 0x009c:
          return "Keyboard Clear";
        case 0x009d:
          return "Keyboard Prior";
        case 0x009e:
          return "Keyboard Return";
        case 0x009f:
          return "Keyboard Separator";
        case 0x00a0:
          return "Keyboard Out";
        case 0x00a1:
          return "Keyboard Oper";
        case 0x00a2:
          return "Keyboard Clear/Again";
        case 0x00a3:
          return "Keyboard CrSel/Props";
        case 0x00a4:
          return "Keyboard ExSel";
        case 0x00b0:
          return "Keypad 00";
        case 0x00b1:
          return "Keypad 000";
        case 0x00b2:
          return "Thousands Separator";
        case 0x00b3:
          return "Decimal Separator";
        case 0x00b4:
          return "Currency Unit";
        case 0x00b5:
          return "Currency Sub-unit";
        case 0x00b6:
          return "Keypad ( (Left Bracket)";
        case 0x00b7:
          return "Keypad ) (Right Bracket)";
        case 0x00b8:
          return "Keypad { (Left Brace)";
        case 0x00b9:
          return "Keypad } (Right Brace)";
        case 0x00ba:
          return "Keypad Tab";
        case 0x00bb:
          return "Keypad Backspace";
        case 0x00bc:
          return "Keypad A";
        case 0x00bd:
          return "Keypad B";
        case 0x00be:
          return "Keypad C";
        case 0x00bf:
          return "Keypad D";
        case 0x00c0:
          return "Keypad E";
        case 0x00c1:
          return "Keypad F";
        case 0x00c2:
          return "Keypad XOR";
        case 0x00c3:
          return "Keypad ^ (Caret)";
        case 0x00c4:
          return "Keypad % (Percent)";
        case 0x00c5:
          return "Keypad < (Less)";
        case 0x00c6:
          return "Keypad > (Greater)";
        case 0x00c7:
          return "Keypad & (Ampersand)";
        case 0x00c8:
          return "Keypad && (Double Ampersand)";
        case 0x00c9:
          return "Keypad | (Bar)";
        case 0x00ca:
          return "Keypad || (Double Bar)";
        case 0x00cb:
          return "Keypad : (Colon)";
        case 0x00cc:
          return "Keypad # (Hash)";
        case 0x00cd:
          return "Keypad Space";
        case 0x00ce:
          return "Keypad @ (At)";
        case 0x00cf:
          return "Keypad ! (Bang)";
        case 0x00d0:
          return "Keypad Memory Store";
        case 0x00d1:
          return "Keypad Memory Recall";
        case 0x00d2:
          return "Keypad Memory Clear";
        case 0x00d3:
          return "Keypad Memory Add";
        case 0x00d4:
          return "Keypad Memory Subtract";
        case 0x00d5:
          return "Keypad Memory Multiply";
        case 0x00d6:
          return "Keypad Memory Divide";
        case 0x00d7:
          return "Keypad +/- (Plus Minus)";
        case 0x00d8:
          return "Keypad Clear";
        case 0x00d9:
          return "Keypad Clear Entry";
        case 0x00da:
          return "Keypad Binary";
        case 0x00db:
          return "Keypad Octal";
        case 0x00dc:
          return "Keypad Decimal";
        case 0x00dd:
          return "Keypad Hexadecimal";
        case 0x00e0:
          return "Keyboard Left Control";
        case 0x00e1:
          return "Keyboard Left Shift";
        case 0x00e2:
          return "Keyboard Left Alt";
        case 0x00e3:
          return "Keyboard Left GUI";
        case 0x00e4:
          return "Keyboard Right Control";
        case 0x00e5:
          return "Keyboard Right Shift";
        case 0x00e6:
          return "Keyboard Right Alt";
        case 0x00e7:
          return "Keyboard Right GUI";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class keyboard_keypad : std::uint8_t {
  ERROR_ROLLOVER = 0x0001,
  POST_FAIL = 0x0002,
  ERROR_UNDEFINED = 0x0003,
  KEYBOARD_A = 0x0004,
  KEYBOARD_B = 0x0005,
  KEYBOARD_C = 0x0006,
  KEYBOARD_D = 0x0007,
  KEYBOARD_E = 0x0008,
  KEYBOARD_F = 0x0009,
  KEYBOARD_G = 0x000a,
  KEYBOARD_H = 0x000b,
  KEYBOARD_I = 0x000c,
  KEYBOARD_J = 0x000d,
  KEYBOARD_K = 0x000e,
  KEYBOARD_L = 0x000f,
  KEYBOARD_M = 0x0010,
  KEYBOARD_N = 0x0011,
  KEYBOARD_O = 0x0012,
  KEYBOARD_P = 0x0013,
  KEYBOARD_Q = 0x0014,
  KEYBOARD_R = 0x0015,
  KEYBOARD_S = 0x0016,
  KEYBOARD_T = 0x0017,
  KEYBOARD_U = 0x0018,
  KEYBOARD_V = 0x0019,
  KEYBOARD_W = 0x001a,
  KEYBOARD_X = 0x001b,
  KEYBOARD_Y = 0x001c,
  KEYBOARD_Z = 0x001d,
  KEYBOARD_1_BANG = 0x001e,
  KEYBOARD_2_AT = 0x001f,
  KEYBOARD_3_HASH = 0x0020,
  KEYBOARD_4_DOLLAR = 0x0021,
  KEYBOARD_5_PERCENT = 0x0022,
  KEYBOARD_6_CARET = 0x0023,
  KEYBOARD_7_AMPERSAND = 0x0024,
  KEYBOARD_8_STAR = 0x0025,
  KEYBOARD_9_LEFT_BRACKET = 0x0026,
  KEYBOARD_0_RIGHT_BRACKET = 0x0027,
  KEYBOARD_ENTER = 0x0028,
  KEYBOARD_ESCAPE = 0x0029,
  KEYBOARD_BACKSPACE = 0x002a,
  KEYBOARD_TAB = 0x002b,
  KEYBOARD_SPACE = 0x002c,
  KEYBOARD_DASH_UNDERSCORE = 0x002d,
  KEYBOARD_EQUALS_PLUS = 0x002e,
  KEYBOARD_LEFT_BRACE = 0x002f,
  KEYBOARD_RIGHT_BRACE = 0x0030,
  KEYBOARD_BACKSLASH_PIPE = 0x0031,
  KEYBOARD_NON_US_HASH_TILDE = 0x0032,
  KEYBOARD_SEMICOLON_COLON = 0x0033,
  KEYBOARD_LEFT_APOS_DOUBLE = 0x0034,
  KEYBOARD_GRAVE_ACCENT_TILDE = 0x0035,
  KEYBOARD_COMMA_LESS = 0x0036,
  KEYBOARD_PERIOD_GREATER = 0x0037,
  KEYBOARD_FORWARDSLASH_QUESTIONMARK = 0x0038,
  KEYBOARD_CAPS_LOCK = 0x0039,
  KEYBOARD_F1 = 0x003a,
  KEYBOARD_F2 = 0x003b,
  KEYBOARD_F3 = 0x003c,
  KEYBOARD_F4 = 0x003d,
  KEYBOARD_F5 = 0x003e,
  KEYBOARD_F6 = 0x003f,
  KEYBOARD_F7 = 0x0040,
  KEYBOARD_F8 = 0x0041,
  KEYBOARD_F9 = 0x0042,
  KEYBOARD_F10 = 0x0043,
  KEYBOARD_F11 = 0x0044,
  KEYBOARD_F12 = 0x0045,
  KEYBOARD_PRINTSCREEN = 0x0046,
  KEYBOARD_SCROLL_LOCK = 0x0047,
  KEYBOARD_PAUSE = 0x0048,
  KEYBOARD_INSERT = 0x0049,
  KEYBOARD_HOME = 0x004a,
  KEYBOARD_PAGEUP = 0x004b,
  KEYBOARD_DELETE = 0x004c,
  KEYBOARD_END = 0x004d,
  KEYBOARD_PAGEDOWN = 0x004e,
  KEYBOARD_RIGHTARROW = 0x004f,
  KEYBOARD_LEFTARROW = 0x0050,
  KEYBOARD_DOWNARROW = 0x0051,
  KEYBOARD_UPARROW = 0x0052,
  KEYPAD_NUM_LOCK_AND_CLEAR = 0x0053,
  KEYPAD_FORWARDSLASH = 0x0054,
  KEYPAD_STAR = 0x0055,
  KEYPAD_DASH = 0x0056,
  KEYPAD_PLUS = 0x0057,
  KEYPAD_ENTER = 0x0058,
  KEYPAD_1_END = 0x0059,
  KEYPAD_2_DOWN_ARROW = 0x005a,
  KEYPAD_3_PAGEDN = 0x005b,
  KEYPAD_4_LEFT_ARROW = 0x005c,
  KEYPAD_5 = 0x005d,
  KEYPAD_6_RIGHT_ARROW = 0x005e,
  KEYPAD_7_HOME = 0x005f,
  KEYPAD_8_UP_ARROW = 0x0060,
  KEYPAD_9_PAGEUP = 0x0061,
  KEYPAD_0_INSERT = 0x0062,
  KEYPAD_DELETE = 0x0063,
  KEYBOARD_NON_US_BACKSLASH_PIPE = 0x0064,
  KEYBOARD_APPLICATION = 0x0065,
  KEYBOARD_POWER = 0x0066,
  KEYPAD_EQUALS = 0x0067,
  KEYBOARD_F13 = 0x0068,
  KEYBOARD_F14 = 0x0069,
  KEYBOARD_F15 = 0x006a,
  KEYBOARD_F16 = 0x006b,
  KEYBOARD_F17 = 0x006c,
  KEYBOARD_F18 = 0x006d,
  KEYBOARD_F19 = 0x006e,
  KEYBOARD_F20 = 0x006f,
  KEYBOARD_F21 = 0x0070,
  KEYBOARD_F22 = 0x0071,
  KEYBOARD_F23 = 0x0072,
  KEYBOARD_F24 = 0x0073,
  KEYBOARD_EXECUTE = 0x0074,
  KEYBOARD_HELP = 0x0075,
  KEYBOARD_MENU = 0x0076,
  KEYBOARD_SELECT = 0x0077,
  KEYBOARD_STOP = 0x0078,
  KEYBOARD_AGAIN = 0x0079,
  KEYBOARD_UNDO = 0x007a,
  KEYBOARD_CUT = 0x007b,
  KEYBOARD_COPY = 0x007c,
  KEYBOARD_PASTE = 0x007d,
  KEYBOARD_FIND = 0x007e,
  KEYBOARD_MUTE = 0x007f,
  KEYBOARD_VOLUME_UP = 0x0080,
  KEYBOARD_VOLUME_DOWN = 0x0081,
  KEYBOARD_LOCKING_CAPS_LOCK = 0x0082,
  KEYBOARD_LOCKING_NUM_LOCK = 0x0083,
  KEYBOARD_LOCKING_SCROLL_LOCK = 0x0084,
  KEYPAD_COMMA = 0x0085,
  KEYPAD_EQUAL_SIGN = 0x0086,
  KEYBOARD_INTERNATIONAL1 = 0x0087,
  KEYBOARD_INTERNATIONAL2 = 0x0088,
  KEYBOARD_INTERNATIONAL3 = 0x0089,
  KEYBOARD_INTERNATIONAL4 = 0x008a,
  KEYBOARD_INTERNATIONAL5 = 0x008b,
  KEYBOARD_INTERNATIONAL6 = 0x008c,
  KEYBOARD_INTERNATIONAL7 = 0x008d,
  KEYBOARD_INTERNATIONAL8 = 0x008e,
  KEYBOARD_INTERNATIONAL9 = 0x008f,
  KEYBOARD_LANG1 = 0x0090,
  KEYBOARD_LANG2 = 0x0091,
  KEYBOARD_LANG3 = 0x0092,
  KEYBOARD_LANG4 = 0x0093,
  KEYBOARD_LANG5 = 0x0094,
  KEYBOARD_LANG6 = 0x0095,
  KEYBOARD_LANG7 = 0x0096,
  KEYBOARD_LANG8 = 0x0097,
  KEYBOARD_LANG9 = 0x0098,
  KEYBOARD_ALTERNATE_ERASE = 0x0099,
  KEYBOARD_SYSREQ_ATTENTION = 0x009a,
  KEYBOARD_CANCEL = 0x009b,
  KEYBOARD_CLEAR = 0x009c,
  KEYBOARD_PRIOR = 0x009d,
  KEYBOARD_RETURN = 0x009e,
  KEYBOARD_SEPARATOR = 0x009f,
  KEYBOARD_OUT = 0x00a0,
  KEYBOARD_OPER = 0x00a1,
  KEYBOARD_CLEAR_AGAIN = 0x00a2,
  KEYBOARD_CRSEL_PROPS = 0x00a3,
  KEYBOARD_EXSEL = 0x00a4,
  KEYPAD_00 = 0x00b0,
  KEYPAD_000 = 0x00b1,
  THOUSANDS_SEPARATOR = 0x00b2,
  DECIMAL_SEPARATOR = 0x00b3,
  CURRENCY_UNIT = 0x00b4,
  CURRENCY_SUB_UNIT = 0x00b5,
  KEYPAD_LEFT_BRACKET = 0x00b6,
  KEYPAD_RIGHT_BRACKET = 0x00b7,
  KEYPAD_LEFT_BRACE = 0x00b8,
  KEYPAD_RIGHT_BRACE = 0x00b9,
  KEYPAD_TAB = 0x00ba,
  KEYPAD_BACKSPACE = 0x00bb,
  KEYPAD_A = 0x00bc,
  KEYPAD_B = 0x00bd,
  KEYPAD_C = 0x00be,
  KEYPAD_D = 0x00bf,
  KEYPAD_E = 0x00c0,
  KEYPAD_F = 0x00c1,
  KEYPAD_XOR = 0x00c2,
  KEYPAD_CARET = 0x00c3,
  KEYPAD_PERCENT = 0x00c4,
  KEYPAD_LESS = 0x00c5,
  KEYPAD_GREATER = 0x00c6,
  KEYPAD_AMPERSAND = 0x00c7,
  KEYPAD_DOUBLE_AMPERSAND = 0x00c8,
  KEYPAD_BAR = 0x00c9,
  KEYPAD_DOUBLE_BAR = 0x00ca,
  KEYPAD_COLON = 0x00cb,
  KEYPAD_HASH = 0x00cc,
  KEYPAD_SPACE = 0x00cd,
  KEYPAD_AT = 0x00ce,
  KEYPAD_BANG = 0x00cf,
  KEYPAD_MEMORY_STORE = 0x00d0,
  KEYPAD_MEMORY_RECALL = 0x00d1,
  KEYPAD_MEMORY_CLEAR = 0x00d2,
  KEYPAD_MEMORY_ADD = 0x00d3,
  KEYPAD_MEMORY_SUBTRACT = 0x00d4,
  KEYPAD_MEMORY_MULTIPLY = 0x00d5,
  KEYPAD_MEMORY_DIVIDE = 0x00d6,
  KEYPAD_PLUS_MINUS = 0x00d7,
  KEYPAD_CLEAR = 0x00d8,
  KEYPAD_CLEAR_ENTRY = 0x00d9,
  KEYPAD_BINARY = 0x00da,
  KEYPAD_OCTAL = 0x00db,
  KEYPAD_DECIMAL = 0x00dc,
  KEYPAD_HEXADECIMAL = 0x00dd,
  KEYBOARD_LEFT_CONTROL = 0x00e0,
  KEYBOARD_LEFT_SHIFT = 0x00e1,
  KEYBOARD_LEFT_ALT = 0x00e2,
  KEYBOARD_LEFT_GUI = 0x00e3,
  KEYBOARD_RIGHT_CONTROL = 0x00e4,
  KEYBOARD_RIGHT_SHIFT = 0x00e5,
  KEYBOARD_RIGHT_ALT = 0x00e6,
  KEYBOARD_RIGHT_GUI = 0x00e7,
};
} // namespace hid::page

#endif // __HID_PAGE_KEYBOARD_KEYPAD_HPP_

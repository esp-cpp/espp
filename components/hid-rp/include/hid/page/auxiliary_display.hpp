#ifndef __HID_PAGE_AUXILIARY_DISPLAY_HPP_
#define __HID_PAGE_AUXILIARY_DISPLAY_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class auxiliary_display : std::uint8_t;
template <> constexpr inline auto get_info<auxiliary_display>() {
  return info(
      0x0014, 0x00ff, "Auxiliary Display",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Alphanumeric Display";
        case 0x0002:
          return "Auxiliary Display";
        case 0x0020:
          return "Display Attributes Report";
        case 0x0021:
          return "ASCII Character Set";
        case 0x0022:
          return "Data Read Back";
        case 0x0023:
          return "Font Read Back";
        case 0x0024:
          return "Display Control Report";
        case 0x0025:
          return "Clear Display";
        case 0x0026:
          return "Display Enable";
        case 0x0027:
          return "Screen Saver Delay";
        case 0x0028:
          return "Screen Saver Enable";
        case 0x0029:
          return "Vertical Scroll";
        case 0x002a:
          return "Horizontal Scroll";
        case 0x002b:
          return "Character Report";
        case 0x002c:
          return "Display Data";
        case 0x002d:
          return "Display Status";
        case 0x002e:
          return "Stat Not Ready";
        case 0x002f:
          return "Stat Ready";
        case 0x0030:
          return "Err Not a loadable character";
        case 0x0031:
          return "Err Font data cannot be read";
        case 0x0032:
          return "Cursor Position Report";
        case 0x0033:
          return "Row";
        case 0x0034:
          return "Column";
        case 0x0035:
          return "Rows";
        case 0x0036:
          return "Columns";
        case 0x0037:
          return "Cursor Pixel Positioning";
        case 0x0038:
          return "Cursor Mode";
        case 0x0039:
          return "Cursor Enable";
        case 0x003a:
          return "Cursor Blink";
        case 0x003b:
          return "Font Report";
        case 0x003c:
          return "Font Data";
        case 0x003d:
          return "Character Width";
        case 0x003e:
          return "Character Height";
        case 0x003f:
          return "Character Spacing Horizontal";
        case 0x0040:
          return "Character Spacing Vertical";
        case 0x0041:
          return "Unicode Character Set";
        case 0x0042:
          return "Font 7-Segment";
        case 0x0043:
          return "7-Segment Direct Map";
        case 0x0044:
          return "Font 14-Segment";
        case 0x0045:
          return "14-Segment Direct Map";
        case 0x0046:
          return "Display Brightness";
        case 0x0047:
          return "Display Contrast";
        case 0x0048:
          return "Character Attribute";
        case 0x0049:
          return "Attribute Readback";
        case 0x004a:
          return "Attribute Data";
        case 0x004b:
          return "Char Attr Enhance";
        case 0x004c:
          return "Char Attr Underline";
        case 0x004d:
          return "Char Attr Blink";
        case 0x0080:
          return "Bitmap Size X";
        case 0x0081:
          return "Bitmap Size Y";
        case 0x0082:
          return "Max Blit Size";
        case 0x0083:
          return "Bit Depth Format";
        case 0x0084:
          return "Display Orientation";
        case 0x0085:
          return "Palette Report";
        case 0x0086:
          return "Palette Data Size";
        case 0x0087:
          return "Palette Data Offset";
        case 0x0088:
          return "Palette Data";
        case 0x008a:
          return "Blit Report";
        case 0x008b:
          return "Blit Rectangle X1";
        case 0x008c:
          return "Blit Rectangle Y1";
        case 0x008d:
          return "Blit Rectangle X2";
        case 0x008e:
          return "Blit Rectangle Y2";
        case 0x008f:
          return "Blit Data";
        case 0x0090:
          return "Soft Button";
        case 0x0091:
          return "Soft Button ID";
        case 0x0092:
          return "Soft Button Side";
        case 0x0093:
          return "Soft Button Offset 1";
        case 0x0094:
          return "Soft Button Offset 2";
        case 0x0095:
          return "Soft Button Report";
        case 0x00c2:
          return "Soft Keys";
        case 0x00cc:
          return "Display Data Extensions";
        case 0x00cf:
          return "Character Mapping";
        case 0x00dd:
          return "Unicode Equivalent";
        case 0x00df:
          return "Character Page Mapping";
        case 0x00ff:
          return "Request Report";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class auxiliary_display : std::uint8_t {
  ALPHANUMERIC_DISPLAY = 0x0001,
  AUXILIARY_DISPLAY = 0x0002,
  DISPLAY_ATTRIBUTES_REPORT = 0x0020,
  ASCII_CHARACTER_SET = 0x0021,
  DATA_READ_BACK = 0x0022,
  FONT_READ_BACK = 0x0023,
  DISPLAY_CONTROL_REPORT = 0x0024,
  CLEAR_DISPLAY = 0x0025,
  DISPLAY_ENABLE = 0x0026,
  SCREEN_SAVER_DELAY = 0x0027,
  SCREEN_SAVER_ENABLE = 0x0028,
  VERTICAL_SCROLL = 0x0029,
  HORIZONTAL_SCROLL = 0x002a,
  CHARACTER_REPORT = 0x002b,
  DISPLAY_DATA = 0x002c,
  DISPLAY_STATUS = 0x002d,
  STAT_NOT_READY = 0x002e,
  STAT_READY = 0x002f,
  ERR_NOT_A_LOADABLE_CHARACTER = 0x0030,
  ERR_FONT_DATA_CANNOT_BE_READ = 0x0031,
  CURSOR_POSITION_REPORT = 0x0032,
  ROW = 0x0033,
  COLUMN = 0x0034,
  ROWS = 0x0035,
  COLUMNS = 0x0036,
  CURSOR_PIXEL_POSITIONING = 0x0037,
  CURSOR_MODE = 0x0038,
  CURSOR_ENABLE = 0x0039,
  CURSOR_BLINK = 0x003a,
  FONT_REPORT = 0x003b,
  FONT_DATA = 0x003c,
  CHARACTER_WIDTH = 0x003d,
  CHARACTER_HEIGHT = 0x003e,
  CHARACTER_SPACING_HORIZONTAL = 0x003f,
  CHARACTER_SPACING_VERTICAL = 0x0040,
  UNICODE_CHARACTER_SET = 0x0041,
  FONT_7_SEGMENT = 0x0042,
  _7_SEGMENT_DIRECT_MAP = 0x0043,
  FONT_14_SEGMENT = 0x0044,
  _14_SEGMENT_DIRECT_MAP = 0x0045,
  DISPLAY_BRIGHTNESS = 0x0046,
  DISPLAY_CONTRAST = 0x0047,
  CHARACTER_ATTRIBUTE = 0x0048,
  ATTRIBUTE_READBACK = 0x0049,
  ATTRIBUTE_DATA = 0x004a,
  CHAR_ATTR_ENHANCE = 0x004b,
  CHAR_ATTR_UNDERLINE = 0x004c,
  CHAR_ATTR_BLINK = 0x004d,
  BITMAP_SIZE_X = 0x0080,
  BITMAP_SIZE_Y = 0x0081,
  MAX_BLIT_SIZE = 0x0082,
  BIT_DEPTH_FORMAT = 0x0083,
  DISPLAY_ORIENTATION = 0x0084,
  PALETTE_REPORT = 0x0085,
  PALETTE_DATA_SIZE = 0x0086,
  PALETTE_DATA_OFFSET = 0x0087,
  PALETTE_DATA = 0x0088,
  BLIT_REPORT = 0x008a,
  BLIT_RECTANGLE_X1 = 0x008b,
  BLIT_RECTANGLE_Y1 = 0x008c,
  BLIT_RECTANGLE_X2 = 0x008d,
  BLIT_RECTANGLE_Y2 = 0x008e,
  BLIT_DATA = 0x008f,
  SOFT_BUTTON = 0x0090,
  SOFT_BUTTON_ID = 0x0091,
  SOFT_BUTTON_SIDE = 0x0092,
  SOFT_BUTTON_OFFSET_1 = 0x0093,
  SOFT_BUTTON_OFFSET_2 = 0x0094,
  SOFT_BUTTON_REPORT = 0x0095,
  SOFT_KEYS = 0x00c2,
  DISPLAY_DATA_EXTENSIONS = 0x00cc,
  CHARACTER_MAPPING = 0x00cf,
  UNICODE_EQUIVALENT = 0x00dd,
  CHARACTER_PAGE_MAPPING = 0x00df,
  REQUEST_REPORT = 0x00ff,
};
} // namespace hid::page

#endif // __HID_PAGE_AUXILIARY_DISPLAY_HPP_

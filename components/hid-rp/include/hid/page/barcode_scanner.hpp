#ifndef __HID_PAGE_BARCODE_SCANNER_HPP_
#define __HID_PAGE_BARCODE_SCANNER_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class barcode_scanner : std::uint16_t;
template <> constexpr inline auto get_info<barcode_scanner>() {
  return info(
      0x008c, 0x0121, "Barcode Scanner",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Barcode Badge Reader";
        case 0x0002:
          return "Barcode Scanner";
        case 0x0003:
          return "Dumb Bar Code Scanner";
        case 0x0004:
          return "Cordless Scanner Base";
        case 0x0005:
          return "Bar Code Scanner Cradle";
        case 0x0010:
          return "Attribute Report";
        case 0x0011:
          return "Settings Report";
        case 0x0012:
          return "Scanned Data Report";
        case 0x0013:
          return "Raw Scanned Data Report";
        case 0x0014:
          return "Trigger Report";
        case 0x0015:
          return "Status Report";
        case 0x0016:
          return "UPC/EAN Control Report";
        case 0x0017:
          return "EAN 2/3 Label Control Report";
        case 0x0018:
          return "Code 39 Control Report";
        case 0x0019:
          return "Interleaved 2 of 5 Control Report";
        case 0x001a:
          return "Standard 2 of 5 Control Report";
        case 0x001b:
          return "MSI Plessey Control Report";
        case 0x001c:
          return "Codabar Control Report";
        case 0x001d:
          return "Code 128 Control Report";
        case 0x001e:
          return "Misc 1D Control Report";
        case 0x001f:
          return "2D Control Report";
        case 0x0030:
          return "Aiming/Pointer Mode";
        case 0x0031:
          return "Bar Code Present Sensor";
        case 0x0032:
          return "Class 1A Laser";
        case 0x0033:
          return "Class 2 Laser";
        case 0x0034:
          return "Heater Present";
        case 0x0035:
          return "Contact Scanner";
        case 0x0036:
          return "Electronic Article Surveillance Notification";
        case 0x0037:
          return "Constant Electronic Article Surveillance";
        case 0x0038:
          return "Error Indication";
        case 0x0039:
          return "Fixed Beeper";
        case 0x003a:
          return "Good Decode Indication";
        case 0x003b:
          return "Hands Free Scanning";
        case 0x003c:
          return "Intrinsically Safe";
        case 0x003d:
          return "Class 1 Laser";
        case 0x003e:
          return "Long Range Scanner";
        case 0x003f:
          return "Mirror Speed Control";
        case 0x0040:
          return "Not On File Indication";
        case 0x0041:
          return "Programmable Beeper";
        case 0x0042:
          return "Triggerless";
        case 0x0043:
          return "Wand";
        case 0x0044:
          return "Water Resistant";
        case 0x0045:
          return "Multi-Range Scanner";
        case 0x0046:
          return "Proximity Sensor";
        case 0x004d:
          return "Fragment Decoding";
        case 0x004e:
          return "Scanner Read Confidence";
        case 0x004f:
          return "Data Prefix";
        case 0x0050:
          return "Prefix AIMI";
        case 0x0051:
          return "Prefix None";
        case 0x0052:
          return "Prefix Proprietary";
        case 0x0055:
          return "Active Time";
        case 0x0056:
          return "Aiming Laser Pattern";
        case 0x0057:
          return "Bar Code Present";
        case 0x0058:
          return "Beeper State";
        case 0x0059:
          return "Laser On Time";
        case 0x005a:
          return "Laser State";
        case 0x005b:
          return "Lockout Time";
        case 0x005c:
          return "Motor State";
        case 0x005d:
          return "Motor Timeout";
        case 0x005e:
          return "Power On Reset Scanner";
        case 0x005f:
          return "Prevent Read of Barcodes";
        case 0x0060:
          return "Initiate Barcode Read";
        case 0x0061:
          return "Trigger State";
        case 0x0062:
          return "Trigger Mode";
        case 0x0063:
          return "Trigger Mode Blinking Laser On";
        case 0x0064:
          return "Trigger Mode Continuous Laser On";
        case 0x0065:
          return "Trigger Mode Laser on while Pulled";
        case 0x0066:
          return "Trigger Mode Laser stays on after release";
        case 0x006d:
          return "Commit Parameters to NVM";
        case 0x006e:
          return "Parameter Scanning";
        case 0x006f:
          return "Parameters Changed";
        case 0x0070:
          return "Set parameter default values";
        case 0x0075:
          return "Scanner In Cradle";
        case 0x0076:
          return "Scanner In Range";
        case 0x007a:
          return "Aim Duration";
        case 0x007b:
          return "Good Read Lamp Duration";
        case 0x007c:
          return "Good Read Lamp Intensity";
        case 0x007d:
          return "Good Read LED";
        case 0x007e:
          return "Good Read Tone Frequency";
        case 0x007f:
          return "Good Read Tone Length";
        case 0x0080:
          return "Good Read Tone Volume";
        case 0x0082:
          return "No Read Message";
        case 0x0083:
          return "Not on File Volume";
        case 0x0084:
          return "Powerup Beep";
        case 0x0085:
          return "Sound Error Beep";
        case 0x0086:
          return "Sound Good Read Beep";
        case 0x0087:
          return "Sound Not On File Beep";
        case 0x0088:
          return "Good Read When to Write";
        case 0x0089:
          return "GRWTI After Decode";
        case 0x008a:
          return "GRWTI Beep/Lamp after transmit";
        case 0x008b:
          return "GRWTI No Beep/Lamp use at all";
        case 0x0091:
          return "Bookland EAN";
        case 0x0092:
          return "Convert EAN 8 to 13 Type";
        case 0x0093:
          return "Convert UPC A to EAN-13";
        case 0x0094:
          return "Convert UPC-E to A";
        case 0x0095:
          return "EAN-13";
        case 0x0096:
          return "EAN-8";
        case 0x0097:
          return "EAN-99 128 Mandatory";
        case 0x0098:
          return "EAN-99 P5/128 Optional";
        case 0x0099:
          return "Enable EAN Two Label";
        case 0x009a:
          return "UPC/EAN";
        case 0x009b:
          return "UPC/EAN Coupon Code";
        case 0x009c:
          return "UPC/EAN Periodicals";
        case 0x009d:
          return "UPC-A";
        case 0x009e:
          return "UPC-A with 128 Mandatory";
        case 0x009f:
          return "UPC-A with 128 Optional";
        case 0x00a0:
          return "UPC-A with P5 Optional";
        case 0x00a1:
          return "UPC-E";
        case 0x00a2:
          return "UPC-E1";
        case 0x00a9:
          return "Periodical";
        case 0x00aa:
          return "Periodical Auto-Discriminate +2";
        case 0x00ab:
          return "Periodical Only Decode with +2";
        case 0x00ac:
          return "Periodical Ignore +2";
        case 0x00ad:
          return "Periodical Auto-Discriminate +5";
        case 0x00ae:
          return "Periodical Only Decode with +5";
        case 0x00af:
          return "Periodical Ignore +5";
        case 0x00b0:
          return "Check";
        case 0x00b1:
          return "Check Disable Price";
        case 0x00b2:
          return "Check Enable 4 digit Price";
        case 0x00b3:
          return "Check Enable 5 digit Price";
        case 0x00b4:
          return "Check Enable European 4 digit Price";
        case 0x00b5:
          return "Check Enable European 5 digit Price";
        case 0x00b7:
          return "EAN Two Label";
        case 0x00b8:
          return "EAN Three Label";
        case 0x00b9:
          return "EAN 8 Flag Digit 1";
        case 0x00ba:
          return "EAN 8 Flag Digit 2";
        case 0x00bb:
          return "EAN 8 Flag Digit 3";
        case 0x00bc:
          return "EAN 13 Flag Digit 1";
        case 0x00bd:
          return "EAN 13 Flag Digit 2";
        case 0x00be:
          return "EAN 13 Flag Digit 3";
        case 0x00bf:
          return "Add EAN 2/3 Label Definition";
        case 0x00c0:
          return "Clear all EAN 2/3 Label Definitions";
        case 0x00c3:
          return "Codabar";
        case 0x00c4:
          return "Code 128";
        case 0x00c7:
          return "Code 39";
        case 0x00c8:
          return "Code 93";
        case 0x00c9:
          return "Full ASCII Conversion";
        case 0x00ca:
          return "Interleaved 2 of 5";
        case 0x00cb:
          return "Italian Pharmacy Code";
        case 0x00cc:
          return "MSI/Plessey";
        case 0x00cd:
          return "Standard 2 of 5 IATA";
        case 0x00ce:
          return "Standard 2 of 5";
        case 0x00d3:
          return "Transmit Start/Stop";
        case 0x00d4:
          return "Tri-Optic";
        case 0x00d5:
          return "UCC/EAN-128";
        case 0x00d6:
          return "Check Digit";
        case 0x00d7:
          return "Check Digit Disable";
        case 0x00d8:
          return "Check Digit Enable Interleaved 2 of 5 OPCC";
        case 0x00d9:
          return "Check Digit Enable Interleaved 2 of 5 USS";
        case 0x00da:
          return "Check Digit Enable Standard 2 of 5 OPCC";
        case 0x00db:
          return "Check Digit Enable Standard 2 of 5 USS";
        case 0x00dc:
          return "Check Digit Enable One MSI Plessey";
        case 0x00dd:
          return "Check Digit Enable Two MSI Plessey";
        case 0x00de:
          return "Check Digit Codabar Enable";
        case 0x00df:
          return "Check Digit Code 39 Enable";
        case 0x00f0:
          return "Transmit Check Digit";
        case 0x00f1:
          return "Disable Check Digit Transmit";
        case 0x00f2:
          return "Enable Check Digit Transmit";
        case 0x00fb:
          return "Symbology Identifier 1";
        case 0x00fc:
          return "Symbology Identifier 2";
        case 0x00fd:
          return "Symbology Identifier 3";
        case 0x00fe:
          return "Decoded Data";
        case 0x00ff:
          return "Decode Data Continued";
        case 0x0100:
          return "Bar Space Data";
        case 0x0101:
          return "Scanner Data Accuracy";
        case 0x0102:
          return "Raw Data Polarity";
        case 0x0103:
          return "Polarity Inverted Bar Code";
        case 0x0104:
          return "Polarity Normal Bar Code";
        case 0x0106:
          return "Minimum Length to Decode";
        case 0x0107:
          return "Maximum Length to Decode";
        case 0x0108:
          return "Discrete Length to Decode 1";
        case 0x0109:
          return "Discrete Length to Decode 2";
        case 0x010a:
          return "Data Length Method";
        case 0x010b:
          return "DL Method Read any";
        case 0x010c:
          return "DL Method Check in Range";
        case 0x010d:
          return "DL Method Check for Discrete";
        case 0x0110:
          return "Aztec Code";
        case 0x0111:
          return "BC412";
        case 0x0112:
          return "Channel Code";
        case 0x0113:
          return "Code 16";
        case 0x0114:
          return "Code 32";
        case 0x0115:
          return "Code 49";
        case 0x0116:
          return "Code One";
        case 0x0117:
          return "Colorcode";
        case 0x0118:
          return "Data Matrix";
        case 0x0119:
          return "MaxiCode";
        case 0x011a:
          return "MicroPDF";
        case 0x011b:
          return "PDF-417";
        case 0x011c:
          return "PosiCode";
        case 0x011d:
          return "QR Code";
        case 0x011e:
          return "SuperCode";
        case 0x011f:
          return "UltraCode";
        case 0x0120:
          return "USD-5 (Slug Code)";
        case 0x0121:
          return "VeriCode";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class barcode_scanner : std::uint16_t {
  BARCODE_BADGE_READER = 0x0001,
  BARCODE_SCANNER = 0x0002,
  DUMB_BAR_CODE_SCANNER = 0x0003,
  CORDLESS_SCANNER_BASE = 0x0004,
  BAR_CODE_SCANNER_CRADLE = 0x0005,
  ATTRIBUTE_REPORT = 0x0010,
  SETTINGS_REPORT = 0x0011,
  SCANNED_DATA_REPORT = 0x0012,
  RAW_SCANNED_DATA_REPORT = 0x0013,
  TRIGGER_REPORT = 0x0014,
  STATUS_REPORT = 0x0015,
  UPC_EAN_CONTROL_REPORT = 0x0016,
  EAN_2_3_LABEL_CONTROL_REPORT = 0x0017,
  CODE_39_CONTROL_REPORT = 0x0018,
  INTERLEAVED_2_OF_5_CONTROL_REPORT = 0x0019,
  STANDARD_2_OF_5_CONTROL_REPORT = 0x001a,
  MSI_PLESSEY_CONTROL_REPORT = 0x001b,
  CODABAR_CONTROL_REPORT = 0x001c,
  CODE_128_CONTROL_REPORT = 0x001d,
  MISC_1D_CONTROL_REPORT = 0x001e,
  _2D_CONTROL_REPORT = 0x001f,
  AIMING_POINTER_MODE = 0x0030,
  BAR_CODE_PRESENT_SENSOR = 0x0031,
  CLASS_1A_LASER = 0x0032,
  CLASS_2_LASER = 0x0033,
  HEATER_PRESENT = 0x0034,
  CONTACT_SCANNER = 0x0035,
  ELECTRONIC_ARTICLE_SURVEILLANCE_NOTIFICATION = 0x0036,
  CONSTANT_ELECTRONIC_ARTICLE_SURVEILLANCE = 0x0037,
  ERROR_INDICATION = 0x0038,
  FIXED_BEEPER = 0x0039,
  GOOD_DECODE_INDICATION = 0x003a,
  HANDS_FREE_SCANNING = 0x003b,
  INTRINSICALLY_SAFE = 0x003c,
  CLASS_1_LASER = 0x003d,
  LONG_RANGE_SCANNER = 0x003e,
  MIRROR_SPEED_CONTROL = 0x003f,
  NOT_ON_FILE_INDICATION = 0x0040,
  PROGRAMMABLE_BEEPER = 0x0041,
  TRIGGERLESS = 0x0042,
  WAND = 0x0043,
  WATER_RESISTANT = 0x0044,
  MULTI_RANGE_SCANNER = 0x0045,
  PROXIMITY_SENSOR = 0x0046,
  FRAGMENT_DECODING = 0x004d,
  SCANNER_READ_CONFIDENCE = 0x004e,
  DATA_PREFIX = 0x004f,
  PREFIX_AIMI = 0x0050,
  PREFIX_NONE = 0x0051,
  PREFIX_PROPRIETARY = 0x0052,
  ACTIVE_TIME = 0x0055,
  AIMING_LASER_PATTERN = 0x0056,
  BAR_CODE_PRESENT = 0x0057,
  BEEPER_STATE = 0x0058,
  LASER_ON_TIME = 0x0059,
  LASER_STATE = 0x005a,
  LOCKOUT_TIME = 0x005b,
  MOTOR_STATE = 0x005c,
  MOTOR_TIMEOUT = 0x005d,
  POWER_ON_RESET_SCANNER = 0x005e,
  PREVENT_READ_OF_BARCODES = 0x005f,
  INITIATE_BARCODE_READ = 0x0060,
  TRIGGER_STATE = 0x0061,
  TRIGGER_MODE = 0x0062,
  TRIGGER_MODE_BLINKING_LASER_ON = 0x0063,
  TRIGGER_MODE_CONTINUOUS_LASER_ON = 0x0064,
  TRIGGER_MODE_LASER_ON_WHILE_PULLED = 0x0065,
  TRIGGER_MODE_LASER_STAYS_ON_AFTER_RELEASE = 0x0066,
  COMMIT_PARAMETERS_TO_NVM = 0x006d,
  PARAMETER_SCANNING = 0x006e,
  PARAMETERS_CHANGED = 0x006f,
  SET_PARAMETER_DEFAULT_VALUES = 0x0070,
  SCANNER_IN_CRADLE = 0x0075,
  SCANNER_IN_RANGE = 0x0076,
  AIM_DURATION = 0x007a,
  GOOD_READ_LAMP_DURATION = 0x007b,
  GOOD_READ_LAMP_INTENSITY = 0x007c,
  GOOD_READ_LED = 0x007d,
  GOOD_READ_TONE_FREQUENCY = 0x007e,
  GOOD_READ_TONE_LENGTH = 0x007f,
  GOOD_READ_TONE_VOLUME = 0x0080,
  NO_READ_MESSAGE = 0x0082,
  NOT_ON_FILE_VOLUME = 0x0083,
  POWERUP_BEEP = 0x0084,
  SOUND_ERROR_BEEP = 0x0085,
  SOUND_GOOD_READ_BEEP = 0x0086,
  SOUND_NOT_ON_FILE_BEEP = 0x0087,
  GOOD_READ_WHEN_TO_WRITE = 0x0088,
  GRWTI_AFTER_DECODE = 0x0089,
  GRWTI_BEEP_LAMP_AFTER_TRANSMIT = 0x008a,
  GRWTI_NO_BEEP_LAMP_USE_AT_ALL = 0x008b,
  BOOKLAND_EAN = 0x0091,
  CONVERT_EAN_8_TO_13_TYPE = 0x0092,
  CONVERT_UPC_A_TO_EAN_13 = 0x0093,
  CONVERT_UPC_E_TO_A = 0x0094,
  EAN_13 = 0x0095,
  EAN_8 = 0x0096,
  EAN_99_128_MANDATORY = 0x0097,
  EAN_99_P5_128_OPTIONAL = 0x0098,
  ENABLE_EAN_TWO_LABEL = 0x0099,
  UPC_EAN = 0x009a,
  UPC_EAN_COUPON_CODE = 0x009b,
  UPC_EAN_PERIODICALS = 0x009c,
  UPC_A = 0x009d,
  UPC_A_WITH_128_MANDATORY = 0x009e,
  UPC_A_WITH_128_OPTIONAL = 0x009f,
  UPC_A_WITH_P5_OPTIONAL = 0x00a0,
  UPC_E = 0x00a1,
  UPC_E1 = 0x00a2,
  PERIODICAL = 0x00a9,
  PERIODICAL_AUTO_DISCRIMINATE_2 = 0x00aa,
  PERIODICAL_ONLY_DECODE_WITH_2 = 0x00ab,
  PERIODICAL_IGNORE_2 = 0x00ac,
  PERIODICAL_AUTO_DISCRIMINATE_5 = 0x00ad,
  PERIODICAL_ONLY_DECODE_WITH_5 = 0x00ae,
  PERIODICAL_IGNORE_5 = 0x00af,
  CHECK = 0x00b0,
  CHECK_DISABLE_PRICE = 0x00b1,
  CHECK_ENABLE_4_DIGIT_PRICE = 0x00b2,
  CHECK_ENABLE_5_DIGIT_PRICE = 0x00b3,
  CHECK_ENABLE_EUROPEAN_4_DIGIT_PRICE = 0x00b4,
  CHECK_ENABLE_EUROPEAN_5_DIGIT_PRICE = 0x00b5,
  EAN_TWO_LABEL = 0x00b7,
  EAN_THREE_LABEL = 0x00b8,
  EAN_8_FLAG_DIGIT_1 = 0x00b9,
  EAN_8_FLAG_DIGIT_2 = 0x00ba,
  EAN_8_FLAG_DIGIT_3 = 0x00bb,
  EAN_13_FLAG_DIGIT_1 = 0x00bc,
  EAN_13_FLAG_DIGIT_2 = 0x00bd,
  EAN_13_FLAG_DIGIT_3 = 0x00be,
  ADD_EAN_2_3_LABEL_DEFINITION = 0x00bf,
  CLEAR_ALL_EAN_2_3_LABEL_DEFINITIONS = 0x00c0,
  CODABAR = 0x00c3,
  CODE_128 = 0x00c4,
  CODE_39 = 0x00c7,
  CODE_93 = 0x00c8,
  FULL_ASCII_CONVERSION = 0x00c9,
  INTERLEAVED_2_OF_5 = 0x00ca,
  ITALIAN_PHARMACY_CODE = 0x00cb,
  MSI_PLESSEY = 0x00cc,
  STANDARD_2_OF_5_IATA = 0x00cd,
  STANDARD_2_OF_5 = 0x00ce,
  TRANSMIT_START_STOP = 0x00d3,
  TRI_OPTIC = 0x00d4,
  UCC_EAN_128 = 0x00d5,
  CHECK_DIGIT = 0x00d6,
  CHECK_DIGIT_DISABLE = 0x00d7,
  CHECK_DIGIT_ENABLE_INTERLEAVED_2_OF_5_OPCC = 0x00d8,
  CHECK_DIGIT_ENABLE_INTERLEAVED_2_OF_5_USS = 0x00d9,
  CHECK_DIGIT_ENABLE_STANDARD_2_OF_5_OPCC = 0x00da,
  CHECK_DIGIT_ENABLE_STANDARD_2_OF_5_USS = 0x00db,
  CHECK_DIGIT_ENABLE_ONE_MSI_PLESSEY = 0x00dc,
  CHECK_DIGIT_ENABLE_TWO_MSI_PLESSEY = 0x00dd,
  CHECK_DIGIT_CODABAR_ENABLE = 0x00de,
  CHECK_DIGIT_CODE_39_ENABLE = 0x00df,
  TRANSMIT_CHECK_DIGIT = 0x00f0,
  DISABLE_CHECK_DIGIT_TRANSMIT = 0x00f1,
  ENABLE_CHECK_DIGIT_TRANSMIT = 0x00f2,
  SYMBOLOGY_IDENTIFIER_1 = 0x00fb,
  SYMBOLOGY_IDENTIFIER_2 = 0x00fc,
  SYMBOLOGY_IDENTIFIER_3 = 0x00fd,
  DECODED_DATA = 0x00fe,
  DECODE_DATA_CONTINUED = 0x00ff,
  BAR_SPACE_DATA = 0x0100,
  SCANNER_DATA_ACCURACY = 0x0101,
  RAW_DATA_POLARITY = 0x0102,
  POLARITY_INVERTED_BAR_CODE = 0x0103,
  POLARITY_NORMAL_BAR_CODE = 0x0104,
  MINIMUM_LENGTH_TO_DECODE = 0x0106,
  MAXIMUM_LENGTH_TO_DECODE = 0x0107,
  DISCRETE_LENGTH_TO_DECODE_1 = 0x0108,
  DISCRETE_LENGTH_TO_DECODE_2 = 0x0109,
  DATA_LENGTH_METHOD = 0x010a,
  DL_METHOD_READ_ANY = 0x010b,
  DL_METHOD_CHECK_IN_RANGE = 0x010c,
  DL_METHOD_CHECK_FOR_DISCRETE = 0x010d,
  AZTEC_CODE = 0x0110,
  BC412 = 0x0111,
  CHANNEL_CODE = 0x0112,
  CODE_16 = 0x0113,
  CODE_32 = 0x0114,
  CODE_49 = 0x0115,
  CODE_ONE = 0x0116,
  COLORCODE = 0x0117,
  DATA_MATRIX = 0x0118,
  MAXICODE = 0x0119,
  MICROPDF = 0x011a,
  PDF_417 = 0x011b,
  POSICODE = 0x011c,
  QR_CODE = 0x011d,
  SUPERCODE = 0x011e,
  ULTRACODE = 0x011f,
  USD_5_SLUG_CODE = 0x0120,
  VERICODE = 0x0121,
};
} // namespace hid::page

#endif // __HID_PAGE_BARCODE_SCANNER_HPP_

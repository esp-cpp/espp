#ifndef __HID_PAGE_LIGHTING_AND_ILLUMINATION_HPP_
#define __HID_PAGE_LIGHTING_AND_ILLUMINATION_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class lighting_and_illumination : std::uint8_t;
template <> constexpr inline auto get_info<lighting_and_illumination>() {
  return info(
      0x0059, 0x0071, "Lighting and Illumination",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Lamp Array";
        case 0x0002:
          return "Lamp Array Attributes Report";
        case 0x0003:
          return "Lamp Count";
        case 0x0004:
          return "Bounding Box Width (um)";
        case 0x0005:
          return "Bounding Box Height (um)";
        case 0x0006:
          return "Bounding Box Depth (um)";
        case 0x0007:
          return "Lamp Array Kind";
        case 0x0008:
          return "Minimal Update Interval (us)";
        case 0x0020:
          return "Lamp Attributes Request Report";
        case 0x0021:
          return "Lamp ID";
        case 0x0022:
          return "Lamp Attributes Response Report";
        case 0x0023:
          return "Position X (um)";
        case 0x0024:
          return "Position Y (um)";
        case 0x0025:
          return "Position Z (um)";
        case 0x0026:
          return "Lamp Purposes";
        case 0x0027:
          return "Update Latency (us)";
        case 0x0028:
          return "Red Level Count";
        case 0x0029:
          return "Green Level Count";
        case 0x002a:
          return "Blue Level Count";
        case 0x002b:
          return "Intensity Level Count";
        case 0x002c:
          return "Programmable";
        case 0x002d:
          return "Input Binding";
        case 0x0050:
          return "Lamp Multi Update Report";
        case 0x0051:
          return "Red Update Channel";
        case 0x0052:
          return "Green Update Channel";
        case 0x0053:
          return "Blue Update Channel";
        case 0x0054:
          return "Intensity Update Channel";
        case 0x0055:
          return "Lamp Update Flags";
        case 0x0060:
          return "Lamp Range Update Report";
        case 0x0061:
          return "Lamp ID Start";
        case 0x0062:
          return "Lamp ID End";
        case 0x0070:
          return "Lamp Array Control Report";
        case 0x0071:
          return "Autonomous Mode";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class lighting_and_illumination : std::uint8_t {
  LAMP_ARRAY = 0x0001,
  LAMP_ARRAY_ATTRIBUTES_REPORT = 0x0002,
  LAMP_COUNT = 0x0003,
  BOUNDING_BOX_WIDTH_UM = 0x0004,
  BOUNDING_BOX_HEIGHT_UM = 0x0005,
  BOUNDING_BOX_DEPTH_UM = 0x0006,
  LAMP_ARRAY_KIND = 0x0007,
  MINIMAL_UPDATE_INTERVAL_US = 0x0008,
  LAMP_ATTRIBUTES_REQUEST_REPORT = 0x0020,
  LAMP_ID = 0x0021,
  LAMP_ATTRIBUTES_RESPONSE_REPORT = 0x0022,
  POSITION_X_UM = 0x0023,
  POSITION_Y_UM = 0x0024,
  POSITION_Z_UM = 0x0025,
  LAMP_PURPOSES = 0x0026,
  UPDATE_LATENCY_US = 0x0027,
  RED_LEVEL_COUNT = 0x0028,
  GREEN_LEVEL_COUNT = 0x0029,
  BLUE_LEVEL_COUNT = 0x002a,
  INTENSITY_LEVEL_COUNT = 0x002b,
  PROGRAMMABLE = 0x002c,
  INPUT_BINDING = 0x002d,
  LAMP_MULTI_UPDATE_REPORT = 0x0050,
  RED_UPDATE_CHANNEL = 0x0051,
  GREEN_UPDATE_CHANNEL = 0x0052,
  BLUE_UPDATE_CHANNEL = 0x0053,
  INTENSITY_UPDATE_CHANNEL = 0x0054,
  LAMP_UPDATE_FLAGS = 0x0055,
  LAMP_RANGE_UPDATE_REPORT = 0x0060,
  LAMP_ID_START = 0x0061,
  LAMP_ID_END = 0x0062,
  LAMP_ARRAY_CONTROL_REPORT = 0x0070,
  AUTONOMOUS_MODE = 0x0071,
};
} // namespace hid::page

#endif // __HID_PAGE_LIGHTING_AND_ILLUMINATION_HPP_

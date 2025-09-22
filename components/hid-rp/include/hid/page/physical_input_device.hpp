#ifndef __HID_PAGE_PHYSICAL_INPUT_DEVICE_HPP_
#define __HID_PAGE_PHYSICAL_INPUT_DEVICE_HPP_

#include "hid/usage.hpp"

namespace hid::page {
enum class physical_input_device : std::uint8_t;
template <> constexpr inline auto get_info<physical_input_device>() {
  return info(
      0x000f, 0x00ac, "Physical Input Device",
      [](hid::usage_id_t id) {
        switch (id) {
        case 0x0001:
          return "Physical Input Device";
        case 0x0020:
          return "Normal";
        case 0x0021:
          return "Set Effect Report";
        case 0x0022:
          return "Effect Parameter Block Index";
        case 0x0023:
          return "Parameter Block Offset";
        case 0x0024:
          return "ROM Flag";
        case 0x0025:
          return "Effect Type";
        case 0x0026:
          return "ET Constant-Force";
        case 0x0027:
          return "ET Ramp";
        case 0x0028:
          return "ET Custom-Force";
        case 0x0030:
          return "ET Square";
        case 0x0031:
          return "ET Sine";
        case 0x0032:
          return "ET Triangle";
        case 0x0033:
          return "ET Sawtooth Up";
        case 0x0034:
          return "ET Sawtooth Down";
        case 0x0040:
          return "ET Spring";
        case 0x0041:
          return "ET Damper";
        case 0x0042:
          return "ET Inertia";
        case 0x0043:
          return "ET Friction";
        case 0x0050:
          return "Duration";
        case 0x0051:
          return "Sample Period";
        case 0x0052:
          return "Gain";
        case 0x0053:
          return "Trigger Button";
        case 0x0054:
          return "Trigger Repeat Interval";
        case 0x0055:
          return "Axes Enable";
        case 0x0056:
          return "Direction Enable";
        case 0x0057:
          return "Direction";
        case 0x0058:
          return "Type Specific Block Offset";
        case 0x0059:
          return "Block Type";
        case 0x005a:
          return "Set Envelope Report";
        case 0x005b:
          return "Attack Level";
        case 0x005c:
          return "Attack Time";
        case 0x005d:
          return "Fade Level";
        case 0x005e:
          return "Fade Time";
        case 0x005f:
          return "Set Condition Report";
        case 0x0060:
          return "Center-Point Offset";
        case 0x0061:
          return "Positive Coefficient";
        case 0x0062:
          return "Negative Coefficient";
        case 0x0063:
          return "Positive Saturation";
        case 0x0064:
          return "Negative Saturation";
        case 0x0065:
          return "Dead Band";
        case 0x0066:
          return "Download Force Sample";
        case 0x0067:
          return "Isoch Custom-Force Enable";
        case 0x0068:
          return "Custom-Force Data Report";
        case 0x0069:
          return "Custom-Force Data";
        case 0x006a:
          return "Custom-Force Vendor Defined Data";
        case 0x006b:
          return "Set Custom-Force Report";
        case 0x006c:
          return "Custom-Force Data Offset";
        case 0x006d:
          return "Sample Count";
        case 0x006e:
          return "Set Periodic Report";
        case 0x006f:
          return "Offset";
        case 0x0070:
          return "Magnitude";
        case 0x0071:
          return "Phase";
        case 0x0072:
          return "Period";
        case 0x0073:
          return "Set Constant-Force Report";
        case 0x0074:
          return "Set Ramp-Force Report";
        case 0x0075:
          return "Ramp Start";
        case 0x0076:
          return "Ramp End";
        case 0x0077:
          return "Effect Operation Report";
        case 0x0078:
          return "Effect Operation";
        case 0x0079:
          return "Op Effect Start";
        case 0x007a:
          return "Op Effect Start Solo";
        case 0x007b:
          return "Op Effect Stop";
        case 0x007c:
          return "Loop Count";
        case 0x007d:
          return "Device Gain Report";
        case 0x007e:
          return "Device Gain";
        case 0x007f:
          return "Parameter Block Pools Report";
        case 0x0080:
          return "RAM Pool Size";
        case 0x0081:
          return "ROM Pool Size";
        case 0x0082:
          return "ROM Effect Block Count";
        case 0x0083:
          return "Simultaneous Effects Max";
        case 0x0084:
          return "Pool Alignment";
        case 0x0085:
          return "Parameter Block Move Report";
        case 0x0086:
          return "Move Source";
        case 0x0087:
          return "Move Destination";
        case 0x0088:
          return "Move Length";
        case 0x0089:
          return "Effect Parameter Block Load Report";
        case 0x008b:
          return "Effect Parameter Block Load Status";
        case 0x008c:
          return "Block Load Success";
        case 0x008d:
          return "Block Load Full";
        case 0x008e:
          return "Block Load Error";
        case 0x008f:
          return "Block Handle";
        case 0x0090:
          return "Effect Parameter Block Free Report";
        case 0x0091:
          return "Type Specific Block Handle";
        case 0x0092:
          return "PID State Report";
        case 0x0094:
          return "Effect Playing";
        case 0x0095:
          return "PID Device Control Report";
        case 0x0096:
          return "PID Device Control";
        case 0x0097:
          return "DC Enable Actuators";
        case 0x0098:
          return "DC Disable Actuators";
        case 0x0099:
          return "DC Stop All Effects";
        case 0x009a:
          return "DC Reset";
        case 0x009b:
          return "DC Pause";
        case 0x009c:
          return "DC Continue";
        case 0x009f:
          return "Device Paused";
        case 0x00a0:
          return "Actuators Enabled";
        case 0x00a4:
          return "Safety Switch";
        case 0x00a5:
          return "Actuator Override Switch";
        case 0x00a6:
          return "Actuator Power";
        case 0x00a7:
          return "Start Delay";
        case 0x00a8:
          return "Parameter Block Size";
        case 0x00a9:
          return "Device-Managed Pool";
        case 0x00aa:
          return "Shared Parameter Blocks";
        case 0x00ab:
          return "Create New Effect Parameter Block Report";
        case 0x00ac:
          return "RAM Pool Available";
        default:
          return (const char *)nullptr;
        }
      },
      0x0000);
}
enum class physical_input_device : std::uint8_t {
  PHYSICAL_INPUT_DEVICE = 0x0001,
  NORMAL = 0x0020,
  SET_EFFECT_REPORT = 0x0021,
  EFFECT_PARAMETER_BLOCK_INDEX = 0x0022,
  PARAMETER_BLOCK_OFFSET = 0x0023,
  ROM_FLAG = 0x0024,
  EFFECT_TYPE = 0x0025,
  ET_CONSTANT_FORCE = 0x0026,
  ET_RAMP = 0x0027,
  ET_CUSTOM_FORCE = 0x0028,
  ET_SQUARE = 0x0030,
  ET_SINE = 0x0031,
  ET_TRIANGLE = 0x0032,
  ET_SAWTOOTH_UP = 0x0033,
  ET_SAWTOOTH_DOWN = 0x0034,
  ET_SPRING = 0x0040,
  ET_DAMPER = 0x0041,
  ET_INERTIA = 0x0042,
  ET_FRICTION = 0x0043,
  DURATION = 0x0050,
  SAMPLE_PERIOD = 0x0051,
  GAIN = 0x0052,
  TRIGGER_BUTTON = 0x0053,
  TRIGGER_REPEAT_INTERVAL = 0x0054,
  AXES_ENABLE = 0x0055,
  DIRECTION_ENABLE = 0x0056,
  DIRECTION = 0x0057,
  TYPE_SPECIFIC_BLOCK_OFFSET = 0x0058,
  BLOCK_TYPE = 0x0059,
  SET_ENVELOPE_REPORT = 0x005a,
  ATTACK_LEVEL = 0x005b,
  ATTACK_TIME = 0x005c,
  FADE_LEVEL = 0x005d,
  FADE_TIME = 0x005e,
  SET_CONDITION_REPORT = 0x005f,
  CENTER_POINT_OFFSET = 0x0060,
  POSITIVE_COEFFICIENT = 0x0061,
  NEGATIVE_COEFFICIENT = 0x0062,
  POSITIVE_SATURATION = 0x0063,
  NEGATIVE_SATURATION = 0x0064,
  DEAD_BAND = 0x0065,
  DOWNLOAD_FORCE_SAMPLE = 0x0066,
  ISOCH_CUSTOM_FORCE_ENABLE = 0x0067,
  CUSTOM_FORCE_DATA_REPORT = 0x0068,
  CUSTOM_FORCE_DATA = 0x0069,
  CUSTOM_FORCE_VENDOR_DEFINED_DATA = 0x006a,
  SET_CUSTOM_FORCE_REPORT = 0x006b,
  CUSTOM_FORCE_DATA_OFFSET = 0x006c,
  SAMPLE_COUNT = 0x006d,
  SET_PERIODIC_REPORT = 0x006e,
  OFFSET = 0x006f,
  MAGNITUDE = 0x0070,
  PHASE = 0x0071,
  PERIOD = 0x0072,
  SET_CONSTANT_FORCE_REPORT = 0x0073,
  SET_RAMP_FORCE_REPORT = 0x0074,
  RAMP_START = 0x0075,
  RAMP_END = 0x0076,
  EFFECT_OPERATION_REPORT = 0x0077,
  EFFECT_OPERATION = 0x0078,
  OP_EFFECT_START = 0x0079,
  OP_EFFECT_START_SOLO = 0x007a,
  OP_EFFECT_STOP = 0x007b,
  LOOP_COUNT = 0x007c,
  DEVICE_GAIN_REPORT = 0x007d,
  DEVICE_GAIN = 0x007e,
  PARAMETER_BLOCK_POOLS_REPORT = 0x007f,
  RAM_POOL_SIZE = 0x0080,
  ROM_POOL_SIZE = 0x0081,
  ROM_EFFECT_BLOCK_COUNT = 0x0082,
  SIMULTANEOUS_EFFECTS_MAX = 0x0083,
  POOL_ALIGNMENT = 0x0084,
  PARAMETER_BLOCK_MOVE_REPORT = 0x0085,
  MOVE_SOURCE = 0x0086,
  MOVE_DESTINATION = 0x0087,
  MOVE_LENGTH = 0x0088,
  EFFECT_PARAMETER_BLOCK_LOAD_REPORT = 0x0089,
  EFFECT_PARAMETER_BLOCK_LOAD_STATUS = 0x008b,
  BLOCK_LOAD_SUCCESS = 0x008c,
  BLOCK_LOAD_FULL = 0x008d,
  BLOCK_LOAD_ERROR = 0x008e,
  BLOCK_HANDLE = 0x008f,
  EFFECT_PARAMETER_BLOCK_FREE_REPORT = 0x0090,
  TYPE_SPECIFIC_BLOCK_HANDLE = 0x0091,
  PID_STATE_REPORT = 0x0092,
  EFFECT_PLAYING = 0x0094,
  PID_DEVICE_CONTROL_REPORT = 0x0095,
  PID_DEVICE_CONTROL = 0x0096,
  DC_ENABLE_ACTUATORS = 0x0097,
  DC_DISABLE_ACTUATORS = 0x0098,
  DC_STOP_ALL_EFFECTS = 0x0099,
  DC_RESET = 0x009a,
  DC_PAUSE = 0x009b,
  DC_CONTINUE = 0x009c,
  DEVICE_PAUSED = 0x009f,
  ACTUATORS_ENABLED = 0x00a0,
  SAFETY_SWITCH = 0x00a4,
  ACTUATOR_OVERRIDE_SWITCH = 0x00a5,
  ACTUATOR_POWER = 0x00a6,
  START_DELAY = 0x00a7,
  PARAMETER_BLOCK_SIZE = 0x00a8,
  DEVICE_MANAGED_POOL = 0x00a9,
  SHARED_PARAMETER_BLOCKS = 0x00aa,
  CREATE_NEW_EFFECT_PARAMETER_BLOCK_REPORT = 0x00ab,
  RAM_POOL_AVAILABLE = 0x00ac,
};
} // namespace hid::page

#endif // __HID_PAGE_PHYSICAL_INPUT_DEVICE_HPP_

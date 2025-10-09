#pragma once

#include <cstdint>

namespace espp {
namespace gamepad {

#pragma pack(push, 1)

/// Accelerometer data
struct Accelerometer {
  union {
    struct {
      std::int16_t X;
      std::int16_t Y;
      std::int16_t Z;
    };
    std::int16_t raw[3];
  };
};

/// Gyroscope data
struct Gyroscope {
  union {
    struct {
      std::int16_t X;
      std::int16_t Y;
      std::int16_t Z;
    };
    struct {
      std::int16_t Pitch;
      std::int16_t Yaw;
      std::int16_t Roll;
    };
    std::int16_t raw[3];
  };
};

#pragma pack(pop)

} // namespace gamepad
} // namespace espp

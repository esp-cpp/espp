#pragma once

#include <cstdint>

namespace espp {
namespace gamepad {
/// Accelerometer data
struct Accelerometer {
  union {
    struct {
      std::int16_t X;
      std::int16_t Y;
      std::int16_t Z;
    } __attribute__((packed));
    std::int16_t raw[3];
  } __attribute__((packed));
} __attribute__((packed));

/// Gyroscope data
struct Gyroscope {
  union {
    struct {
      std::int16_t X;
      std::int16_t Y;
      std::int16_t Z;
    } __attribute__((packed));
    struct {
      std::int16_t Pitch;
      std::int16_t Yaw;
      std::int16_t Roll;
    } __attribute__((packed));
    std::int16_t raw[3];
  } __attribute__((packed));
} __attribute__((packed));
} // namespace gamepad
} // namespace espp

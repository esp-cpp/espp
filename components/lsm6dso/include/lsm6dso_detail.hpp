#pragma once

#include <array>
#include <cstdint>

namespace espp {
namespace lsm6dso {

/// @brief Enum class for the interface type of the LSM6DSO
/// @note The LSM6DSO supports both I2C and SPI
enum class Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SPI = 1, ///< Serial Peripheral Interface (SPI)
};

/// @brief Register map for the LSM6DSO
/// @note Only a subset of registers are listed here for brevity
enum class Register : uint8_t {
  WHO_AM_I = 0x0F,        ///< Device ID register
  CTRL1_XL = 0x10,        ///< Accelerometer control (ODR, FS)
  CTRL2_G = 0x11,         ///< Gyroscope control (ODR, FS)
  CTRL3_C = 0x12,         ///< Main control (reset, BDU, etc.)
  CTRL4_C = 0x13,         ///< Additional control (interrupts, etc.)
  CTRL5_C = 0x14,         ///< Additional control (rounding, etc.)
  CTRL6_C = 0x15,         ///< Accelerometer filtering
  CTRL7_G = 0x16,         ///< Gyro filtering
  CTRL8_XL = 0x17,        ///< Accel filtering
  CTRL9_XL = 0x18,        ///< Accel/gyro enable
  CTRL10_C = 0x19,        ///< Master control
  STATUS_REG = 0x1E,      ///< Status register
  OUT_TEMP_L = 0x20,      ///< Temperature output (low byte)
  OUT_TEMP_H = 0x21,      ///< Temperature output (high byte)
  OUTX_L_G = 0x22,        ///< Gyro X low
  OUTX_H_G = 0x23,        ///< Gyro X high
  OUTY_L_G = 0x24,        ///< Gyro Y low
  OUTY_H_G = 0x25,        ///< Gyro Y high
  OUTZ_L_G = 0x26,        ///< Gyro Z low
  OUTZ_H_G = 0x27,        ///< Gyro Z high
  OUTX_L_A = 0x28,        ///< Accel X low
  OUTX_H_A = 0x29,        ///< Accel X high
  OUTY_L_A = 0x2A,        ///< Accel Y low
  OUTY_H_A = 0x2B,        ///< Accel Y high
  OUTZ_L_A = 0x2C,        ///< Accel Z low
  OUTZ_H_A = 0x2D,        ///< Accel Z high
  FIFO_CTRL1 = 0x07,      ///< FIFO control 1
  FIFO_CTRL2 = 0x08,      ///< FIFO control 2
  FIFO_CTRL3 = 0x09,      ///< FIFO control 3
  FIFO_CTRL4 = 0x0A,      ///< FIFO control 4
  FIFO_STATUS1 = 0x3A,    ///< FIFO status 1
  FIFO_STATUS2 = 0x3B,    ///< FIFO status 2
  FIFO_DATA_OUT_L = 0x3E, ///< FIFO data out low
  FIFO_DATA_OUT_H = 0x3F, ///< FIFO data out high
  TAP_CFG = 0x58,         ///< Tap configuration
  TAP_THS_6D = 0x59,      ///< Tap threshold
  INT1_CTRL = 0x0D,       ///< INT1 pin control
  INT2_CTRL = 0x0E,       ///< INT2 pin control
  WAKE_UP_SRC = 0x1B,     ///< Wake-up source
  TAP_SRC = 0x1C,         ///< Tap source
  D6D_SRC = 0x1D,         ///< 6D orientation source
};

/// @brief Accelerometer full-scale range
enum class AccelRange : uint8_t {
  RANGE_2G = 0,  ///< ±2g
  RANGE_16G = 1, ///< ±16g
  RANGE_4G = 2,  ///< ±4g
  RANGE_8G = 3,  ///< ±8g
};

/// @brief Accelerometer output data rate
enum class AccelODR : uint8_t {
  POWER_DOWN = 0,    ///< Power-down
  ODR_12_5_HZ = 1,   ///< 12.5 Hz
  ODR_26_HZ = 2,     ///< 26 Hz
  ODR_52_HZ = 3,     ///< 52 Hz
  ODR_104_HZ = 4,    ///< 104 Hz
  ODR_208_HZ = 5,    ///< 208 Hz
  ODR_416_HZ = 6,    ///< 416 Hz
  ODR_833_HZ = 7,    ///< 833 Hz
  ODR_1_66_KHZ = 8,  ///< 1.66 kHz
  ODR_3_33_KHZ = 9,  ///< 3.33 kHz
  ODR_6_66_KHZ = 10, ///< 6.66 kHz
};

/// @brief Accelerometer Filter Configuration
enum class AccelFilter : uint8_t {
  DISABLED = 0, ///< Filter disabled
  LOWPASS = 1,  ///< Low-pass filter
  HIGHPASS = 2, ///< High-pass filter
  BANDPASS = 3, ///< Band-pass filter
};

/// @brief Gyroscope full-scale range
enum class GyroRange : uint8_t {
  DPS_125 = std::numeric_limits<uint8_t>::max(), ///< ±125 dps
  DPS_250 = 0,                                   ///< ±250 dps
  DPS_500 = 1,                                   ///< ±500 dps
  DPS_1000 = 2,                                  ///< ±1000 dps
  DPS_2000 = 3,                                  ///< ±2000 dps
};

/// @brief Gyroscope output data rate
using GyroODR = AccelODR;

/// @brief Gyroscope high-pass filter
enum class GyroHPF : uint8_t {
  HPF_0_016_HZ = 0, ///< High-pass filter at 0.016 Hz
  HPF_0_065_HZ = 1, ///< High-pass filter at 0.065 Hz
  HPF_0_26_HZ = 2,  ///< High-pass filter at 0.26 Hz
  HPF_1_04_HZ = 3,  ///< High-pass filter at 1.04 Hz
};

/// @brief Configuration structure for the LSM6DSO IMU
struct ImuConfig {
  AccelRange accel_range; ///< Accelerometer full-scale range
  AccelODR accel_odr;     ///< Accelerometer output data rate
  GyroRange gyro_range;   ///< Gyroscope full-scale range
  GyroODR gyro_odr;       ///< Gyroscope output data rate
};

/// @brief Raw IMU data
struct RawValue {
  union {
    struct {
      int16_t x; ///< Raw X-axis value
      int16_t y; ///< Raw Y-axis value
      int16_t z; ///< Raw Z-axis value
    };
    uint8_t raw[6];     ///< Raw byte array for direct access
    uint16_t values[3]; ///< Array for direct access to raw values
  };
};

/// @brief IMU data (float)
struct Value {
  union {
    struct {
      float x; ///< X-axis value
      float y; ///< Y-axis value
      float z; ///< Z-axis value
    };
    struct {
      float roll;  ///< Roll value
      float pitch; ///< Pitch value
      float yaw;   ///< Yaw value
    };
    float values[3];
  };
};

} // namespace lsm6dso
} // namespace espp

// Fmt formatters for enums/structs (for logging/debug)
#include "format.hpp"

// Add formatters as needed for enums/structs
template <> struct fmt::formatter<espp::lsm6dso::AccelRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::AccelRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::lsm6dso::AccelRange::RANGE_2G:
      return format_to(ctx.out(), "±2g");
    case espp::lsm6dso::AccelRange::RANGE_16G:
      return format_to(ctx.out(), "±16g");
    case espp::lsm6dso::AccelRange::RANGE_4G:
      return format_to(ctx.out(), "±4g");
    case espp::lsm6dso::AccelRange::RANGE_8G:
      return format_to(ctx.out(), "±8g");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::lsm6dso::AccelODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::AccelODR range, FormatContext &ctx) const {
    switch (range) {
    case espp::lsm6dso::AccelODR::POWER_DOWN:
      return format_to(ctx.out(), "Power Down");
    case espp::lsm6dso::AccelODR::ODR_12_5_HZ:
      return format_to(ctx.out(), "12.5 Hz");
    case espp::lsm6dso::AccelODR::ODR_26_HZ:
      return format_to(ctx.out(), "26 Hz");
    case espp::lsm6dso::AccelODR::ODR_52_HZ:
      return format_to(ctx.out(), "52 Hz");
    case espp::lsm6dso::AccelODR::ODR_104_HZ:
      return format_to(ctx.out(), "104 Hz");
    case espp::lsm6dso::AccelODR::ODR_208_HZ:
      return format_to(ctx.out(), "208 Hz");
    case espp::lsm6dso::AccelODR::ODR_416_HZ:
      return format_to(ctx.out(), "416 Hz");
    case espp::lsm6dso::AccelODR::ODR_833_HZ:
      return format_to(ctx.out(), "833 Hz");
    case espp::lsm6dso::AccelODR::ODR_1_66_KHZ:
      return format_to(ctx.out(), "1.66 kHz");
    case espp::lsm6dso::AccelODR::ODR_3_33_KHZ:
      return format_to(ctx.out(), "3.33 kHz");
    case espp::lsm6dso::AccelODR::ODR_6_66_KHZ:
      return format_to(ctx.out(), "6.66 kHz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::lsm6dso::GyroRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::GyroRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::lsm6dso::GyroRange::DPS_125:
      return format_to(ctx.out(), "±125 dps");
    case espp::lsm6dso::GyroRange::DPS_250:
      return format_to(ctx.out(), "±250 dps");
    case espp::lsm6dso::GyroRange::DPS_500:
      return format_to(ctx.out(), "±500 dps");
    case espp::lsm6dso::GyroRange::DPS_1000:
      return format_to(ctx.out(), "±1000 dps");
    case espp::lsm6dso::GyroRange::DPS_2000:
      return format_to(ctx.out(), "±2000 dps");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::lsm6dso::AccelFilter> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::AccelFilter f, FormatContext &ctx) const {
    switch (f) {
    case espp::lsm6dso::AccelFilter::DISABLED:
      return format_to(ctx.out(), "Disabled");
    case espp::lsm6dso::AccelFilter::LOWPASS:
      return format_to(ctx.out(), "Low-pass");
    case espp::lsm6dso::AccelFilter::HIGHPASS:
      return format_to(ctx.out(), "High-pass");
    case espp::lsm6dso::AccelFilter::BANDPASS:
      return format_to(ctx.out(), "Band-pass");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::lsm6dso::GyroHPF> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::GyroHPF f, FormatContext &ctx) const {
    switch (f) {
    case espp::lsm6dso::GyroHPF::HPF_0_016_HZ:
      return format_to(ctx.out(), "0.016 Hz");
    case espp::lsm6dso::GyroHPF::HPF_0_065_HZ:
      return format_to(ctx.out(), "0.065 Hz");
    case espp::lsm6dso::GyroHPF::HPF_0_26_HZ:
      return format_to(ctx.out(), "0.26 Hz");
    case espp::lsm6dso::GyroHPF::HPF_1_04_HZ:
      return format_to(ctx.out(), "1.04 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::lsm6dso::RawValue> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::lsm6dso::RawValue &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "RawValue(x: {}, y: {}, z: {})", value.x, value.y, value.z);
  }
};

template <> struct fmt::formatter<espp::lsm6dso::Value> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::lsm6dso::Value &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "Value(x: {}, y: {}, z: {})", value.x, value.y, value.z);
  }
};

template <> struct fmt::formatter<espp::lsm6dso::Interface> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::lsm6dso::Interface iface, FormatContext &ctx) const {
    switch (iface) {
    case espp::lsm6dso::Interface::I2C:
      return format_to(ctx.out(), "I2C");
    case espp::lsm6dso::Interface::SPI:
      return format_to(ctx.out(), "SPI");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

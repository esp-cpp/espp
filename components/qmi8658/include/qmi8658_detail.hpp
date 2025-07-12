#pragma once

#include <cstdint>

namespace espp {
/// @brief Namespace for the QMI8658 6-axis motion sensor
namespace qmi8658 {
/// @brief Enum class for the interface type of the QMI8658
enum class Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SSI = 1, ///< Synchronous Serial Interface (SSI), which can be SPI or SSI
};

/// @brief Enum class for FIFO configuration
enum class FifoMode : uint8_t {
  STREAM = 0,       ///< Stream mode
  STOP_ON_FULL = 1, ///< Stop on full mode
};

/// Accelerometer range
enum class AccelerometerRange : uint8_t {
  RANGE_2G = 0b000,  ///< ±2g
  RANGE_4G = 0b001,  ///< ±4g
  RANGE_8G = 0b010,  ///< ±8g
  RANGE_16G = 0b011, ///< ±16g
};

/// Accelerometer output data rate
enum class ODR : uint8_t {
  ODR_8000_HZ = 0b0000, ///< 8000 Hz
  ODR_4000_HZ = 0b0001, ///< 4000 Hz
  ODR_2000_HZ = 0b0010, ///< 2000 Hz
  ODR_1000_HZ = 0b0011, ///< 1000 Hz
  ODR_500_HZ = 0b0100,  ///< 500 Hz
  ODR_250_HZ = 0b0101,  ///< 250 Hz
  ODR_125_HZ = 0b0110,  ///< 125 Hz
  ODR_62_5_HZ = 0b0111, ///< 62.5 Hz
  ODR_31_25_HZ = 0b1000 ///< 31.25 Hz
};

/// Gyroscope range
enum class GyroscopeRange : uint8_t {
  RANGE_32_DPS = 0b000,   ///< ±32°/s
  RANGE_64_DPS = 0b001,   ///< ±64°/s
  RANGE_128_DPS = 0b010,  ///< ±128°/s
  RANGE_256_DPS = 0b011,  ///< ±256°/s
  RANGE_512_DPS = 0b100,  ///< ±512°/s
  RANGE_1024_DPS = 0b101, ///< ±1024°/s
  RANGE_2048_DPS = 0b110, ///< ±2048°/s
  RANGE_4096_DPS = 0b111, ///< ±4096°/s
};

/// IMU Configuration
struct ImuConfig {
  AccelerometerRange accelerometer_range = AccelerometerRange::RANGE_16G; ///< Accelerometer range
  ODR accelerometer_odr = ODR::ODR_1000_HZ; ///< Accelerometer output data rate
  GyroscopeRange gyroscope_range = GyroscopeRange::RANGE_2048_DPS; ///< Gyroscope range
  ODR gyroscope_odr = ODR::ODR_1000_HZ;                            ///< Gyroscope output data rate
};

/// Raw IMU data
struct RawValue {
  int16_t x; ///< Raw X-axis value
  int16_t y; ///< Raw Y-axis value
  int16_t z; ///< Raw Z-axis value
};

/// IMU data
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

/// @brief Enum class for the QMI8658 interrupt configuration
enum class InterruptDriveMode {
  PUSH_PULL = 0,  ///< Push-pull
  OPEN_DRAIN = 1, ///< Open drain
};

/// @brief Enum class for the QMI8658 interrupt configuration
enum class InterruptPolarity {
  ACTIVE_LOW = 0,  ///< Active low
  ACTIVE_HIGH = 1, ///< Active high
};

/// @brief Enum class for the QMI8658 interrupt configuration
enum class InterruptMode {
  PULSED = 0,  ///< Pulsed
  LATCHED = 1, ///< Latched
};

/// @brief Struct for the QMI8658 interrupt configuration
struct InterruptConfig {
  InterruptDriveMode drive_mode; ///< Drive mode
  InterruptPolarity polarity;    ///< Polarity
  InterruptMode mode;            ///< Mode
};
} // namespace qmi8658
} // namespace espp

#include "format.hpp"

// for libfmt printing of relevant imu types and structs
template <> struct fmt::formatter<espp::qmi8658::AccelerometerRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::qmi8658::AccelerometerRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::qmi8658::AccelerometerRange::RANGE_16G:
      return format_to(ctx.out(), "±16g");
    case espp::qmi8658::AccelerometerRange::RANGE_8G:
      return format_to(ctx.out(), "±8g");
    case espp::qmi8658::AccelerometerRange::RANGE_4G:
      return format_to(ctx.out(), "±4g");
    case espp::qmi8658::AccelerometerRange::RANGE_2G:
      return format_to(ctx.out(), "±2g");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::qmi8658::ODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext> auto format(espp::qmi8658::ODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::qmi8658::ODR::ODR_8000_HZ:
      return format_to(ctx.out(), "8000 Hz");
    case espp::qmi8658::ODR::ODR_4000_HZ:
      return format_to(ctx.out(), "4000 Hz");
    case espp::qmi8658::ODR::ODR_2000_HZ:
      return format_to(ctx.out(), "2000 Hz");
    case espp::qmi8658::ODR::ODR_1000_HZ:
      return format_to(ctx.out(), "1000 Hz");
    case espp::qmi8658::ODR::ODR_500_HZ:
      return format_to(ctx.out(), "500 Hz");
    case espp::qmi8658::ODR::ODR_250_HZ:
      return format_to(ctx.out(), "250 Hz");
    case espp::qmi8658::ODR::ODR_125_HZ:
      return format_to(ctx.out(), "125 Hz");
    case espp::qmi8658::ODR::ODR_62_5_HZ:
      return format_to(ctx.out(), "62.5 Hz");
    case espp::qmi8658::ODR::ODR_31_25_HZ:
      return format_to(ctx.out(), "31.25 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::qmi8658::GyroscopeRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::qmi8658::GyroscopeRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::qmi8658::GyroscopeRange::RANGE_4096_DPS:
      return format_to(ctx.out(), "±4096°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_2048_DPS:
      return format_to(ctx.out(), "±2048°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_1024_DPS:
      return format_to(ctx.out(), "±1024°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_512_DPS:
      return format_to(ctx.out(), "±512°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_256_DPS:
      return format_to(ctx.out(), "±256°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_128_DPS:
      return format_to(ctx.out(), "±128°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_64_DPS:
      return format_to(ctx.out(), "±64°/s");
    case espp::qmi8658::GyroscopeRange::RANGE_32_DPS:
      return format_to(ctx.out(), "±32°/s");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::qmi8658::ImuConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::qmi8658::ImuConfig &config, FormatContext &ctx) const {
    return format_to(ctx.out(),
                     "Accelerometer: {{ range: {}, odr: {} }}, Gyroscope: {{ range: {}, odr: {} }}",
                     config.accelerometer_range, config.accelerometer_odr, config.gyroscope_range,
                     config.gyroscope_odr);
  }
};

template <> struct fmt::formatter<espp::qmi8658::RawValue> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::qmi8658::RawValue &raw, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {}, y: {}, z: {} }}", raw.x, raw.y, raw.z);
  }
};

template <> struct fmt::formatter<espp::qmi8658::Value> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::qmi8658::Value &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {:.2f}, y: {:.2f}, z: {:.2f} }}", value.x, value.y, value.z);
  }
};

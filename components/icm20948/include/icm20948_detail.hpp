#pragma once

#include <cstdint>

#include "bitmask_operators.hpp"

namespace espp {
/// @brief Namespace for the ICM20948 9-axis motion sensor
namespace icm20948 {
/// @brief Enum class for the interface type of the ICM20948
enum class Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C). Max bus speed in this mode is 400 kHz.
  SSI = 1, ///< Synchronous Serial Interface (SSI), which can be SPI or SSI. Max bus speed in this
           ///< mode is 7 MHz.
};

/// @brief Enum class for the ICM20948 low power mode configuration
enum class PowerMode : uint8_t {
  SLEEP = 0,                          ///< Sleep mode
  LOW_POWER_ACCELEROMETER_ONLY = 1,   ///< Low power accelerometer only mode
  LOW_NOISE_ACCELEROMETER_ONLY = 2,   ///< Low noise accelerometer only mode
  GYROSCOPE_ONLY = 3,                 ///< Gyroscope only mode
  MAGNETOMETER_ONLY = 4,              ///< Magnetometer only mode
  ACCELEROMETER_AND_GYROSCOPE = 5,    ///< Accelerometer and gyroscope mode
  ACCELEROMETER_AND_MAGNETOMETER = 6, ///< Accelerometer and magnetometer mode
  NINE_AXIS = 7,                      ///< Nine-axis mode
};

enum class DutyCycleMode : uint8_t {
  GYRO_ONLY = (1 << 4),                                     ///< Gyroscope enabled
  ACCEL_ENABLED = (1 << 5),                                 ///< Accelerometer enabled
  I2C_MAST_ENABLED = (1 << 6),                              ///< I2C master enabled
  GYRO_AND_ACCEL_ENABLED = GYRO_ONLY | ACCEL_ENABLED,       ///< Gyroscope and accelerometer enabled
  GYRO_AND_I2C_MAST_ENABLED = GYRO_ONLY | I2C_MAST_ENABLED, ///< Gyroscope and I2C master enabled
  ACCEL_AND_I2C_MAST_ENABLED =
      ACCEL_ENABLED | I2C_MAST_ENABLED, ///< Accelerometer and I2C master enabled
  ALL_ENABLED = GYRO_ONLY | ACCEL_ENABLED |
                I2C_MAST_ENABLED, ///< Gyroscope, accelerometer, and I2C master enabled
};

/// @brief Enum class for the ICM20948 interrupt types
enum class InterruptType : uint8_t {
  FSYNC = 0x01,          ///< FSYNC interrupt
  WAKE_ON_MOTION = 0x02, ///< Wake on motion interrupt
  DMP = 0x04,            ///< DMP interrupt
  DATA_READY = 0x08,     ///< Raw data ready interrupt
  FIFO_OVERFLOW = 0x10,  ///< FIFO overflow interrupt
  FIFO_WATERMARK = 0x20, ///< FIFO watermark interrupt
};

/// @brief Enum class for the ICM20948 FIFO types
enum class FifoType : uint8_t {
  ACCEL = 0,      ///< Accelerometer
  GYRO = 1,       ///< Gyroscope
  ACCEL_GYRO = 2, ///< Accelerometer and Gyroscope
};

/// @brief Enum class for FIFO configuration
enum class FifoMode : uint8_t {
  STREAM = 0,       ///< Stream mode
  STOP_ON_FULL = 1, ///< Stop on full mode
};

/// Accelerometer range
enum class AccelerometerRange : uint8_t {
  RANGE_2G = (0 << 1),  ///< ±2g
  RANGE_4G = (1 << 1),  ///< ±4g
  RANGE_8G = (2 << 1),  ///< ±8g
  RANGE_16G = (3 << 1), ///< ±16g
};

/// Accelerometer output data rate
enum class AccelerometerODR : uint8_t {
  ODR_4500_HZ = 0,    ///< 4500 Hz
  ODR_1600_HZ = 5,    ///< 1600 Hz
  ODR_800_HZ = 6,     ///< 800 Hz
  ODR_400_HZ = 7,     ///< 400 Hz
  ODR_200_HZ = 8,     ///< 200 Hz
  ODR_100_HZ = 9,     ///< 100 Hz
  ODR_50_HZ = 10,     ///< 50 Hz
  ODR_25_HZ = 11,     ///< 25 Hz
  ODR_12_5_HZ = 12,   ///< 12.5 Hz
  ODR_6_25_HZ = 13,   ///< 6.25 Hz
  ODR_3_125_HZ = 14,  ///< 3.125 Hz
  ODR_1_5625_HZ = 15, ///< 1.5625 Hz
};

/// Gyroscope range
enum class GyroscopeRange : uint8_t {
  RANGE_250DPS = (0 << 1),  ///< ±250°/s
  RANGE_500DPS = (1 << 1),  ///< ±500°/s
  RANGE_1000DPS = (2 << 1), ///< ±1000°/s
  RANGE_2000DPS = (3 << 1), ///< ±2000°/s
};

/// Gyroscope output data rate
enum class GyroscopeODR : uint8_t {
  ODR_1600_HZ = 5,  ///< 1600 Hz
  ODR_800_HZ = 6,   ///< 800 Hz
  ODR_400_HZ = 7,   ///< 400 Hz
  ODR_200_HZ = 8,   ///< 200 Hz
  ODR_100_HZ = 9,   ///< 100 Hz
  ODR_50_HZ = 10,   ///< 50 Hz
  ODR_25_HZ = 11,   ///< 25 Hz
  ODR_12_5_HZ = 12, ///< 12.5 Hz
};

/// Digital Motion Processor (DMP) output data rate
enum class DmpODR : uint8_t {
  ODR_25_HZ = 0,  ///< 25 Hz
  ODR_400_HZ = 1, ///< 200 Hz
  ODR_50_HZ = 2,  ///< 50 Hz
  ODR_100_HZ = 3, ///< 100 Hz
};

/// Temperature DLPF Bandwidth
enum class TemperatureFilterBandwidth : uint8_t {
  FILTER_OFF = 0, ///< Filter off
  BW_180_HZ = 1,  ///< 180 Hz
  BW_72_HZ = 2,   ///< 72 Hz
  BW_34_HZ = 3,   ///< 34 Hz
  BW_16_HZ = 4,   ///< 16 Hz
  BW_8_HZ = 5,    ///< 8 Hz
  BW_4_HZ = 6,    ///< 4 Hz
  // NOTE: datasheet has both 0b111 and 0b110 matching to 4 Hz
};

/// @brief Sensor DLPF Bandwidth for both accelerometer and gyroscope
enum class SensorFilterBandwidth : uint8_t {
  FILTER_OFF = 0, ///< Filter off
  BW_180_HZ = 1,  ///< 180 Hz
  BW_121_HZ = 2,  ///< 121 Hz
  BW_73_HZ = 3,   ///< 73 Hz
  BW_53_HZ = 4,   ///< 53 Hz
  BW_34_HZ = 5,   ///< 34 Hz
  BW_25_HZ = 6,   ///< 25 Hz
  BW_16_HZ = 7,   ///< 16 Hz
};

/// @brief Magnetometer operation mode
enum class MagnetometerMode : uint8_t {
  POWER_DOWN = 0,             ///< Power down
  TRIGGER_MODE = 1,           ///< Trigger mode
  CONTINUOUS_MODE_10_HZ = 2,  ///< Continuous mode 10 Hz
  CONTINUOUS_MODE_20_HZ = 4,  ///< Continuous mode 20 Hz
  CONTINUOUS_MODE_50_HZ = 6,  ///< Continuous mode 50 Hz
  CONTINUOUS_MODE_100_HZ = 8, ///< Continuous mode 100 Hz
};

/// IMU Configuration
struct ImuConfig {
  AccelerometerRange accelerometer_range = AccelerometerRange::RANGE_16G; ///< Accelerometer range
  AccelerometerODR accelerometer_odr =
      AccelerometerODR::ODR_100_HZ; ///< Accelerometer output data rate
  GyroscopeRange gyroscope_range = GyroscopeRange::RANGE_2000DPS; ///< Gyroscope range
  GyroscopeODR gyroscope_odr = GyroscopeODR::ODR_100_HZ;          ///< Gyroscope output data rate
  MagnetometerMode magnetometer_mode =
      MagnetometerMode::CONTINUOUS_MODE_100_HZ; ///< Magnetometer mode
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

/// @brief Enum class for the ICM20948 interrupt configuration
enum class InterruptDriveMode {
  OPEN_DRAIN = 0, ///< Open drain
  PUSH_PULL = 1,  ///< Push-pull
};

/// @brief Enum class for the ICM20948 interrupt configuration
enum class InterruptPolarity {
  ACTIVE_LOW = 0,  ///< Active low
  ACTIVE_HIGH = 1, ///< Active high
};

/// @brief Enum class for the ICM20948 interrupt configuration
enum class InterruptMode {
  PULSED = 0,  ///< Pulsed
  LATCHED = 1, ///< Latched
};

/// @brief Struct for the ICM20948 interrupt configuration
struct InterruptConfig {
  InterruptDriveMode drive_mode; ///< Drive mode
  InterruptPolarity polarity;    ///< Polarity
  InterruptMode mode;            ///< Mode
};
} // namespace icm20948
} // namespace espp

// enable bitmaks operators for DutyCycleMode
template <> struct enable_bitmask_operators<espp::icm20948::DutyCycleMode> {
  static constexpr bool enable = true;
};

#include "format.hpp"

// for libfmt printing of relevant imu types and structs

template <> struct fmt::formatter<espp::icm20948::Interface> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::Interface interface, FormatContext &ctx) const {
    switch (interface) {
    case espp::icm20948::Interface::I2C:
      return format_to(ctx.out(), "I2C");
    case espp::icm20948::Interface::SSI:
      return format_to(ctx.out(), "SSI");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::PowerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::PowerMode power_mode, FormatContext &ctx) const {
    switch (power_mode) {
    case espp::icm20948::PowerMode::SLEEP:
      return format_to(ctx.out(), "Sleep");
    case espp::icm20948::PowerMode::LOW_POWER_ACCELEROMETER_ONLY:
      return format_to(ctx.out(), "Low power accelerometer only");
    case espp::icm20948::PowerMode::LOW_NOISE_ACCELEROMETER_ONLY:
      return format_to(ctx.out(), "Low noise accelerometer only");
    case espp::icm20948::PowerMode::GYROSCOPE_ONLY:
      return format_to(ctx.out(), "Gyroscope only");
    case espp::icm20948::PowerMode::MAGNETOMETER_ONLY:
      return format_to(ctx.out(), "Magnetometer only");
    case espp::icm20948::PowerMode::ACCELEROMETER_AND_GYROSCOPE:
      return format_to(ctx.out(), "Accelerometer and gyroscope");
    case espp::icm20948::PowerMode::ACCELEROMETER_AND_MAGNETOMETER:
      return format_to(ctx.out(), "Accelerometer and magnetometer");
    case espp::icm20948::PowerMode::NINE_AXIS:
      return format_to(ctx.out(), "Nine-axis");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::DutyCycleMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::DutyCycleMode mode, FormatContext &ctx) const {
    return format_to(ctx.out(), "Gyro: {}, Accel: {}, I2C: {}",
                     (mode & espp::icm20948::DutyCycleMode::GYRO_ONLY) != 0,
                     (mode & espp::icm20948::DutyCycleMode::ACCEL_ENABLED) != 0,
                     (mode & espp::icm20948::DutyCycleMode::I2C_MAST_ENABLED) != 0);
  }
};

template <> struct fmt::formatter<espp::icm20948::AccelerometerRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::AccelerometerRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::icm20948::AccelerometerRange::RANGE_16G:
      return format_to(ctx.out(), "±16g");
    case espp::icm20948::AccelerometerRange::RANGE_8G:
      return format_to(ctx.out(), "±8g");
    case espp::icm20948::AccelerometerRange::RANGE_4G:
      return format_to(ctx.out(), "±4g");
    case espp::icm20948::AccelerometerRange::RANGE_2G:
      return format_to(ctx.out(), "±2g");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::AccelerometerODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::AccelerometerODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::icm20948::AccelerometerODR::ODR_1600_HZ:
      return format_to(ctx.out(), "1600 Hz");
    case espp::icm20948::AccelerometerODR::ODR_800_HZ:
      return format_to(ctx.out(), "800 Hz");
    case espp::icm20948::AccelerometerODR::ODR_400_HZ:
      return format_to(ctx.out(), "400 Hz");
    case espp::icm20948::AccelerometerODR::ODR_200_HZ:
      return format_to(ctx.out(), "200 Hz");
    case espp::icm20948::AccelerometerODR::ODR_100_HZ:
      return format_to(ctx.out(), "100 Hz");
    case espp::icm20948::AccelerometerODR::ODR_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm20948::AccelerometerODR::ODR_25_HZ:
      return format_to(ctx.out(), "25 Hz");
    case espp::icm20948::AccelerometerODR::ODR_12_5_HZ:
      return format_to(ctx.out(), "12.5 Hz");
    case espp::icm20948::AccelerometerODR::ODR_6_25_HZ:
      return format_to(ctx.out(), "6.25 Hz");
    case espp::icm20948::AccelerometerODR::ODR_3_125_HZ:
      return format_to(ctx.out(), "3.125 Hz");
    case espp::icm20948::AccelerometerODR::ODR_1_5625_HZ:
      return format_to(ctx.out(), "1.5625 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::GyroscopeRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::GyroscopeRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::icm20948::GyroscopeRange::RANGE_2000DPS:
      return format_to(ctx.out(), "±2000°/s");
    case espp::icm20948::GyroscopeRange::RANGE_1000DPS:
      return format_to(ctx.out(), "±1000°/s");
    case espp::icm20948::GyroscopeRange::RANGE_500DPS:
      return format_to(ctx.out(), "±500°/s");
    case espp::icm20948::GyroscopeRange::RANGE_250DPS:
      return format_to(ctx.out(), "±250°/s");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::GyroscopeODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::GyroscopeODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::icm20948::GyroscopeODR::ODR_1600_HZ:
      return format_to(ctx.out(), "1600 Hz");
    case espp::icm20948::GyroscopeODR::ODR_800_HZ:
      return format_to(ctx.out(), "800 Hz");
    case espp::icm20948::GyroscopeODR::ODR_400_HZ:
      return format_to(ctx.out(), "400 Hz");
    case espp::icm20948::GyroscopeODR::ODR_200_HZ:
      return format_to(ctx.out(), "200 Hz");
    case espp::icm20948::GyroscopeODR::ODR_100_HZ:
      return format_to(ctx.out(), "100 Hz");
    case espp::icm20948::GyroscopeODR::ODR_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm20948::GyroscopeODR::ODR_25_HZ:
      return format_to(ctx.out(), "25 Hz");
    case espp::icm20948::GyroscopeODR::ODR_12_5_HZ:
      return format_to(ctx.out(), "12.5 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::ImuConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm20948::ImuConfig &config, FormatContext &ctx) const {
    return format_to(ctx.out(),
                     "Accelerometer: {{ range: {}, odr: {} }}, Gyroscope: {{ range: {}, odr: {} }}",
                     config.accelerometer_range, config.accelerometer_odr, config.gyroscope_range,
                     config.gyroscope_odr);
  }
};

template <> struct fmt::formatter<espp::icm20948::RawValue> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm20948::RawValue &raw, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {}, y: {}, z: {} }}", raw.x, raw.y, raw.z);
  }
};

template <> struct fmt::formatter<espp::icm20948::Value> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm20948::Value &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {:.2f}, y: {:.2f}, z: {:.2f} }}", value.x, value.y, value.z);
  }
};

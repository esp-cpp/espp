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

/// Gyroscope range
enum class GyroscopeRange : uint8_t {
  RANGE_250DPS = (0 << 1),  ///< ±250°/s
  RANGE_500DPS = (1 << 1),  ///< ±500°/s
  RANGE_1000DPS = (2 << 1), ///< ±1000°/s
  RANGE_2000DPS = (3 << 1), ///< ±2000°/s
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
  BW_218_HZ = 1,  ///< 218 Hz
  BW_123_HZ = 2,  ///< 123 Hz
  BW_66_HZ = 3,   ///< 66 Hz
  BW_34_HZ = 4,   ///< 34 Hz
  BW_17_HZ = 5,   ///< 17 Hz
  BW_8_HZ = 6,    ///< 8 Hz
  BW_4_HZ = 7,    ///< 4 Hz
};

/// @brief Sensor DLPF Bandwidth for accelerometer
enum class AccelerometerFilterBandwidth : uint8_t {
  BW_246_HZ = 0, ///< 246 Hz
  BW_111_HZ = 2, ///< 111 Hz
  BW_50_HZ = 3,  ///< 50 Hz
  BW_24_HZ = 4,  ///< 24 Hz
  BW_12_HZ = 5,  ///< 12 Hz
  BW_6_HZ = 6,   ///< 6 Hz
  BW_473_HZ = 7, ///< 473 Hz
};

/// @brief Sensor DLPF Bandwidth for both gyroscope
enum class GyroscopeFilterBandwidth : uint8_t {
  BW_196_HZ = 0, ///< 196 Hz
  BW_151_HZ = 1, ///< 151 Hz
  BW_119_HZ = 2, ///< 119 Hz
  BW_51_HZ = 3,  ///< 51 Hz
  BW_23_HZ = 4,  ///< 23 Hz
  BW_11_HZ = 5,  ///< 11 Hz
  BW_5_HZ = 6,   ///< 5 Hz
  BW_361_HZ = 7, ///< 361 Hz
};

/// @brief Averaging filter configuration settings for the accelerometer in low power mode
enum class AccelerometerAveraging : uint8_t {
  X1_4 = 0, ///< 1x averaging (or 4x depending on ACCEL_FCHOICE)
  X8 = 1,   ///< 8x averaging
  X16 = 2,  ///< 16x averaging
  X32 = 3,  ///< 32x averaging
};

/// @brief Averaging filter configuration settings for the gyroscope in low power mode
enum class GyroscopeAveraging : uint8_t {
  X1 = 0,   ///< 1x averaging
  X2 = 1,   ///< 2x averaging
  X4 = 2,   ///< 4x averaging
  X8 = 3,   ///< 8x averaging
  X16 = 4,  ///< 16x averaging
  X32 = 5,  ///< 32x averaging
  X64 = 6,  ///< 64x averaging
  X128 = 7, ///< 128x averaging
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
  GyroscopeRange gyroscope_range = GyroscopeRange::RANGE_250DPS;          ///< Gyroscope range
  uint8_t accelerometer_sample_rate_divider =
      0; ///< Accelerometer sample rate divider. Output data rate is
         ///< calculated as follows: 1 kHz / (1 + sample_rate_divider)
  uint8_t gyroscope_sample_rate_divider =
      0; ///< Gyroscope sample rate divider. Output data rate is
         ///< calculated as follows: 1 kHz / (1 + sample_rate_divider)
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

template <> struct fmt::formatter<espp::icm20948::DmpODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::DmpODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::icm20948::DmpODR::ODR_25_HZ:
      return format_to(ctx.out(), "25 Hz");
    case espp::icm20948::DmpODR::ODR_400_HZ:
      return format_to(ctx.out(), "400 Hz");
    case espp::icm20948::DmpODR::ODR_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm20948::DmpODR::ODR_100_HZ:
      return format_to(ctx.out(), "100 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::TemperatureFilterBandwidth> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::TemperatureFilterBandwidth bandwidth, FormatContext &ctx) const {
    switch (bandwidth) {
    case espp::icm20948::TemperatureFilterBandwidth::FILTER_OFF:
      return format_to(ctx.out(), "Filter off");
    case espp::icm20948::TemperatureFilterBandwidth::BW_218_HZ:
      return format_to(ctx.out(), "218 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_123_HZ:
      return format_to(ctx.out(), "123 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_66_HZ:
      return format_to(ctx.out(), "66 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_34_HZ:
      return format_to(ctx.out(), "34 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_17_HZ:
      return format_to(ctx.out(), "17 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_8_HZ:
      return format_to(ctx.out(), "8 Hz");
    case espp::icm20948::TemperatureFilterBandwidth::BW_4_HZ:
      return format_to(ctx.out(), "4 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::AccelerometerFilterBandwidth> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::AccelerometerFilterBandwidth bandwidth, FormatContext &ctx) const {
    switch (bandwidth) {
    case espp::icm20948::AccelerometerFilterBandwidth::BW_246_HZ:
      return format_to(ctx.out(), "246 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_111_HZ:
      return format_to(ctx.out(), "111 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_24_HZ:
      return format_to(ctx.out(), "24 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_12_HZ:
      return format_to(ctx.out(), "12 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_6_HZ:
      return format_to(ctx.out(), "6 Hz");
    case espp::icm20948::AccelerometerFilterBandwidth::BW_473_HZ:
      return format_to(ctx.out(), "473 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::GyroscopeFilterBandwidth> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::GyroscopeFilterBandwidth bandwidth, FormatContext &ctx) const {
    switch (bandwidth) {
    case espp::icm20948::GyroscopeFilterBandwidth::BW_196_HZ:
      return format_to(ctx.out(), "196 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_151_HZ:
      return format_to(ctx.out(), "151 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_119_HZ:
      return format_to(ctx.out(), "119 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_51_HZ:
      return format_to(ctx.out(), "51 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_23_HZ:
      return format_to(ctx.out(), "23 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_11_HZ:
      return format_to(ctx.out(), "11 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_5_HZ:
      return format_to(ctx.out(), "5 Hz");
    case espp::icm20948::GyroscopeFilterBandwidth::BW_361_HZ:
      return format_to(ctx.out(), "361 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm20948::MagnetometerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::MagnetometerMode mode, FormatContext &ctx) const {
    switch (mode) {
    case espp::icm20948::MagnetometerMode::POWER_DOWN:
      return format_to(ctx.out(), "Power down");
    case espp::icm20948::MagnetometerMode::TRIGGER_MODE:
      return format_to(ctx.out(), "Trigger mode");
    case espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_10_HZ:
      return format_to(ctx.out(), "Continuous mode 10 Hz");
    case espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_20_HZ:
      return format_to(ctx.out(), "Continuous mode 20 Hz");
    case espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_50_HZ:
      return format_to(ctx.out(), "Continuous mode 50 Hz");
    case espp::icm20948::MagnetometerMode::CONTINUOUS_MODE_100_HZ:
      return format_to(ctx.out(), "Continuous mode 100 Hz");
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
                     "{{ Accelerometer: {}, Gyroscope: {}, Accel Sample Rate Divider: {}, Gyro "
                     "Sample Rate Divider: {}, Magnetometer Mode: {} }}",
                     config.accelerometer_range, config.gyroscope_range,
                     config.accelerometer_sample_rate_divider, config.gyroscope_sample_rate_divider,
                     config.magnetometer_mode);
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

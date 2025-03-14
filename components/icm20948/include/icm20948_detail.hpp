#pragma once

#include <cstdint>

namespace espp {
/// @brief Namespace for the ICM20948 9-axis motion sensor
namespace icm20948 {
/// @brief Enum class for the interface type of the ICM20948
enum class Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SSI = 1, ///< Synchronous Serial Interface (SSI), which can be SPI or SSI
};

/// @brief Enum class for the ICM20948 low power duty cycle mode configuration
enum class LowPowerDutyCycle : uint8_t {
  NONE = 0,        ///< No sensors active in duty cycled mode
  GYRO = (1 << 4), ///< Gyroscope active in duty cycled mode. ODR is determined by GYRO_SMPLRT_DIV
  ACCEL =
      (1 << 5), ///< Accelerometer active in duty cycled mode. ODR is determined by ACCEL_SMPLRT_DIV
  GYRO_ACCEL = (1 << 4) | (1 << 5), ///< Gyroscope and accelerometer active in duty cycled mode. ODR
                                    ///< is determined by GYRO_SMPLRT_DIV and ACCEL_SMPLRT_DIV
  I2C_MST =
      (1 << 6), ///< I2C master active in duty cycled mode. ODR is determined by I2C_MST_ODR_CONFIG
  GYRO_ACCEL_I2C_MST =
      (1 << 4) | (1 << 5) | (1 << 6), ///< Gyroscope, accelerometer, and I2C master active in duty
                                      ///< cycled mode. ODR is determined by GYRO_SMPLRT_DIV,
                                      ///< ACCEL_SMPLRT_DIV, and I2C_MST_ODR_CONFIG
};

/// @brief Enum class for the ICM20948 interrupt types
enum class InterruptType : uint8_t {
  FSYNC = 0x01,          ///< FSYNC interrupt
  WAKE_ON_MOTION = 0x02, ///< Wake on motion interrupt
  DMP = 0x04,            ///< DMP interrupt
  DATA_READY = 0x08,     ///< Data ready interrupt
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
  RANGE_16G = 0, ///< ±16g
  RANGE_8G = 1,  ///< ±8g
  RANGE_4G = 2,  ///< ±4g
  RANGE_2G = 3,  ///< ±2g
};

/// Accelerometer power mode
enum class AccelerometerPowerMode : uint8_t {
  OFF = 0,       ///< Off
  ON = 1,        ///< On
  LOW_POWER = 2, ///< Low power
  LOW_NOISE = 3, ///< Low noise
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
  RANGE_2000DPS = 0, ///< ±2000°/s
  RANGE_1000DPS = 1, ///< ±1000°/s
  RANGE_500DPS = 2,  ///< ±500°/s
  RANGE_250DPS = 3,  ///< ±250°/s
};

/// Gyroscope power mode
enum class GyroscopePowerMode : uint8_t {
  OFF = 0,       ///< Off
  STANDBY = 1,   ///< Standby
  LOW_NOISE = 3, ///< Low noise
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
    float x;     ///< X-axis value
    float pitch; ///< Roll angle
  };
  union {
    float y;    ///< Y-axis value
    float roll; ///< Pitch angle
  };
  union {
    float z;   ///< Z-axis value
    float yaw; ///< Yaw angle
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

#include "format.hpp"

// for libfmt printing of relevant imu types and structs
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

template <> struct fmt::formatter<espp::icm20948::AccelerometerPowerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::AccelerometerPowerMode power_mode, FormatContext &ctx) const {
    switch (power_mode) {
    case espp::icm20948::AccelerometerPowerMode::OFF:
      return format_to(ctx.out(), "Off");
    case espp::icm20948::AccelerometerPowerMode::ON:
      return format_to(ctx.out(), "On");
    case espp::icm20948::AccelerometerPowerMode::LOW_POWER:
      return format_to(ctx.out(), "Low power");
    case espp::icm20948::AccelerometerPowerMode::LOW_NOISE:
      return format_to(ctx.out(), "Low noise");
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

template <> struct fmt::formatter<espp::icm20948::GyroscopePowerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm20948::GyroscopePowerMode power_mode, FormatContext &ctx) const {
    switch (power_mode) {
    case espp::icm20948::GyroscopePowerMode::OFF:
      return format_to(ctx.out(), "Off");
    case espp::icm20948::GyroscopePowerMode::STANDBY:
      return format_to(ctx.out(), "Standby");
    case espp::icm20948::GyroscopePowerMode::LOW_NOISE:
      return format_to(ctx.out(), "Low noise");
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

template <> struct fmt::formatter<espp::icm20948::ComplimentaryAngle> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm20948::ComplimentaryAngle &angle, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ roll: {:.2f}, pitch: {:.2f} }}", angle.roll, angle.pitch);
  }
};

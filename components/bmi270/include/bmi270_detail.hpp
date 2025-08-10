#pragma once

#include <cstdint>

namespace espp {
/// @brief Namespace for the BMI270 6-axis motion sensor
namespace bmi270 {

/// @brief Enum class for the interface type of the BMI270
enum class Interface : uint8_t {
  I2C = 0, ///< Inter-Integrated Circuit (I2C)
  SPI = 1, ///< Serial Peripheral Interface (SPI)
};

/// Accelerometer range (±g)
enum class AccelerometerRange : uint8_t {
  RANGE_2G = 0x00,  ///< ±2g
  RANGE_4G = 0x01,  ///< ±4g
  RANGE_8G = 0x02,  ///< ±8g
  RANGE_16G = 0x03, ///< ±16g
};

/// Accelerometer output data rate
enum class AccelerometerODR : uint8_t {
  ODR_0_78_HZ = 0x01, ///< 0.78 Hz
  ODR_1_56_HZ = 0x02, ///< 1.56 Hz
  ODR_3_12_HZ = 0x03, ///< 3.12 Hz
  ODR_6_25_HZ = 0x04, ///< 6.25 Hz
  ODR_12_5_HZ = 0x05, ///< 12.5 Hz
  ODR_25_HZ = 0x06,   ///< 25 Hz
  ODR_50_HZ = 0x07,   ///< 50 Hz
  ODR_100_HZ = 0x08,  ///< 100 Hz
  ODR_200_HZ = 0x09,  ///< 200 Hz
  ODR_400_HZ = 0x0A,  ///< 400 Hz
  ODR_800_HZ = 0x0B,  ///< 800 Hz
  ODR_1600_HZ = 0x0C, ///< 1600 Hz
};

/// Accelerometer bandwidth parameter (affects filter)
enum class AccelerometerBandwidth : uint8_t {
  OSR4_AVG1 = 0x00,   ///< OSR4, avg1
  OSR2_AVG2 = 0x01,   ///< OSR2, avg2
  NORMAL_AVG4 = 0x02, ///< Normal, avg4
  CIC_AVG8 = 0x03,    ///< CIC, avg8
  RES_AVG16 = 0x04,   ///< RES, avg16
  RES_AVG32 = 0x05,   ///< RES, avg32
  RES_AVG64 = 0x06,   ///< RES, avg64
  RES_AVG128 = 0x07,  ///< RES, avg128
};

/// Gyroscope range (±dps - degrees per second)
enum class GyroscopeRange : uint8_t {
  RANGE_2000DPS = 0x00, ///< ±2000°/s
  RANGE_1000DPS = 0x01, ///< ±1000°/s
  RANGE_500DPS = 0x02,  ///< ±500°/s
  RANGE_250DPS = 0x03,  ///< ±250°/s
  RANGE_125DPS = 0x04,  ///< ±125°/s
};

/// Gyroscope output data rate
enum class GyroscopeODR : uint8_t {
  ODR_25_HZ = 0x06,   ///< 25 Hz
  ODR_50_HZ = 0x07,   ///< 50 Hz
  ODR_100_HZ = 0x08,  ///< 100 Hz
  ODR_200_HZ = 0x09,  ///< 200 Hz
  ODR_400_HZ = 0x0A,  ///< 400 Hz
  ODR_800_HZ = 0x0B,  ///< 800 Hz
  ODR_1600_HZ = 0x0C, ///< 1600 Hz
  ODR_3200_HZ = 0x0D, ///< 3200 Hz
  ODR_6400_HZ = 0x0E, ///< 6400 Hz
};

/// Gyroscope bandwidth parameter
enum class GyroscopeBandwidth : uint8_t {
  OSR4_MODE = 0x00,   ///< OSR4 mode
  OSR2_MODE = 0x01,   ///< OSR2 mode
  NORMAL_MODE = 0x02, ///< Normal mode
  CIC_MODE = 0x03,    ///< CIC mode
};

/// Gyroscope performance mode
enum class GyroscopePerformanceMode : uint8_t {
  POWER_OPTIMIZED = 0x00,      ///< Power optimized
  PERFORMANCE_OPTIMIZED = 0x01 ///< Performance optimized (lower noise)
};

/// FIFO mode configuration
enum class FifoMode : uint8_t {
  BYPASS = 0x00, ///< FIFO bypass mode
  FIFO = 0x01,   ///< FIFO mode
  STREAM = 0x02, ///< Stream mode
};

/// Advanced feature configurations for BMI270's built-in features
enum class FeatureConfig : uint16_t {
  ANY_MOTION = 0x00,           ///< Any motion detection
  NO_MOTION = 0x04,            ///< No motion detection
  SIGNIFICANT_MOTION = 0x08,   ///< Significant motion detection
  STEP_COUNTER = 0x0C,         ///< Step counter
  STEP_DETECTOR = 0x10,        ///< Step detector
  STEP_ACTIVITY = 0x14,        ///< Step activity recognition
  WRIST_GESTURE = 0x18,        ///< Wrist gesture detection
  WRIST_WEAR_WAKEUP = 0x1C,    ///< Wrist wear wakeup
  ACTIVITY_RECOGNITION = 0x20, ///< Activity recognition
  GYRO_SELF_OFFSET = 0x24,     ///< Gyroscope self-offset compensation
};

/// Interrupt pin configuration
enum class InterruptPin : uint8_t {
  INT1 = 0, ///< Interrupt pin 1
  INT2 = 1, ///< Interrupt pin 2
};

/// Interrupt output configuration
enum class InterruptOutput : uint8_t {
  PUSH_PULL = 0,  ///< Push-pull output
  OPEN_DRAIN = 1, ///< Open-drain output
};

/// Interrupt active level
enum class InterruptLevel : uint8_t {
  ACTIVE_LOW = 0,  ///< Active low
  ACTIVE_HIGH = 1, ///< Active high
};

/// IMU Configuration structure
struct ImuConfig {
  AccelerometerRange accelerometer_range = AccelerometerRange::RANGE_4G; ///< Accelerometer range
  AccelerometerODR accelerometer_odr = AccelerometerODR::ODR_100_HZ;     ///< Accelerometer ODR
  AccelerometerBandwidth accelerometer_bandwidth =
      AccelerometerBandwidth::NORMAL_AVG4; ///< Accel bandwidth

  GyroscopeRange gyroscope_range = GyroscopeRange::RANGE_2000DPS;           ///< Gyroscope range
  GyroscopeODR gyroscope_odr = GyroscopeODR::ODR_100_HZ;                    ///< Gyroscope ODR
  GyroscopeBandwidth gyroscope_bandwidth = GyroscopeBandwidth::NORMAL_MODE; ///< Gyro bandwidth
  GyroscopePerformanceMode gyroscope_performance_mode =
      GyroscopePerformanceMode::POWER_OPTIMIZED; ///< Gyro performance

  bool enable_advanced_features = false; ///< Enable advanced motion features
  FifoMode fifo_mode = FifoMode::BYPASS; ///< FIFO configuration
};

/// Raw IMU data (16-bit signed)
struct RawValue {
  int16_t x; ///< Raw X-axis value
  int16_t y; ///< Raw Y-axis value
  int16_t z; ///< Raw Z-axis value
};

/// Processed IMU data (floating point)
struct Value {
  union {
    struct {
      float x; ///< X-axis value
      float y; ///< Y-axis value
      float z; ///< Z-axis value
    };
    struct {
      float roll;  ///< Roll value (for orientation)
      float pitch; ///< Pitch value (for orientation)
      float yaw;   ///< Yaw value (for orientation)
    };
    float values[3]; ///< Array access
  };
};

/// Advanced feature data structures
struct MotionData {
  bool any_motion_detected = false;         ///< Any motion detection status
  bool no_motion_detected = false;          ///< No motion detection status
  bool significant_motion_detected = false; ///< Significant motion detection status
};

struct StepData {
  uint32_t step_count = 0;    ///< Total step count
  bool step_detected = false; ///< Step detector status
  uint8_t activity_type = 0;  ///< Activity type (0=unknown, 1=still, 2=walk, 3=run)
};

struct WristData {
  bool wrist_wear_status = false; ///< Wrist wear detection status
  bool gesture_detected = false;  ///< Wrist gesture detection status
  uint8_t gesture_type = 0;       ///< Gesture type
};

/// Interrupt configuration structure
struct InterruptConfig {
  InterruptPin pin = InterruptPin::INT1;                     ///< Which interrupt pin to use
  InterruptOutput output_type = InterruptOutput::PUSH_PULL;  ///< Output type
  InterruptLevel active_level = InterruptLevel::ACTIVE_HIGH; ///< Active level
  bool latch_mode = false;                                   ///< Latch interrupt until cleared
  bool enable_data_ready = false;                            ///< Enable data ready interrupt
  bool enable_fifo_watermark = false;                        ///< Enable FIFO watermark interrupt
  bool enable_fifo_full = false;                             ///< Enable FIFO full interrupt
  bool enable_any_motion = false;                            ///< Enable any motion interrupt
  bool enable_no_motion = false;                             ///< Enable no motion interrupt
  bool enable_significant_motion = false; ///< Enable significant motion interrupt
  bool enable_step_detector = false;      ///< Enable step detector interrupt
  bool enable_wrist_wear_wakeup = false;  ///< Enable wrist wear wakeup interrupt
};

/// FIFO configuration structure
struct FifoConfig {
  FifoMode mode = FifoMode::BYPASS; ///< FIFO mode
  bool enable_accel_data = true;    ///< Include accelerometer data in FIFO
  bool enable_gyro_data = true;     ///< Include gyroscope data in FIFO
  bool enable_temperature = false;  ///< Include temperature data in FIFO
  bool enable_header = true;        ///< Include header in FIFO data
  uint16_t watermark_level = 1024;  ///< FIFO watermark level (bytes)
};

} // namespace bmi270
} // namespace espp

#include "format.hpp"

// Format support for BMI270 types
template <> struct fmt::formatter<espp::bmi270::AccelerometerRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::bmi270::AccelerometerRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::bmi270::AccelerometerRange::RANGE_2G:
      return format_to(ctx.out(), "±2g");
    case espp::bmi270::AccelerometerRange::RANGE_4G:
      return format_to(ctx.out(), "±4g");
    case espp::bmi270::AccelerometerRange::RANGE_8G:
      return format_to(ctx.out(), "±8g");
    case espp::bmi270::AccelerometerRange::RANGE_16G:
      return format_to(ctx.out(), "±16g");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::bmi270::GyroscopeRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::bmi270::GyroscopeRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::bmi270::GyroscopeRange::RANGE_2000DPS:
      return format_to(ctx.out(), "±2000°/s");
    case espp::bmi270::GyroscopeRange::RANGE_1000DPS:
      return format_to(ctx.out(), "±1000°/s");
    case espp::bmi270::GyroscopeRange::RANGE_500DPS:
      return format_to(ctx.out(), "±500°/s");
    case espp::bmi270::GyroscopeRange::RANGE_250DPS:
      return format_to(ctx.out(), "±250°/s");
    case espp::bmi270::GyroscopeRange::RANGE_125DPS:
      return format_to(ctx.out(), "±125°/s");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::bmi270::ImuConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::bmi270::ImuConfig &config, FormatContext &ctx) const {
    return format_to(ctx.out(),
                     "Accelerometer: {{ range: {}, odr: {} }}, Gyroscope: {{ range: {}, odr: {} }}",
                     config.accelerometer_range, static_cast<int>(config.accelerometer_odr),
                     config.gyroscope_range, static_cast<int>(config.gyroscope_odr));
  }
};

template <> struct fmt::formatter<espp::bmi270::RawValue> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::bmi270::RawValue &raw, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {}, y: {}, z: {} }}", raw.x, raw.y, raw.z);
  }
};

template <> struct fmt::formatter<espp::bmi270::Value> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::bmi270::Value &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {:.2f}, y: {:.2f}, z: {:.2f} }}", value.x, value.y, value.z);
  }
};

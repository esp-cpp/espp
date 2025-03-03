#pragma once

#include "base_peripheral.hpp"

namespace espp {
/// @brief Namespace for the ICM42607 6-axis motion sensor
namespace icm42607 {
/// @brief Enum class for the interface type of the ICM42607
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

/// Sensor DLPF Bandwidth for both accelerometer and gyroscope
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
  float x; ///< X-axis value
  float y; ///< Y-axis value
  float z; ///< Z-axis value
};

/// Complimentary angle
struct ComplimentaryAngle {
  float roll;  ///< Roll angle
  float pitch; ///< Pitch angle
};

/// @brief Enum class for the ICM42607 interrupt configuration
enum class InterruptDriveMode {
  OPEN_DRAIN = 0, ///< Open drain
  PUSH_PULL = 1,  ///< Push-pull
};

/// @brief Enum class for the ICM42607 interrupt configuration
enum class InterruptPolarity {
  ACTIVE_LOW = 0,  ///< Active low
  ACTIVE_HIGH = 1, ///< Active high
};

/// @brief Enum class for the ICM42607 interrupt configuration
enum class InterruptMode {
  PULSED = 0,  ///< Pulsed
  LATCHED = 1, ///< Latched
};

/// @brief Struct for the ICM42607 interrupt configuration
struct InterruptConfig {
  InterruptDriveMode drive_mode; ///< Drive mode
  InterruptPolarity polarity;    ///< Polarity
  InterruptMode mode;            ///< Mode
};
} // namespace icm42607

/// @brief Class for the ICM42607 6-axis motion sensor
/// @tparam Interface The interface type of the ICM42607
///
/// @note The ICM42607 is a 6-axis motion sensor that can be interfaced with
/// either I2C or SPI.
///
/// @note The Icm42607 can work with both the Icm42607 and Icm42670, as they
/// are the same sensor, but the Icm42670 has a different I2C address.
///
/// The ICM42607 has a 3-axis gyroscope and a 3-axis accelerometer. The
/// ICM42607 has a 16-bit ADC for the gyroscope and a 16-bit ADC for the
/// accelerometer. The gyroscope has a full-scale range of up to ±2000°/s and
/// the accelerometer has a full-scale range of up to ±16g.
///
/// The ICM42607 has a FIFO buffer that can store up to 2.25 kilobytes of
/// data. The FIFO buffer can be configured to store gyroscope data,
/// accelerometer data, or both. The FIFO buffer can be configured to store
/// data in either FIFO mode or stream mode. In FIFO mode, the FIFO buffer
/// stores data until the buffer is full, at which point the FIFO buffer stops
/// storing data. In stream mode, the FIFO buffer stores data continuously,
/// overwriting the oldest data when the buffer is full.
///
/// The ICM42607 has a Digital Motion Processor (DMP) that can be used to
/// process the data from the gyroscope and accelerometer. The DMP can be used
/// to calculate the orientation of the ICM42607, the linear acceleration of
/// the ICM42607, and the angular velocity of the ICM42607. The DMP can also
/// be used to detect motion events, such as tap events, shake events, and
/// orientation events.
///
/// The ICM42607 has a built-in temperature sensor that can be used to measure
/// the temperature of the ICM42607. The temperature sensor has a resolution
/// of 0.5°C and an accuracy of ±2°C.
///
/// It supports low-noise 6-axis measurement with a current consumption at
/// only 0.55 mA and a sleep mode current consumption of only 3.5 µA.
///
/// For more information see, the datasheet:
/// https://invensense.tdk.com/wp-content/uploads/2021/07/ds-000451_icm-42670-p-datasheet.pdf
template <icm42607::Interface Interface = icm42607::Interface::I2C>
class Icm42607 : public espp::BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C> {
  // Since the BasePeripheral is a dependent base class (e.g. its template
  // parameters depend on our template parameters), we need to use the `using`
  // keyword to bring in the functions / members we want to use, otherwise we
  // have to either use `this->` or explicitly scope each call, which clutters
  // the code / is annoying. This is needed because of the two phases of name
  // lookups for templates.
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::write_u8_to_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::write_many_to_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::read_u16_from_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::read_many_from_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::set_bits_in_register;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      0x68; ///< Default I2C address of the ICM42607 with AD0 low
  static constexpr uint8_t DEFAULT_ADDRESS_AD0_HIGH =
      0x69; ///< Default I2C address of the ICM42607 with AD0 high

  using FifoMode = icm42607::FifoMode;                             ///< FIFO mode
  using AccelerometerRange = icm42607::AccelerometerRange;         ///< Accelerometer range
  using AccelerometerPowerMode = icm42607::AccelerometerPowerMode; ///< Accelerometer power mode
  using AccelerometerODR = icm42607::AccelerometerODR;     ///< Accelerometer output data rate
  using GyroscopeRange = icm42607::GyroscopeRange;         ///< Gyroscope range
  using GyroscopePowerMode = icm42607::GyroscopePowerMode; ///< Gyroscope power mode
  using GyroscopeODR = icm42607::GyroscopeODR;             ///< Gyroscope output data rate
  using ImuConfig = icm42607::ImuConfig;                   ///< IMU configuration
  using RawValue = icm42607::RawValue;                     ///< Raw IMU data
  using Value = icm42607::Value;                           ///< IMU data
  using ComplimentaryAngle = icm42607::ComplimentaryAngle; ///< Complimentary angle
  using InterruptDriveMode = icm42607::InterruptDriveMode; ///< Interrupt drive mode
  using InterruptPolarity = icm42607::InterruptPolarity;   ///< Interrupt polarity
  using InterruptMode = icm42607::InterruptMode;           ///< Interrupt mode
  using InterruptConfig = icm42607::InterruptConfig;       ///< Interrupt configuration

  /// Configuration struct for the ICM42607
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the ICM42607
    BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::write_fn write =
        nullptr; ///< Write function
    BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>::read_fn read =
        nullptr;                                          ///< Read function
    ImuConfig imu_config;                                 ///< IMU configuration
    bool auto_init{true};                                 ///< Automatically initialize the ICM42607
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /// Constructor
  /// @param config The configuration
  explicit Icm42607(const Config &config)
      : BasePeripheral<uint8_t, Interface == icm42607::Interface::I2C>({}, "Icm42607",
                                                                       config.log_level)
      , imu_config_(config.imu_config) {
    if constexpr (Interface == icm42607::Interface::I2C) {
      set_address(config.device_address);
    }
    set_write(config.write);
    set_read(config.read);
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
    }
  }

  /// Initialize the ICM42607
  /// @param ec The error code to set if an error occurs
  /// @return True if the ICM42607 was initialized successfully, false otherwise
  bool init(std::error_code &ec) {
    auto device_id = get_device_id(ec);
    if (device_id != ICM42607_ID && device_id != ICM42670_ID) {
      logger_.error("Invalid device ID: 0x{:02X}", device_id);
      return false;
    }

    // set the configuration
    if (!set_config(imu_config_, ec)) {
      return false;
    }

    // turn on the accelerometer
    if (!set_accelerometer_power_mode(AccelerometerPowerMode::LOW_NOISE, ec)) {
      return false;
    }

    // turn on the gyroscope
    if (!set_gyroscope_power_mode(GyroscopePowerMode::LOW_NOISE, ec)) {
      return false;
    }

    return true;
  }

  /// Get the device ID
  /// @param ec The error code to set if an error occurs
  /// @return The device ID
  uint8_t get_device_id(std::error_code &ec) {
    return read_u8_from_register(static_cast<uint8_t>(Register::WHO_AM_I), ec);
  }

  /// Set the IMU configuration
  /// @param imu_config The IMU configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the configuration was set successfully, false otherwise
  bool set_config(const ImuConfig &imu_config, std::error_code &ec) {
    // save the config
    imu_config_ = imu_config;

    // the config is set as two bytes containing the configuration for the
    // accelerometer and gyroscope (full scale range and output data rate)
    uint8_t data[2] = {// first byte is gyro fs + odr
                       uint8_t((static_cast<uint8_t>(imu_config.gyroscope_range) & 0x3) << 5 |
                               (static_cast<uint8_t>(imu_config.gyroscope_odr) & 0x0F)),
                       // second byte is accel fs + odr
                       uint8_t((static_cast<uint8_t>(imu_config.accelerometer_range) & 0x3) << 5 |
                               (static_cast<uint8_t>(imu_config.accelerometer_odr) & 0x0F))};

    write_many_to_register(static_cast<uint8_t>(Register::GYRO_CONFIG0), data, 2, ec);
    if (ec) {
      return false;
    }

    return true;
  }

  /// Set the Accelerometer power mode
  /// @param power_mode The power mode
  /// @param ec The error code to set if an error occurs
  /// @return True if the power mode was set successfully, false otherwise
  bool set_accelerometer_power_mode(AccelerometerPowerMode power_mode, std::error_code &ec) {
    set_bits_in_register(static_cast<uint8_t>(Register::PWR_MGMT0),
                         static_cast<uint8_t>(power_mode) & 0x03, ec);
    return !ec;
  }

  /// Set the Gyroscope power mode
  /// @param power_mode The power mode
  /// @param ec The error code to set if an error occurs
  /// @return True if the power mode was set successfully, false otherwise
  bool set_gyroscope_power_mode(GyroscopePowerMode power_mode, std::error_code &ec) {
    set_bits_in_register(static_cast<uint8_t>(Register::PWR_MGMT0),
                         (static_cast<uint8_t>(power_mode) & 0x03) << 2, ec);
    return !ec;
  }

  /// Read the accelerometer sensitivity
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer sensitivity in g/LSB
  float get_accelerometer_sensitivity(std::error_code &ec) {
    // read the byte from the register
    uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::ACCEL_CONFIG0), ec);
    if (ec) {
      return 0.0f;
    }
    // get the range
    AccelerometerRange range = static_cast<AccelerometerRange>((data >> 5) & 0x03);
    // convert to sensitivity
    return accelerometer_range_to_sensitivty(range);
  }

  /// Read the gyroscope sensitivity
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope sensitivity in °/s/LSB
  float get_gyroscope_sensitivity(std::error_code &ec) {
    // read the byte from the register
    uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::GYRO_CONFIG0), ec);
    if (ec) {
      return 0.0f;
    }
    // get the range
    GyroscopeRange range = static_cast<GyroscopeRange>((data >> 5) & 0x03);
    // convert to sensitivity
    return gyroscope_range_to_sensitivty(range);
  }

  /// Read the accelerometer data
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer data
  Value get_accelerometer(std::error_code &ec) {
    RawValue raw = get_accelerometer_raw(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    float sensitivity = get_accelerometer_sensitivity(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    return {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
  }

  /// Read the gyroscope data
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope data
  Value get_gyroscope(std::error_code &ec) {
    RawValue raw = get_gyroscope_raw(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    float sensitivity = get_gyroscope_sensitivity(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    return {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
  }

  /// Read the temperature
  /// @param ec The error code to set if an error occurs
  /// @return The temperature in °C
  float get_temperature(std::error_code &ec) {
    uint16_t raw = get_temperature_raw(ec);
    if (ec) {
      return 0.0f;
    }
    return static_cast<float>(raw) / 128.0f + 25.0f; // 132.48 + 25
  }

  /// Configure the FIFO buffer
  /// @param mode The FIFO mode
  /// @param bypassed True if the FIFO is bypassed, false otherwise
  /// @param ec The error code to set if an error occurs
  /// @return True if the FIFO buffer was configured successfully, false otherwise
  bool configure_fifo(FifoMode mode, bool bypassed, std::error_code &ec) {
    // FIFO_MODE is bit 1, FIFO_BYPASS is bit 0
    uint8_t data = (static_cast<uint8_t>(mode) << 1) | (bypassed ? 1 : 0);
    write_u8_to_register(static_cast<uint8_t>(Register::FIFO_CONFIG1), data, ec);
    return !ec;
  }

  /// Flush the FIFO buffer
  /// @param ec The error code to set if an error occurs
  /// @return True if the FIFO buffer was flushed successfully, false otherwise
  bool fifo_flush(std::error_code &ec) {
    set_bits_in_register(static_cast<uint8_t>(Register::SIGNAL_PATH_RESET), FIFO_FLUSH, ec);
    return !ec;
  }

  /// Get the FIFO count
  /// @param ec The error code to set if an error occurs
  /// @return The FIFO count.
  /// @note The count will be either the number of bytes (if FIFO_COUNT_FORMAT
  ///       is 0) or the number of records (if FIFO_COUNT_FORMAT is 1). A
  ///       record is 16 bytes for header + gyro + accel + temp sensor data +
  ///       time stamp, OR 8 bytes for header + gyro/accel + temp sensor data.
  /// @see set_fifo_count_format
  uint16_t fifo_count(std::error_code &ec) {
    uint8_t data[2];
    read_many_from_register(static_cast<uint8_t>(Register::FIFO_COUNTH), data, 2, ec);
    if (ec) {
      return 0;
    }
    return (data[0] << 8) | data[1];
  }

  /// Set the FIFO count format to count bytes
  /// @param ec The error code to set if an error occurs
  /// @return True if the FIFO count format was set successfully, false
  ///         otherwise
  /// @see fifo_count
  bool set_fifo_count_bytes(std::error_code &ec) {
    uint8_t bitmask = 1 << 6;
    clear_bits_in_register(static_cast<uint8_t>(Register::INTERFACE_CONFIG0), bitmask, ec);
    return !ec;
  }

  /// Set the FIFO count format to count records
  /// @param ec The error code to set if an error occurs
  /// @return True if the FIFO count format was set successfully, false
  ///         otherwise
  /// @see fifo_count
  bool set_fifo_count_records(std::error_code &ec) {
    uint8_t bitmask = 1 << 6;
    set_bits_in_register(static_cast<uint8_t>(Register::INTERFACE_CONFIG0), bitmask, ec);
    return !ec;
  }

  /// Get the FIFO data
  /// @param data The buffer to store the FIFO data
  /// @param size The size of the buffer
  /// @param ec The error code to set if an error occurs
  /// @return The number of bytes read
  size_t fifo_data(uint8_t *data, size_t size, std::error_code &ec) {
    return read(static_cast<uint8_t>(Register::FIFO_DATA), data, size, ec);
  }

  /// Configure interrupt 1
  /// @param config The interrupt configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the interrupt was configured successfully, false otherwise
  bool configure_interrupt_1(const InterruptConfig &config, std::error_code &ec) {
    // interrupt 1 is bits 0-2 in INT_CONFIG (MODE << 2) | (POLARITY << 1) | DRIVE_MODE
    uint8_t data = (static_cast<uint8_t>(config.mode) << 2) |
                   (static_cast<uint8_t>(config.polarity) << 1) |
                   static_cast<uint8_t>(config.drive_mode);
    set_bits_in_register(static_cast<uint8_t>(Register::INT_CONFIG), data, ec);
    return !ec;
  }

  /// Configure interrupt 2
  /// @param config The interrupt configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the interrupt was configured successfully, false otherwise
  bool configure_interrupt_2(const InterruptConfig &config, std::error_code &ec) {
    // interrupt 2 is bits 3-5 in INT_CONFIG (MODE << 5) | (POLARITY << 4) | (DRIVE_MODE << 3)
    uint8_t data = (static_cast<uint8_t>(config.mode) << 5) |
                   (static_cast<uint8_t>(config.polarity) << 4) |
                   (static_cast<uint8_t>(config.drive_mode) << 3);
    set_bits_in_register(static_cast<uint8_t>(Register::INT_CONFIG), data, ec);
    return !ec;
  }

  /// Read whether the data is ready
  /// @param ec The error code to set if an error occurs
  /// @return True if the data is ready, false otherwise
  /// @note This will clear the data ready interrupt bit, which is
  ///       automatically set to 1 when a Data Ready interrupt is generated.
  bool is_data_ready(std::error_code &ec) {
    // clearing the bit just involves reading the register
    uint8_t data_ready =
        read_u8_from_register(static_cast<uint8_t>(Register::INT_STATUS_DATA_READY), ec);
    return !ec && (data_ready & 0x01);
  }

  /// Compute the complimentary angle
  /// @param dt The time delta
  /// @param prev_angle The previous angle
  /// @param accel The accelerometer data
  /// @param gyro The gyroscope data
  /// @param alpha The alpha value
  /// @return The complimentary angle
  static ComplimentaryAngle complimentary_filter(float dt, const ComplimentaryAngle &prev_angle,
                                                 const Value &accel, const Value &gyro,
                                                 float alpha = 0.99f) {
    float roll = atan2f(accel.y, accel.z) * RAD_TO_DEG;
    float pitch = atan2f(accel.x, accel.z) * RAD_TO_DEG;

    if (dt == 0) {
      return {
          .roll = roll,
          .pitch = pitch,
      };
    }

    return {
        .roll = alpha * (prev_angle.roll + gyro.x * dt) + (1.0f - alpha) * roll,
        .pitch = alpha * (prev_angle.pitch + gyro.y * dt) + (1.0f - alpha) * pitch,
    };
  }

protected:
  static constexpr float RAD_TO_DEG = 57.27272727f;   ///< Radians to degrees
  static constexpr float DEG_TO_RAD = 0.01745329252f; ///< Degrees to radians

  static constexpr uint8_t ICM42607_ID = 0x60; ///< ICM42607 ID
  static constexpr uint8_t ICM42670_ID = 0x67; ///< ICM42670 ID

  static constexpr float GYRO_FS_2000_SENS = 16.4f; ///< Gyroscope sensitivity for ±2000°/s
  static constexpr float GYRO_FS_1000_SENS = 32.8f; ///< Gyroscope sensitivity for ±1000°/s
  static constexpr float GYRO_FS_500_SENS = 65.5f;  ///< Gyroscope sensitivity for ±500°/s
  static constexpr float GYRO_FS_250_SENS = 131.0f; ///< Gyroscope sensitivity for ±250°/s

  static constexpr float ACCEL_FS_16G_SENS = 2048.0f; ///< Accelerometer sensitivity for ±16g
  static constexpr float ACCEL_FS_8G_SENS = 4096.0f;  ///< Accelerometer sensitivity for ±8g
  static constexpr float ACCEL_FS_4G_SENS = 8192.0f;  ///< Accelerometer sensitivity for ±4g
  static constexpr float ACCEL_FS_2G_SENS = 16384.0f; ///< Accelerometer sensitivity for ±2g

  static constexpr uint8_t FIFO_FLUSH = (1 << 2); ///< FIFO flush bit, in register SIGNAL_PATH_RESET

  /// Register addresses
  enum class Register : uint8_t {
    /// USER BANK 0

    MCLK_READY_STATUS = 0x00, ///< MCLK ready status register
    DEVICE_CONFIG = 0x01,     ///< Device configuration register
    SIGNAL_PATH_RESET = 0x02, ///< Signal path reset register

    DRIVE_CONFIG_1 = 0x03, ///< Drive configuration 1 register
    DRIVE_CONFIG_2 = 0x04, ///< Drive configuration 2 register
    DRIVE_CONFIG_3 = 0x05, ///< Drive configuration 3 register

    INT_CONFIG = 0x06, ///< Interrupt configuration register

    TEMP_DATA = 0x09,  ///< Temperature data register
    ACCEL_DATA = 0x0B, ///< Accelerometer data register
    GYRO_DATA = 0x11,  ///< Gyroscope data register

    APEX_DATA4 = 0x1D, ///< APEX data 4 register, contains FF_DUR_L (freefall duration lower byte)
    APEX_DATA5 = 0x1E, ///< APEX data 5 register, contains FF_DUR_H (freefall duration higher byte)

    PWR_MGMT0 = 0x1F, ///< Power management register

    GYRO_CONFIG0 = 0x20,  ///< Gyroscope configuration register
    ACCEL_CONFIG0 = 0x21, ///< Accelerometer configuration register
    TEMP_CONFIG = 0x22,   ///< Temperature configuration register. Stores the configuration for the
                          ///< temperature DLPF BW.
    GYRO_CONFIG1 = 0x23,  ///< Gyroscope configuration 1 register. Stores the configuration for the
                          ///< gyroscope LPF BW.
    ACCEL_CONFIG1 = 0x24, ///< Accelerometer configuration 1 register. Stores the configuration for
                          ///< the accelerometer LPF BW.

    APEX_CONFIG0 = 0x25, ///< APEX configuration 0 register
    APEX_CONFIG1 = 0x26, ///< APEX configuration 1 register

    WOM_CONFIG0 = 0x27, ///< Wake-on-motion (WOM) configuration 0 register

    FIFO_CONFIG1 = 0x28, ///< FIFO configuration 1 register
    FIFO_CONFIG2 = 0x29, ///< FIFO configuration 2 register
    FIFO_CONFIG3 = 0x2A, ///< FIFO configuration 3 register

    INT_SOURCE0 = 0x2B, ///< Interrupt source 0 register
    INT_SOURCE1 = 0x2C, ///< Interrupt source 1 register
    // Datasheet doesn't have INT_SOURCE2???
    INT_SOURCE3 = 0x2D, ///< Interrupt source 3 register
    INT_SOURCE4 = 0x2E, ///< Interrupt source 4 register

    FIFO_LOST_PKT0 =
        0x2F, ///< FIFO lost packet 0 register. Low byte for number of packets lost in the FIFO
    FIFO_LOST_PKT1 =
        0x30, ///< FIFO lost packet 1 register. High byte for number of packets lost in the FIFO

    APEX_DATA0 = 0x31, ///< APEX data 0 register. Contains pedometer output (lower byte step count)
    APEX_DATA1 = 0x32, ///< APEX data 1 register. Contains pedometer output (higher byte step count)
    APEX_DATA2 = 0x33, ///< APEX data 2 register. Pedometer output - walk/run cadency in number of
                       ///< samples formatted as u6.2. e.g. At 50 Hz ODR and 2 Hz walk frequency,
                       ///< the cadency is 25 samples and register will output 100.
    APEX_DATA3 = 0x34, ///< APEX data 3 register. Contains DMP_IDLE bit and ACTIVITY_CLASS bits (00:
                       ///< unk, 01: walk, 10: run, 11: reserved)

    INTERFACE_CONFIG0 = 0x35, ///< Interface configuration 0 register
    INTERFACE_CONFIG1 = 0x36, ///< Interface configuration 1 register

    INT_STATUS_DATA_READY = 0x39, ///< Interrupt status data ready register
    INT_STATUS1 = 0x3A,           ///< Interrupt status 1 register
    INT_STATUS2 = 0x3B,           ///< Interrupt status 2 register
    INT_STATUS3 = 0x3C,           ///< Interrupt status 3 register

    FIFO_COUNTH = 0x3D, ///< FIFO count high register
    FIFO_COUNTL = 0x3E, ///< FIFO count low register

    FIFO_DATA = 0x3F, ///< FIFO data register

    WHO_AM_I = 0x75, ///< Who am I register

    BLK_SEL_W = 0x79, ///< Block select write register
    MADDR_W = 0x7A,   ///< Memory address write register
    M_WR_DATA = 0x7B, ///< Memory write data register

    BLK_SEL_R = 0x7C, ///< Block select read register
    MADDR_R = 0x7D,   ///< Memory address read register
    M_RD_DATA = 0x7E, ///< Memory read data register

    /// USER BANK MREG1

    // TODO: fill out from datasheet pp. 43-44

    /// USER BANK MREG2

    OTP_CTRL7 = 0x06, ///< OTP control 7 register

    /// USER BANK MREG3

    XA_ST_DATA = 0x00, ///< X-axis accelerometer self-test data register
    YA_ST_DATA = 0x01, ///< Y-axis accelerometer self-test data register
    ZA_ST_DATA = 0x02, ///< Z-axis accelerometer self-test data register
    XG_ST_DATA = 0x03, ///< X-axis gyroscope self-test data register
    YG_ST_DATA = 0x04, ///< Y-axis gyroscope self-test data register
    ZG_ST_DATA = 0x05, ///< Z-axis gyroscope self-test data register
  };

  static float accelerometer_range_to_sensitivty(AccelerometerRange range) {
    switch (range) {
    case AccelerometerRange::RANGE_16G:
      return ACCEL_FS_16G_SENS;
    case AccelerometerRange::RANGE_8G:
      return ACCEL_FS_8G_SENS;
    case AccelerometerRange::RANGE_4G:
      return ACCEL_FS_4G_SENS;
    case AccelerometerRange::RANGE_2G:
      return ACCEL_FS_2G_SENS;
    default:
      return 0.0f;
    }
  }

  static float gyroscope_range_to_sensitivty(GyroscopeRange range) {
    switch (range) {
    case GyroscopeRange::RANGE_2000DPS:
      return GYRO_FS_2000_SENS;
    case GyroscopeRange::RANGE_1000DPS:
      return GYRO_FS_1000_SENS;
    case GyroscopeRange::RANGE_500DPS:
      return GYRO_FS_500_SENS;
    case GyroscopeRange::RANGE_250DPS:
      return GYRO_FS_250_SENS;
    default:
      return 0.0f;
    }
  }

  uint16_t get_temperature_raw(std::error_code &ec) {
    return read_u16_from_register(static_cast<uint8_t>(Register::TEMP_DATA), ec);
  }

  RawValue get_accelerometer_raw(std::error_code &ec) { return get_raw(Register::ACCEL_DATA, ec); }

  RawValue get_gyroscope_raw(std::error_code &ec) { return get_raw(Register::GYRO_DATA, ec); }

  RawValue get_raw(Register reg, std::error_code &ec) {
    uint8_t data[6];
    read_many_from_register(static_cast<uint8_t>(reg), data, 6, ec);
    if (ec) {
      return {0, 0, 0};
    }
    return {
        static_cast<int16_t>((data[0] << 8) | data[1]),
        static_cast<int16_t>((data[2] << 8) | data[3]),
        static_cast<int16_t>((data[4] << 8) | data[5]),
    };
  }

  ImuConfig imu_config_; ///< IMU configuration
};                       // class Icm42607
} // namespace espp

// for libfmt printing of relevant imu types and structs
template <> struct fmt::formatter<espp::icm42607::AccelerometerRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::AccelerometerRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::icm42607::AccelerometerRange::RANGE_16G:
      return format_to(ctx.out(), "±16g");
    case espp::icm42607::AccelerometerRange::RANGE_8G:
      return format_to(ctx.out(), "±8g");
    case espp::icm42607::AccelerometerRange::RANGE_4G:
      return format_to(ctx.out(), "±4g");
    case espp::icm42607::AccelerometerRange::RANGE_2G:
      return format_to(ctx.out(), "±2g");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::AccelerometerPowerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::AccelerometerPowerMode power_mode, FormatContext &ctx) const {
    switch (power_mode) {
    case espp::icm42607::AccelerometerPowerMode::OFF:
      return format_to(ctx.out(), "Off");
    case espp::icm42607::AccelerometerPowerMode::ON:
      return format_to(ctx.out(), "On");
    case espp::icm42607::AccelerometerPowerMode::LOW_POWER:
      return format_to(ctx.out(), "Low power");
    case espp::icm42607::AccelerometerPowerMode::LOW_NOISE:
      return format_to(ctx.out(), "Low noise");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::AccelerometerODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::AccelerometerODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::icm42607::AccelerometerODR::ODR_1600_HZ:
      return format_to(ctx.out(), "1600 Hz");
    case espp::icm42607::AccelerometerODR::ODR_800_HZ:
      return format_to(ctx.out(), "800 Hz");
    case espp::icm42607::AccelerometerODR::ODR_400_HZ:
      return format_to(ctx.out(), "400 Hz");
    case espp::icm42607::AccelerometerODR::ODR_200_HZ:
      return format_to(ctx.out(), "200 Hz");
    case espp::icm42607::AccelerometerODR::ODR_100_HZ:
      return format_to(ctx.out(), "100 Hz");
    case espp::icm42607::AccelerometerODR::ODR_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm42607::AccelerometerODR::ODR_25_HZ:
      return format_to(ctx.out(), "25 Hz");
    case espp::icm42607::AccelerometerODR::ODR_12_5_HZ:
      return format_to(ctx.out(), "12.5 Hz");
    case espp::icm42607::AccelerometerODR::ODR_6_25_HZ:
      return format_to(ctx.out(), "6.25 Hz");
    case espp::icm42607::AccelerometerODR::ODR_3_125_HZ:
      return format_to(ctx.out(), "3.125 Hz");
    case espp::icm42607::AccelerometerODR::ODR_1_5625_HZ:
      return format_to(ctx.out(), "1.5625 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::GyroscopeRange> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::GyroscopeRange range, FormatContext &ctx) const {
    switch (range) {
    case espp::icm42607::GyroscopeRange::RANGE_2000DPS:
      return format_to(ctx.out(), "±2000°/s");
    case espp::icm42607::GyroscopeRange::RANGE_1000DPS:
      return format_to(ctx.out(), "±1000°/s");
    case espp::icm42607::GyroscopeRange::RANGE_500DPS:
      return format_to(ctx.out(), "±500°/s");
    case espp::icm42607::GyroscopeRange::RANGE_250DPS:
      return format_to(ctx.out(), "±250°/s");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::GyroscopePowerMode> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::GyroscopePowerMode power_mode, FormatContext &ctx) const {
    switch (power_mode) {
    case espp::icm42607::GyroscopePowerMode::OFF:
      return format_to(ctx.out(), "Off");
    case espp::icm42607::GyroscopePowerMode::STANDBY:
      return format_to(ctx.out(), "Standby");
    case espp::icm42607::GyroscopePowerMode::LOW_NOISE:
      return format_to(ctx.out(), "Low noise");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::GyroscopeODR> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(espp::icm42607::GyroscopeODR odr, FormatContext &ctx) const {
    switch (odr) {
    case espp::icm42607::GyroscopeODR::ODR_1600_HZ:
      return format_to(ctx.out(), "1600 Hz");
    case espp::icm42607::GyroscopeODR::ODR_800_HZ:
      return format_to(ctx.out(), "800 Hz");
    case espp::icm42607::GyroscopeODR::ODR_400_HZ:
      return format_to(ctx.out(), "400 Hz");
    case espp::icm42607::GyroscopeODR::ODR_200_HZ:
      return format_to(ctx.out(), "200 Hz");
    case espp::icm42607::GyroscopeODR::ODR_100_HZ:
      return format_to(ctx.out(), "100 Hz");
    case espp::icm42607::GyroscopeODR::ODR_50_HZ:
      return format_to(ctx.out(), "50 Hz");
    case espp::icm42607::GyroscopeODR::ODR_25_HZ:
      return format_to(ctx.out(), "25 Hz");
    case espp::icm42607::GyroscopeODR::ODR_12_5_HZ:
      return format_to(ctx.out(), "12.5 Hz");
    default:
      return format_to(ctx.out(), "Unknown");
    }
  }
};

template <> struct fmt::formatter<espp::icm42607::ImuConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm42607::ImuConfig &config, FormatContext &ctx) const {
    return format_to(ctx.out(),
                     "Accelerometer: {{ range: {}, odr: {} }}, Gyroscope: {{ range: {}, odr: {} }}",
                     config.accelerometer_range, config.accelerometer_odr, config.gyroscope_range,
                     config.gyroscope_odr);
  }
};

template <> struct fmt::formatter<espp::icm42607::RawValue> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm42607::RawValue &raw, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {}, y: {}, z: {} }}", raw.x, raw.y, raw.z);
  }
};

template <> struct fmt::formatter<espp::icm42607::Value> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm42607::Value &value, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ x: {:.2f}, y: {:.2f}, z: {:.2f} }}", value.x, value.y, value.z);
  }
};

template <> struct fmt::formatter<espp::icm42607::ComplimentaryAngle> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::icm42607::ComplimentaryAngle &angle, FormatContext &ctx) const {
    return format_to(ctx.out(), "{{ roll: {:.2f}, pitch: {:.2f} }}", angle.roll, angle.pitch);
  }
};

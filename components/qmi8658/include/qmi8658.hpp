#pragma once

#include "base_peripheral.hpp"

#include "qmi8658_detail.hpp"

namespace espp {
/// @brief Class for the QMI8658 6-axis motion sensor
/// @tparam Interface The interface type of the QMI8658
///
/// @note The QMI8658 is a 6-axis motion sensor that can be interfaced with
/// either I2C or SPI.
///
/// The QMI8658 has a 3-axis gyroscope and a 3-axis accelerometer. The
/// QMI8658 has a 16-bit ADC for the gyroscope and a 16-bit ADC for the
/// accelerometer. The gyroscope has a full-scale range of up to ±4096°/s and
/// the accelerometer has a full-scale range of up to ±16g.
///
/// The QMI8658 has a FIFO buffer that can store up to 2.25 kilobytes of
/// data. The FIFO buffer can be configured to store gyroscope data,
/// accelerometer data, or both. The FIFO buffer can be configured to store
/// data in either FIFO mode or stream mode. In FIFO mode, the FIFO buffer
/// stores data until the buffer is full, at which point the FIFO buffer stops
/// storing data. In stream mode, the FIFO buffer stores data continuously,
/// overwriting the oldest data when the buffer is full.
///
/// The QMI8658 has a built-in temperature sensor that can be used to measure
/// the temperature of the QMI8658.
///
/// \section qmi8658_example Example
/// \snippet qmi8658_example.cpp qmi8658 example
template <qmi8658::Interface Interface = qmi8658::Interface::I2C>
class Qmi8658 : public espp::BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C> {
  // Since the BasePeripheral is a dependent base class (e.g. its template
  // parameters depend on our template parameters), we need to use the `using`
  // keyword to bring in the functions / members we want to use, otherwise we
  // have to either use `this->` or explicitly scope each call, which clutters
  // the code / is annoying. This is needed because of the two phases of name
  // lookups for templates.
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::write_u8_to_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::write_many_to_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::read_u16_from_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::read_many_from_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::clear_bits_in_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::set_bits_in_register;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::set_bits_in_register_by_mask;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::base_mutex_;
  using BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      0x6B; ///< Default I2C address of the QMI8658 with AD0 high
  static constexpr uint8_t DEFAULT_ADDRESS_AD0_LOW =
      0x6A; ///< Default I2C address of the QMI8658 with AD0 low

  using FifoMode = qmi8658::FifoMode;                     ///< FIFO mode
  using AccelerometerRange = qmi8658::AccelerometerRange; ///< Accelerometer range
  using ODR = qmi8658::ODR;                               ///< Output data rate
  using GyroscopeRange = qmi8658::GyroscopeRange;         ///< Gyroscope range
  using ImuConfig = qmi8658::ImuConfig;                   ///< IMU configuration
  using RawValue = qmi8658::RawValue;                     ///< Raw IMU data
  using Value = qmi8658::Value;                           ///< IMU data
  using InterruptDriveMode = qmi8658::InterruptDriveMode; ///< Interrupt drive mode
  using InterruptPolarity = qmi8658::InterruptPolarity;   ///< Interrupt polarity
  using InterruptMode = qmi8658::InterruptMode;           ///< Interrupt mode
  using InterruptConfig = qmi8658::InterruptConfig;       ///< Interrupt configuration

  /// Filter function for filtering 6-axis data into 3-axis orientation data
  /// @param dt The time step in seconds
  /// @param accel The accelerometer data
  /// @param gyro The gyroscope data
  /// @return The filtered orientation data in radians
  typedef std::function<Value(float, const Value &, const Value &)> filter_fn; ///< Filter function

  /// Configuration struct for the QMI8658
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the QMI8658
    BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::write_fn write =
        nullptr; ///< Write function
    BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>::read_fn read =
        nullptr;                                          ///< Read function
    ImuConfig imu_config;                                 ///< IMU configuration
    filter_fn orientation_filter = nullptr;               ///< Filter function for orientation
    bool auto_init{true};                                 ///< Automatically initialize the QMI8658
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /// Constructor
  /// @param config The configuration
  explicit Qmi8658(const Config &config)
      : BasePeripheral<uint8_t, Interface == qmi8658::Interface::I2C>({}, "Qmi8658",
                                                                      config.log_level)
      , orientation_filter_(config.orientation_filter)
      , imu_config_(config.imu_config) {
    if constexpr (Interface == qmi8658::Interface::I2C) {
      set_address(config.device_address);
    }
    set_write(config.write);
    set_read(config.read);
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
    }
  }

  /// Initialize the QMI8658
  /// @param ec The error code to set if an error occurs
  /// @return True if the QMI8658 was initialized successfully, false otherwise
  bool init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    auto device_id = get_device_id(ec);
    if (device_id != WHO_AM_I_ID) {
      logger_.error("Invalid device ID: 0x{:02X}", device_id);
      return false;
    }

    write_u8_to_register(static_cast<uint8_t>(Register::CTRL1), 0x60, ec);
    if (ec) {
      logger_.error("Failed to set CTRL1 register: {}", ec.message());
      return false;
    }

    // set the configuration
    if (!set_config(imu_config_, ec)) {
      return false;
    }

    // enable the accelerometer and gyroscope
    if (!set_accelerometer_enabled(true, ec)) {
      return false;
    }
    if (!set_gyroscope_enabled(true, ec)) {
      return false;
    }

    return true;
  }

  /// Reset the device
  /// @param ec The error code to set if an error occurs
  /// @return True if the device was reset successfully, false otherwise
  bool reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    write_u8_to_register(static_cast<uint8_t>(Register::CTRL1), 0x80, ec);
    return !ec;
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
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // save the config
    imu_config_ = imu_config;

    // the config is set as two bytes containing the configuration for the
    // accelerometer and gyroscope (full scale range and output data rate)
    uint8_t accel_config = (static_cast<uint8_t>(imu_config.accelerometer_range) & 0x7) << 4 |
                           (static_cast<uint8_t>(imu_config.accelerometer_odr) & 0x0F);
    uint8_t gyro_config = (static_cast<uint8_t>(imu_config.gyroscope_range) & 0x7) << 4 |
                          (static_cast<uint8_t>(imu_config.gyroscope_odr) & 0x0F);

    write_u8_to_register(static_cast<uint8_t>(Register::CTRL2), accel_config, ec);
    if (ec) {
      logger_.error("Failed to set CTRL2 (Accel) register: {}", ec.message());
      return false;
    }
    write_u8_to_register(static_cast<uint8_t>(Register::CTRL3), gyro_config, ec);
    if (ec) {
      logger_.error("Failed to set CTRL3 (Gyro) register: {}", ec.message());
      return false;
    }

    logger_.info("IMU configuration set: Accel Range: {}, Gyro Range: {}",
                 imu_config.accelerometer_range, imu_config.gyroscope_range);
    logger_.info("IMU configuration set: Accel ODR: {}, Gyro ODR: {}", imu_config.accelerometer_odr,
                 imu_config.gyroscope_odr);
    return true;
  }

  /// Get the IMU configuration
  /// @return The IMU configuration
  ImuConfig get_config() const { return imu_config_; }

  /// Set the Accelerometer enabled state
  /// @param enabled True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if the enabled state was set successfully, false otherwise
  bool set_accelerometer_enabled(bool enabled, std::error_code &ec) {
    uint8_t bitmask = 0x01;
    if (enabled) {
      set_bits_in_register(static_cast<uint8_t>(Register::CTRL7), bitmask, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::CTRL7), bitmask, ec);
    }
    if (ec) {
      logger_.error("Failed to set accelerometer enabled state: {}", ec.message());
      return false;
    }
    logger_.info("Accelerometer enabled state set to {}", enabled ? "true" : "false");
    return true;
  }

  /// Set the Gyroscope enabled state
  /// @param enabled True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if the enabled state was set successfully, false otherwise
  bool set_gyroscope_enabled(bool enabled, std::error_code &ec) {
    uint8_t bitmask = 0x02;
    if (enabled) {
      set_bits_in_register(static_cast<uint8_t>(Register::CTRL7), bitmask, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::CTRL7), bitmask, ec);
    }
    if (ec) {
      logger_.error("Failed to set gyroscope enabled state: {}", ec.message());
      return false;
    }
    logger_.info("Gyroscope enabled state set to {}", enabled ? "true" : "false");
    return true;
  }

  /// Get the accelerometer sensitivity from memory
  /// @return The accelerometer sensitivity in g/LSB
  float get_accelerometer_sensitivity() {
    return accelerometer_range_to_sensitivty(imu_config_.accelerometer_range);
  }

  /// Read the accelerometer sensitivity
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer sensitivity in g/LSB
  float read_accelerometer_sensitivity(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read the byte from the register
    uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::CTRL2), ec);
    if (ec) {
      return 0.0f;
    }
    // get the range
    AccelerometerRange range = static_cast<AccelerometerRange>((data >> 4) & 0x07);
    // update the config
    imu_config_.accelerometer_range = range;
    // convert to sensitivity
    return accelerometer_range_to_sensitivty(range);
  }

  /// Get the gyroscope sensitivity from memory
  /// @return The gyroscope sensitivity in °/s/LSB
  float get_gyroscope_sensitivity() {
    return gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
  }

  /// Read the gyroscope sensitivity
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope sensitivity in °/s/LSB
  float read_gyroscope_sensitivity(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // read the byte from the register
    uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::CTRL3), ec);
    if (ec) {
      return 0.0f;
    }
    // get the range
    GyroscopeRange range = static_cast<GyroscopeRange>((data >> 4) & 0x07);
    // update the config
    imu_config_.gyroscope_range = range;
    // convert to sensitivity
    return gyroscope_range_to_sensitivty(range);
  }

  /// Update the accelerometer and gyroscope, and read the temperature
  /// @param dt The time step in seconds
  /// @param ec The error code to set if an error occurs
  /// @return True if the values were updated successfully, false otherwise
  /// @note The values can be retrieved with get_accelerometer_values and
  ///       get_gyroscope_values, and the temperature can be retrieved with
  ///       get_temperature
  bool update(float dt, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // update accel
    Value accel = read_accelerometer(ec);
    if (ec) {
      return false;
    }
    // update gyro
    Value gyro = read_gyroscope(ec);
    if (ec) {
      return false;
    }
    // update temp
    read_temperature(ec);
    if (ec) {
      return false;
    }
    // if we have a filter function, then filter the data and update the
    // orientation
    if (orientation_filter_) {
      orientation_ = orientation_filter_(dt, accel, gyro);
      // now calculate the gravity vector
      gravity_vector_ = {
          (float)(sin(orientation_.pitch)),
          (float)(-sin(orientation_.roll) * cos(orientation_.pitch)),
          (float)(-cos(orientation_.roll) * cos(orientation_.pitch)),
      };
    }
    return true;
  }

  /// Get the accelerometer values
  /// @return The accelerometer values
  /// @note The values are in g and are updated when read_accelerometer is
  ///       called or update_values is called
  Value get_accelerometer() const { return accel_values_; }

  /// Read the accelerometer data
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer data
  Value read_accelerometer(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    RawValue raw = get_accelerometer_raw(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    float sensitivity = get_accelerometer_sensitivity();
    Value v = {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
    // update accel values
    accel_values_ = v;
    return v;
  }

  /// Get the gyroscope values
  /// @return The gyroscope values
  /// @note The values are in °/s and are updated when read_gyroscope is
  ///       called or update_values is called
  Value get_gyroscope() const { return gyro_values_; }

  /// Read the gyroscope data
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope data
  Value read_gyroscope(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    RawValue raw = get_gyroscope_raw(ec);
    if (ec) {
      return {0.0f, 0.0f, 0.0f};
    }
    float sensitivity = get_gyroscope_sensitivity();
    Value v = {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
    // update gyro values
    gyro_values_ = v;
    return v;
  }

  /// Get the temperature
  /// @return The temperature in °C
  /// @note The temperature is updated when read_temperature is called or
  ///       update_values is called
  float get_temperature() const { return temperature_; }

  /// Read the temperature
  /// @param ec The error code to set if an error occurs
  /// @return The temperature in °C
  float read_temperature(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    int16_t raw = get_temperature_raw(ec);
    if (ec) {
      return 0.0f;
    }
    // temp_in_c = raw / 256
    float temp = static_cast<float>(raw) / 256.0f;
    // update temperature
    temperature_ = temp;
    return temp;
  }

  /// Get the most recently calculated orientation, if any.
  /// @return The orientation angles in radians - pitch, roll, yaw
  /// @note The orientation is updated when update_values is called
  /// @note The orientation is calculated using the orientation filter function.
  ///       If no filter function is set, the orientation will not be updated.
  Value get_orientation() const { return orientation_; }

  /// Get the most recently calculated gravity vector, if any.
  /// @return The gravity vector
  /// @note The gravity vector is updated when update_values is called
  /// @note The gravity vector is calculated using the orientation filter
  ///       function. If no filter function is set, the gravity vector will not
  ///       be calculated.
  Value get_gravity_vector() const { return gravity_vector_; }

  /// Configure interrupt 1
  /// @param config The interrupt configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the interrupt was configured successfully, false otherwise
  bool configure_interrupt_1(const InterruptConfig &config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // interrupt 1 is in CTRL6
    uint8_t mask = 0b11; // bits 0 and 1
    uint8_t data =
        (static_cast<uint8_t>(config.polarity) << 1) | (static_cast<uint8_t>(config.drive_mode));
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::CTRL6), mask, data, ec);
    return !ec;
  }

  /// Configure interrupt 2
  /// @param config The interrupt configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the interrupt was configured successfully, false otherwise
  bool configure_interrupt_2(const InterruptConfig &config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    // interrupt 2 is in CTRL6
    uint8_t mask = 0b11 << 2; // bits 2 and 3
    uint8_t data =
        ((static_cast<uint8_t>(config.polarity) << 1) | (static_cast<uint8_t>(config.drive_mode)))
        << 2;
    set_bits_in_register_by_mask(static_cast<uint8_t>(Register::CTRL6), mask, data, ec);
    return !ec;
  }

  /// Read whether the data is ready
  /// @param ec The error code to set if an error occurs
  /// @return True if the data is ready, false otherwise
  bool is_data_ready(std::error_code &ec) {
    uint8_t data_ready = read_u8_from_register(static_cast<uint8_t>(Register::STATUS0), ec);
    return !ec && (data_ready & 0x01);
  }

protected:
  static constexpr float RAD_TO_DEG = 57.27272727f;   ///< Radians to degrees
  static constexpr float DEG_TO_RAD = 0.01745329252f; ///< Degrees to radians

  static constexpr uint8_t WHO_AM_I_ID = 0x05; ///< QMI8658 ID

  /// Register addresses
  enum class Register : uint8_t {
    WHO_AM_I = 0x00,
    REVISION = 0x01,
    CTRL1 = 0x02,
    CTRL2 = 0x03,
    CTRL3 = 0x04,
    CTRL4 = 0x05,
    CTRL5 = 0x06,
    CTRL6 = 0x07,
    CTRL7 = 0x08,
    CTRL8 = 0x09,
    CTRL9 = 0x0A,

    STATUS0 = 0x2E,
    STATUS1 = 0x2F,

    TIMESTAMP_L = 0x30,
    TIMESTAMP_M = 0x31,
    TIMESTAMP_H = 0x32,

    TEMP_L = 0x33,
    TEMP_H = 0x34,

    ACC_X_L = 0x35,
    ACC_X_H = 0x36,
    ACC_Y_L = 0x37,
    ACC_Y_H = 0x38,
    ACC_Z_L = 0x39,
    ACC_Z_H = 0x3A,
    GYR_X_L = 0x3B,
    GYR_X_H = 0x3C,
    GYR_Y_L = 0x3D,
    GYR_Y_H = 0x3E,
    GYR_Z_L = 0x3F,
    GYR_Z_H = 0x40,

    FIFO_CTRL = 0x4C,
    FIFO_DATA = 0x4E,
    FIFO_STATUS = 0x4D,
  };

  static float accelerometer_range_to_sensitivty(const AccelerometerRange &range) {
    switch (range) {
    case AccelerometerRange::RANGE_16G:
      return 2048.0f;
    case AccelerometerRange::RANGE_8G:
      return 4096.0f;
    case AccelerometerRange::RANGE_4G:
      return 8192.0f;
    case AccelerometerRange::RANGE_2G:
      return 16384.0f;
    default:
      return 0.0f;
    }
  }

  static float gyroscope_range_to_sensitivty(const GyroscopeRange &range) {
    switch (range) {
    case GyroscopeRange::RANGE_4096_DPS:
      return 8.0f;
    case GyroscopeRange::RANGE_2048_DPS:
      return 16.0f;
    case GyroscopeRange::RANGE_1024_DPS:
      return 32.0f;
    case GyroscopeRange::RANGE_512_DPS:
      return 64.0f;
    case GyroscopeRange::RANGE_256_DPS:
      return 128.0f;
    case GyroscopeRange::RANGE_128_DPS:
      return 256.0f;
    case GyroscopeRange::RANGE_64_DPS:
      return 512.0f;
    case GyroscopeRange::RANGE_32_DPS:
      return 1024.0f;
    default:
      return 0.0f;
    }
  }

  int16_t get_temperature_raw(std::error_code &ec) {
    uint8_t data[2];
    read_many_from_register(static_cast<uint8_t>(Register::TEMP_L), data, 2, ec);
    if (ec) {
      return 0;
    }
    return (data[1] << 8) | data[0];
  }

  RawValue get_accelerometer_raw(std::error_code &ec) { return get_raw(Register::ACC_X_L, ec); }

  RawValue get_gyroscope_raw(std::error_code &ec) { return get_raw(Register::GYR_X_L, ec); }

  RawValue get_raw(Register reg, std::error_code &ec) {
    uint8_t data[6];
    read_many_from_register(static_cast<uint8_t>(reg), data, 6, ec);
    if (ec) {
      return {0, 0, 0};
    }
    return {
        static_cast<int16_t>((data[1] << 8) | data[0]),
        static_cast<int16_t>((data[3] << 8) | data[2]),
        static_cast<int16_t>((data[5] << 8) | data[4]),
    };
  }

  Value accel_values_{};
  Value gyro_values_{};
  float temperature_{0.0f};
  Value orientation_{};
  Value gravity_vector_{};
  filter_fn orientation_filter_{nullptr};
  ImuConfig imu_config_{}; ///< IMU configuration
};                         // class Qmi8658
} // namespace espp

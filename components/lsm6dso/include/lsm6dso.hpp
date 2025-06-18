#pragma once

#include "base_peripheral.hpp"
#include "lsm6dso_detail.hpp"

namespace espp {
/// @brief Class for the LSM6DSO 6-axis IMU (Accel + Gyro)
/// @tparam Interface The interface type (I2C or SPI)
///
/// The LSM6DSO is a 6-axis IMU with a 3-axis accelerometer and 3-axis gyroscope.
/// It supports both I2C and SPI, and on-chip filtering.
///
/// For more information, see the LSM6DSO datasheet:
/// https://www.st.com/resource/en/datasheet/lsm6dso.pdf
///
/// \section lsm6dso_example Example
/// \snippet lsm6dso_example.cpp lsm6dso example
///
/// Usage:
///   - Construct with a Config struct
///   - Call init() to initialize
///   - Use update() to read and filter data
///   - Use get_accelerometer(), get_gyroscope(), get_temperature() to access data
///   - Use get_orientation(), get_gravity_vector() for orientation and gravity vector if filter is
///   set
///
/// \note This class is intended for use with the LSM6DSO and compatible variants.
template <lsm6dso::Interface Interface = lsm6dso::Interface::I2C>
class Lsm6dso : public BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C> {
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::write_u8_to_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::write_many_to_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::read_many_from_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::clear_bits_in_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::set_bits_in_register;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::set_bits_in_register_by_mask;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::base_mutex_;
  using BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x6A; ///< Default I2C address (can be 0x6B)

  using AccelRange = lsm6dso::AccelRange;   ///< Alias for accelerometer range
  using AccelODR = lsm6dso::AccelODR;       ///< Alias for accelerometer output data rate (ODR)
  using AccelFilter = lsm6dso::AccelFilter; ///< Alias for accelerometer filter type
  using GyroRange = lsm6dso::GyroRange;     ///< Alias for gyroscope range
  using GyroODR = lsm6dso::GyroODR;         ///< Alias for gyroscope output data rate (ODR)
  using GyroHPF = lsm6dso::GyroHPF;         ///< Alias for gyroscope high-pass filter (HPF)
  using RawValue = lsm6dso::RawValue;       ///< Alias for raw sensor values
  using Value = lsm6dso::Value;             ///< Alias for processed sensor values
  using ImuConfig = lsm6dso::ImuConfig;     ///< Alias for IMU configuration

  /// Filter function for orientation filtering
  /// @param dt The time step in seconds
  /// @param accel The accelerometer data
  /// @param gyro The gyroscope data
  /// @return The filtered orientation data in radians
  typedef std::function<Value(float, const Value &, const Value &)> filter_fn;

  /// Configuration struct for the LSM6DSO
  struct Config {
    uint8_t device_address = DEFAULT_I2C_ADDRESS; ///< I2C address (if I2C)
    BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::write_fn write =
        nullptr; ///< Write function
    BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>::read_fn read =
        nullptr;                            ///< Read function
    ImuConfig imu_config;                   ///< IMU configuration
    filter_fn orientation_filter = nullptr; ///< Algorithmic orientation filter
    bool auto_init{true};                   ///< Automatically initialize on construction
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /// Constructor
  /// @param config The configuration
  explicit Lsm6dso(const Config &config);

  /// Initialize the LSM6DSO
  /// @param ec The error code to set if an error occurs
  /// @return True if initialized successfully, false otherwise
  bool init(std::error_code &ec);

  /// Get the device ID
  /// @param ec The error code to set if an error occurs
  /// @return The device ID
  uint8_t get_device_id(std::error_code &ec);

  /// Set the IMU configuration
  /// @param config The configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the configuration was set successfully, false otherwise
  bool set_config(const ImuConfig &config, std::error_code &ec);

  /// Get the IMU configuration
  /// @return The configuration
  ImuConfig get_config() const;

  /// Set the accelerometer output data rate (ODR)
  /// @param odr The output data rate
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_accelerometer_range(AccelRange range, std::error_code &ec);

  /// Set the gyroscope output data rate (ODR)
  /// @param odr The output data rate
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_gyroscope_range(GyroRange range, std::error_code &ec);

  /// Set the accelerometer output data rate (ODR)
  /// @param odr The output data rate
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_accelerometer_odr(AccelODR odr, std::error_code &ec);

  /// Set the gyroscope output data rate (ODR)
  /// @param odr The output data rate
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_gyroscope_odr(GyroODR odr, std::error_code &ec);

  /// Get the accelerometer sensitivity in g/LSB
  /// @return The accelerometer sensitivity
  float get_accelerometer_sensitivity() const {
    return accelerometer_range_to_sensitivity(imu_config_.accel_range);
  }

  /// Get the gyroscope sensitivity in dps/LSB
  /// @return The gyroscope sensitivity
  float get_gyroscope_sensitivity() const {
    return gyroscope_range_to_sensitivity(imu_config_.gyro_range);
  }

  /// Read the accelerometer sensitivity in g/LSB
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer sensitivity
  float read_accelerometer_sensitivity(std::error_code &ec);

  /// Read the gyroscope sensitivity in dps/LSB
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope sensitivity
  float read_gyroscope_sensitivity(std::error_code &ec);

  /// Set the accelerometer power mode
  /// @param enable True to enable, false to power down
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_accelerometer_power_mode(bool enable, std::error_code &ec);

  /// Set the gyroscope power mode
  /// @param enable True to enable, false to power down
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_gyroscope_power_mode(bool enable, std::error_code &ec);

  /// Set the accelerometer filter bandwidth
  /// @param bw The filter bandwidth
  /// @param accel_filter The accelerometer filter type
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_accelerometer_filter(uint8_t bw, AccelFilter accel_filter, std::error_code &ec);

  /// Set the gyroscope filter bandwidth
  /// @param lpf1_bw The low-pass filter 1 bandwidth. Bandwidth differs
  ///        depending on ODR setting.
  /// @param hpf_enabled True to enable high-pass filter, false to disable
  /// @param hpf_bw The high-pass filter bandwidth
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool set_gyroscope_filter(uint8_t lpf1_bw, bool hpf_enabled, GyroHPF hpf_bw, std::error_code &ec);

  /// Update the sensor values (read accel, gyro, temp, run filter)
  /// @param dt The time step in seconds
  /// @param ec The error code to set if an error occurs
  /// @return True if successful
  bool update(float dt, std::error_code &ec);

  /// Get the accelerometer values (g)
  /// @return The accelerometer values
  Value get_accelerometer() const;

  /// Read the accelerometer values (g)
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer values
  Value read_accelerometer(std::error_code &ec);

  /// Get the gyroscope values (dps)
  /// @return The gyroscope values
  Value get_gyroscope() const;

  /// Read the gyroscope values (dps)
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope values
  Value read_gyroscope(std::error_code &ec);

  /// Get the temperature (deg C)
  /// @return The temperature
  float get_temperature() const;

  /// Read the temperature (deg C)
  /// @param ec The error code to set if an error occurs
  /// @return The temperature
  float read_temperature(std::error_code &ec);

  /// Get the orientation values (radians)
  /// @return The orientation values
  Value get_orientation() const;

  /// Get the gravity vector (in Gs)
  /// @return The gravity vector
  Value get_gravity_vector() const;

protected:
  static constexpr float accelerometer_range_to_sensitivity(const AccelRange &range) {
    switch (range) {
    case AccelRange::RANGE_2G:
      return 0.061f / 1000.0f; // 2000mg / 32768
    case AccelRange::RANGE_4G:
      return 0.122f / 1000.0f; // 4000mg / 32768
    case AccelRange::RANGE_8G:
      return 0.244f / 1000.0f; // 8000mg / 32768
    case AccelRange::RANGE_16G:
      return 0.488f / 1000.0f; // 16000mg / 32768
    default:
      return 0.061f / 1000.0f; // Default to RANGE_2G
    }
  }

  static constexpr float gyroscope_range_to_sensitivity(const GyroRange &range) {
    switch (range) {
    case GyroRange::DPS_125:
      return 4.375f / 1000.0f; // 125dps / 32768
    case GyroRange::DPS_250:
      return 8.75f / 1000.0f; // 250dps / 32768
    case GyroRange::DPS_500:
      return 17.50f / 1000.0f; // 500dps / 32768
    case GyroRange::DPS_1000:
      return 35.00f / 1000.0f; // 1000dps / 32768
    case GyroRange::DPS_2000:
      return 70.00f / 1000.0f; // 2000dps / 32768
    default:
      return 8.75f / 1000.0f; // Default to DPS_250
    }
  }

  RawValue read_accelerometer_raw(std::error_code &ec) {
    return read_raw(lsm6dso::Register::OUTX_L_A, ec);
  }
  RawValue read_gyroscope_raw(std::error_code &ec) {
    return read_raw(lsm6dso::Register::OUTX_L_G, ec);
  }
  RawValue read_raw(lsm6dso::Register reg, std::error_code &ec);

  Value accel_values_{};
  Value gyro_values_{};
  float temperature_{0.0f};
  Value orientation_{};
  Value gravity_vector_{};
  filter_fn orientation_filter_{nullptr};
  ImuConfig imu_config_{};
};
} // namespace espp

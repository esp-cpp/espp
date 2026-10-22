#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <system_error>

#include "base_peripheral.hpp"

namespace espp {

/// @brief Driver for the Bosch BNO055 9-axis absolute orientation sensor over I2C.
///
/// The BNO055 combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis
/// magnetometer with an on-chip Cortex-M0 that runs Bosch's sensor fusion and
/// outputs orientation (Euler angles and a quaternion), linear acceleration,
/// and gravity.
///
/// This driver configures the sensor into one of its operating modes and reads
/// all sensor and fusion outputs in a single 46-byte burst. Units are set to
/// SI: m/s^2 for acceleration, rad/s for angular rate, rad for Euler angles,
/// and uT for the magnetic field.
///
/// The default mode is IMU (accelerometer + gyroscope fusion, heading relative
/// to power-on). NDOF adds the magnetometer for an absolute heading, but is
/// sensitive to nearby motors and ferrous metal.
///
/// \note The BNO055 stretches the I2C clock; keep the I2C timeout at or above
///       the espp default (10 ms).
///
/// Datasheet:
/// - https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
///
/// \section bno055_example Example
/// \snippet bno055_example.cpp bno055 example
class Bno055 : public BasePeripheral<uint8_t> {
public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x28;   ///< Address with ADR (COM3) pulled low.
  static constexpr uint8_t ALTERNATE_ADDRESS = 0x29; ///< Address with ADR (COM3) pulled high.
  static constexpr uint8_t CHIP_ID = 0xA0;           ///< Expected value of the CHIP_ID register.

  /// @brief Sensor operating mode (OPR_MODE register).
  enum class OperatingMode : uint8_t {
    CONFIG = 0x00,       ///< Configuration mode; no outputs are updated.
    ACC_ONLY = 0x01,     ///< Accelerometer only.
    ACC_GYRO = 0x05,     ///< Accelerometer and gyroscope, no fusion.
    AMG = 0x07,          ///< All three sensors, no fusion.
    IMU = 0x08,          ///< Accel + gyro fusion; heading is relative to power-on.
    COMPASS = 0x09,      ///< Accel + mag fusion.
    M4G = 0x0A,          ///< Accel + mag fusion, mag used in place of gyro.
    NDOF_FMC_OFF = 0x0B, ///< Full 9-DoF fusion without fast mag calibration.
    NDOF = 0x0C,         ///< Full 9-DoF fusion; heading is absolute (magnetic north).
  };

  /// @brief Calibration status of each sensor, 0 (uncalibrated) to 3 (fully calibrated).
  struct CalibrationStatus {
    uint8_t system{0}; ///< Overall fusion calibration.
    uint8_t gyro{0};   ///< Gyroscope calibration.
    uint8_t accel{0};  ///< Accelerometer calibration.
    uint8_t mag{0};    ///< Magnetometer calibration.
  };

  /// @brief Three-axis vector.
  struct Vector3 {
    float x{0.0f}; ///< X component.
    float y{0.0f}; ///< Y component.
    float z{0.0f}; ///< Z component.
  };

  /// @brief Unit quaternion.
  struct Quaternion {
    float w{1.0f}; ///< Scalar component.
    float x{0.0f}; ///< X component.
    float y{0.0f}; ///< Y component.
    float z{0.0f}; ///< Z component.
  };

  /// @brief Euler angles, in radians.
  struct Euler {
    float heading{0.0f}; ///< Yaw, 0 to 2*pi.
    float roll{0.0f};    ///< Roll, -pi/2 to pi/2.
    float pitch{0.0f};   ///< Pitch, -pi to pi.
  };

  /// @brief One complete set of sensor and fusion outputs.
  struct Data {
    Vector3 accel_mps2;            ///< Raw acceleration including gravity.
    Vector3 mag_ut;                ///< Magnetic field, in microtesla.
    Vector3 gyro_radps;            ///< Angular rate.
    Euler euler;                   ///< Fused orientation as Euler angles.
    Quaternion orientation;        ///< Fused orientation as a quaternion.
    Vector3 linear_accel_mps2;     ///< Acceleration with gravity removed.
    Vector3 gravity_mps2;          ///< Gravity vector.
    int8_t temperature_c{0};       ///< Die temperature, in degrees Celsius.
    CalibrationStatus calibration; ///< Per-sensor calibration status.
  };

  /// @brief Driver configuration.
  struct Config {
    uint8_t device_address{DEFAULT_ADDRESS};                     ///< I2C address of the sensor.
    BasePeripheral::probe_fn probe{nullptr};                     ///< Probe the I2C address.
    BasePeripheral::write_fn write{nullptr};                     ///< Write bytes to the sensor.
    BasePeripheral::write_then_read_fn write_then_read{nullptr}; ///< Write then read.
    OperatingMode mode{OperatingMode::IMU};                      ///< Fusion mode to run in.
    bool use_external_crystal{true}; ///< Use the board's 32 kHz crystal.
    bool auto_init{true};            ///< Call init() on construction.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity.
  };

  /// @brief Construct the driver.
  /// @param config Bus access functions and sensor settings.
  explicit Bno055(const Config &config);

  /// @brief Reset the sensor, verify its chip ID, set SI units, and enter the configured mode.
  /// @param ec Set on any bus error or if the chip ID does not match.
  /// @return True if the sensor is running in the configured mode.
  /// @note Blocks for roughly 700 ms while the sensor resets.
  bool init(std::error_code &ec);

  /// @brief Read all sensor and fusion outputs in one burst and cache them.
  /// @param ec Set on a bus error.
  /// @return True if the read succeeded.
  bool update(std::error_code &ec);

  /// @brief Get the most recent outputs read by update().
  /// @return A copy of the cached data.
  Data data() const;

  /// @brief Check whether init() completed successfully.
  /// @return True if the sensor is initialized.
  bool initialized() const { return initialized_; }

  /// @brief Read the SYS_STATUS and SYS_ERR registers.
  /// @param status Set to SYS_STATUS (5 = fusion running, 1 = system error).
  /// @param error Set to SYS_ERR (0 = no error).
  /// @param ec Set on a bus error.
  void read_system_status(uint8_t &status, uint8_t &error, std::error_code &ec);

protected:
  /// Register addresses on page 0 (datasheet section 4.2).
  enum class Register : uint8_t {
    CHIP_ID = 0x00,        ///< Chip ID, reads 0xA0.
    PAGE_ID = 0x07,        ///< Register page select.
    ACC_DATA_X_LSB = 0x08, ///< Start of the 46-byte data block through CALIB_STAT.
    CALIB_STAT = 0x35,     ///< Calibration status of each sensor.
    ST_RESULT = 0x36,      ///< Power-on self test result.
    SYS_STATUS = 0x39,     ///< System status.
    SYS_ERR = 0x3A,        ///< System error code.
    UNIT_SEL = 0x3B,       ///< Output unit selection.
    OPR_MODE = 0x3D,       ///< Operating mode.
    PWR_MODE = 0x3E,       ///< Power mode.
    SYS_TRIGGER = 0x3F,    ///< Reset, self test, and clock source.
  };

  /// Number of bytes from ACC_DATA_X_LSB up to and including CALIB_STAT.
  static constexpr size_t DATA_BLOCK_SIZE = 0x35 - 0x08 + 1;

  /// Write OPR_MODE and wait the datasheet's mode switching time.
  void set_mode(OperatingMode mode, std::error_code &ec);
  /// Poll CHIP_ID until it reads 0xA0, for up to about 1 s.
  bool wait_for_chip_id(std::error_code &ec);
  /// Convert the raw data block into SI units.
  static Data parse(const std::array<uint8_t, DATA_BLOCK_SIZE> &raw);

  OperatingMode mode_;            ///< Mode entered at the end of init().
  bool use_external_crystal_;     ///< Whether init() selects the external crystal.
  bool initialized_{false};       ///< Set when init() succeeds.
  mutable std::mutex data_mutex_; ///< Protects data_.
  Data data_{};                   ///< Latest outputs read by update().
};

} // namespace espp

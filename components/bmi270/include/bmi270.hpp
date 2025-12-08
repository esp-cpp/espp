#pragma once

#include "base_peripheral.hpp"
#include "bmi270_detail.hpp"

namespace espp {
/// @brief Class for the BMI270 6-axis motion sensor
/// @tparam Interface The interface type of the BMI270 (I2C or SPI)
///
/// @note The BMI270 is a highly integrated, low power inertial measurement unit (IMU)
/// that combines precise acceleration and angular rate measurement with intelligent
/// on-chip motion-triggered interrupt features.
///
/// Key features from the datasheet:
/// - 16-bit digital, triaxial accelerometer with ±2g/±4g/±8g/±16g range
/// - 16-bit digital, triaxial gyroscope with ±125°/s to ±2000°/s range
/// - Output data rates: 0.78Hz to 1.6kHz (accelerometer), 25Hz to 6.4kHz (gyroscope)
/// - Low current consumption: typ. 685µA in full operation
/// - Built-in power management unit (PMU) for advanced power management
/// - 2KB on-chip FIFO buffer
/// - Advanced features: step counter, activity recognition, wrist gestures, etc.
/// - Temperature sensor with 0.5°C resolution
///
/// For more information, see the datasheet:
/// https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi270-ds000.pdf
///
/// \section bmi270_example Example
/// \snippet bmi270_example.cpp bmi270 example
template <bmi270::Interface Interface = bmi270::Interface::I2C>
class Bmi270 : public espp::BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C> {
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::write_u8_to_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::write_many_to_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::read_u16_from_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::read_many_from_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::clear_bits_in_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::set_bits_in_register;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::set_bits_in_register_by_mask;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::base_mutex_;
  using BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS = 0x68;          ///< Default I2C address with SDO low
  static constexpr uint8_t DEFAULT_ADDRESS_SDO_HIGH = 0x69; ///< I2C address with SDO high

  // Type aliases for easier access
  using AccelerometerRange = bmi270::AccelerometerRange;
  using AccelerometerODR = bmi270::AccelerometerODR;
  using AccelerometerBandwidth = bmi270::AccelerometerBandwidth;
  using GyroscopeRange = bmi270::GyroscopeRange;
  using GyroscopeODR = bmi270::GyroscopeODR;
  using GyroscopeBandwidth = bmi270::GyroscopeBandwidth;
  using GyroscopePerformanceMode = bmi270::GyroscopePerformanceMode;
  using FifoMode = bmi270::FifoMode;
  using FeatureConfig = bmi270::FeatureConfig;
  using InterruptPin = bmi270::InterruptPin;
  using InterruptOutput = bmi270::InterruptOutput;
  using InterruptLevel = bmi270::InterruptLevel;
  using ImuConfig = bmi270::ImuConfig;
  using RawValue = bmi270::RawValue;
  using Value = bmi270::Value;
  using MotionData = bmi270::MotionData;
  using StepData = bmi270::StepData;
  using WristData = bmi270::WristData;
  using InterruptConfig = bmi270::InterruptConfig;
  using FifoConfig = bmi270::FifoConfig;

  /// Filter function for filtering 6-axis data into 3-axis orientation data
  /// @param dt The time step in seconds
  /// @param accel The accelerometer data
  /// @param gyro The gyroscope data
  /// @return The filtered orientation data in radians
  typedef std::function<Value(float, const Value &, const Value &)> filter_fn;

  /// Configuration struct for the BMI270
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the BMI270
    BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::write_fn write =
        nullptr; ///< Write function
    BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>::read_fn read =
        nullptr;                                          ///< Read function
    ImuConfig imu_config;                                 ///< IMU configuration
    filter_fn orientation_filter = nullptr;               ///< Filter function for orientation
    /**
     * @brief Maximum number of bytes to write in a single burst during config upload. If 0 will use full config file size of 8192 bytes.
     * Default is 0 (uses full config file size of 8192 bytes).
     * Set this to a small non-zero value (e.g., 128) if you encounter stack overflow or want to decrease memory usage.
     */
    uint16_t burst_write_size = 0;
    bool auto_init{true};                                 ///< Automatically initialize the BMI270
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /// Constructor
  /// @param config The configuration
  explicit Bmi270(const Config &config)
      : BasePeripheral<uint8_t, Interface == bmi270::Interface::I2C>({}, "Bmi270", config.log_level)
      , orientation_filter_(config.orientation_filter)
      , imu_config_(config.imu_config)
      , burst_write_size_(config.burst_write_size == 0 ? config_file_size : config.burst_write_size) {
    if constexpr (Interface == bmi270::Interface::I2C) {
      set_address(config.device_address);
    }
    set_write(config.write);
    set_read(config.read);
    if (config.auto_init) {
      std::error_code ec;
      init(ec);
    }
  }

  /// Initialize the BMI270
  /// @param ec The error code to set if an error occurs
  /// @return True if the BMI270 was initialized successfully, false otherwise
  bool init(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Check device ID
    auto device_id = get_device_id(ec);
    if (ec || device_id != BMI270_CHIP_ID) {
      logger_.error("Invalid device ID: 0x{:02X}, expected 0x{:02X}", device_id, BMI270_CHIP_ID);
      return false;
    }

    // Perform soft reset
    if (!soft_reset(ec)) {
      return false;
    }

    // Set power mode to normal
    if (!set_low_power_mode(false, ec)) {
      return false;
    }

    // Wait for device to be ready (datasheet recommends 450µs after reset)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Load configuration file (BMI270 requires config file to be loaded)
    if (!load_config_file(ec)) {
      return false;
    }

    // Configure accelerometer and gyroscope
    if (!set_config(imu_config_, ec)) {
      return false;
    }

    // Enable accelerometer and gyroscope
    if (!enable_accelerometer(true, ec)) {
      return false;
    }

    if (!enable_gyroscope(true, ec)) {
      return false;
    }

    // Enable temperature sensor
    if (!enable_temperature_sensor(true, ec)) {
      return false;
    }

    logger_.info("BMI270 initialized successfully");
    return true;
  }

  /// Get the device ID
  /// @param ec The error code to set if an error occurs
  /// @return The device ID
  uint8_t get_device_id(std::error_code &ec) {
    return read_u8_from_register(static_cast<uint8_t>(Register::CHIP_ID), ec);
  }

  /// Perform soft reset
  /// @param ec The error code to set if an error occurs
  /// @return True if reset was successful, false otherwise
  bool soft_reset(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    write_u8_to_register(static_cast<uint8_t>(Register::CMD), SOFT_RESET_CMD, ec);
    if (ec) {
      return false;
    }
    // Wait for reset to complete
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    return true;
  }

  /// Set whether to enable or disable low power mode
  /// @param enabled True to enable low power mode, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if power mode was set successfully, false otherwise
  bool set_low_power_mode(bool enabled, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    logger_.info("Setting low power mode to {}", enabled ? "enabled" : "disabled");
    static constexpr uint8_t ADV_POWER_SAVE = 0x01; // Advanced power save bit
    if (enabled) {
      set_bits_in_register(static_cast<uint8_t>(Register::PWR_CONF), ADV_POWER_SAVE, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::PWR_CONF), ADV_POWER_SAVE, ec);
    }
    if (ec) {
      logger_.error("Failed to set low power mode: {}", ec.message());
      return false;
    }
    return true;
  }

  /// Enable/disable temperature sensor
  /// @param enable True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if successful, false otherwise
  bool enable_temperature_sensor(bool enable, std::error_code &ec) {
    if (enable) {
      set_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), TMP_ENABLE, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), TMP_ENABLE, ec);
    }
    return !ec;
  }

  /// Enable/disable accelerometer
  /// @param enable True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if successful, false otherwise
  bool enable_accelerometer(bool enable, std::error_code &ec) {
    if (enable) {
      set_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), ACC_ENABLE, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), ACC_ENABLE, ec);
    }
    return !ec;
  }

  /// Enable/disable gyroscope
  /// @param enable True to enable, false to disable
  /// @param ec The error code to set if an error occurs
  /// @return True if successful, false otherwise
  bool enable_gyroscope(bool enable, std::error_code &ec) {
    if (enable) {
      set_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), GYR_ENABLE, ec);
    } else {
      clear_bits_in_register(static_cast<uint8_t>(Register::PWR_CTRL), GYR_ENABLE, ec);
    }
    return !ec;
  }

  /// Set the IMU configuration
  /// @param imu_config The IMU configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the configuration was set successfully, false otherwise
  bool set_config(const ImuConfig &imu_config, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    imu_config_ = imu_config;

    static constexpr uint8_t filter_performance_optimized =
        0x80; ///< Performance optimized filter (instead of power)

    // Configure accelerometer
    uint8_t acc_conf = filter_performance_optimized | // Enable performance optimized filter
                       (static_cast<uint8_t>(imu_config.accelerometer_bandwidth) << 4) |
                       static_cast<uint8_t>(imu_config.accelerometer_odr);
    write_u8_to_register(static_cast<uint8_t>(Register::ACC_CONF), acc_conf, ec);
    if (ec)
      return false;

    uint8_t acc_range = static_cast<uint8_t>(imu_config.accelerometer_range);
    write_u8_to_register(static_cast<uint8_t>(Register::ACC_RANGE), acc_range, ec);
    if (ec)
      return false;

    // Configure gyroscope
    static constexpr uint8_t gyr_noise_performance_optimized =
        0x40; ///< noise is optimized for performance (not power)
    uint8_t gyr_conf = filter_performance_optimized |    // Enable performance optimized filter
                       gyr_noise_performance_optimized | // Optimize noise for performance
                       (static_cast<uint8_t>(imu_config.gyroscope_bandwidth) << 4) |
                       static_cast<uint8_t>(imu_config.gyroscope_odr);
    write_u8_to_register(static_cast<uint8_t>(Register::GYR_CONF), gyr_conf, ec);
    if (ec)
      return false;

    uint8_t gyr_range = static_cast<uint8_t>(imu_config.gyroscope_range);
    write_u8_to_register(static_cast<uint8_t>(Register::GYR_RANGE), gyr_range, ec);
    return !ec;
  }

  /// Get the IMU configuration
  /// @return The IMU configuration
  ImuConfig get_config() const { return imu_config_; }

  /// Check if new data is available
  /// @param ec The error code to set if an error occurs
  /// @return True if new data is available, false otherwise
  bool has_new_data(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint8_t status = read_u8_from_register(static_cast<uint8_t>(Register::STATUS), ec);
    if (ec)
      return false;
    has_new_data_ = (status & (DRDY_ACC | DRDY_GYR)) != 0;
    return has_new_data_;
  }

  /// Check if new data is available (cached version)
  /// @return True if new data is available, false otherwise
  bool has_new_data() const { return has_new_data_; }

  /// Update all sensor data
  /// @param dt The time step in seconds
  /// @param ec The error code to set if an error occurs
  /// @return True if data was updated successfully, false otherwise
  bool update(float dt, std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // Read accelerometer data
    Value accel = read_accelerometer(ec);
    if (ec)
      return false;

    // Read gyroscope data
    Value gyro = read_gyroscope(ec);
    if (ec)
      return false;

    // Read temperature
    read_temperature(ec);
    if (ec)
      return false;

    // Apply orientation filter if available
    if (orientation_filter_) {
      orientation_ = orientation_filter_(dt, accel, gyro);
      // Calculate gravity vector from orientation
      gravity_vector_ = {
          static_cast<float>(sinf(orientation_.pitch)),
          static_cast<float>(-sinf(orientation_.roll) * cosf(orientation_.pitch)),
          static_cast<float>(-cosf(orientation_.roll) * cosf(orientation_.pitch)),
      };
    }

    return true;
  }

  /// Get accelerometer data
  /// @return The accelerometer data in g
  Value get_accelerometer() const { return accel_values_; }

  /// Read accelerometer data
  /// @param ec The error code to set if an error occurs
  /// @return The accelerometer data in g
  Value read_accelerometer(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    RawValue raw = get_accelerometer_raw(ec);
    if (ec)
      return {0.0f, 0.0f, 0.0f};

    float sensitivity = get_accelerometer_sensitivity();
    Value v = {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
    accel_values_ = v;
    return v;
  }

  /// Get gyroscope data
  /// @return The gyroscope data in °/s
  Value get_gyroscope() const { return gyro_values_; }

  /// Read gyroscope data
  /// @param ec The error code to set if an error occurs
  /// @return The gyroscope data in °/s
  Value read_gyroscope(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    RawValue raw = get_gyroscope_raw(ec);
    if (ec)
      return {0.0f, 0.0f, 0.0f};

    float sensitivity = get_gyroscope_sensitivity();
    Value v = {
        static_cast<float>(raw.x) / sensitivity,
        static_cast<float>(raw.y) / sensitivity,
        static_cast<float>(raw.z) / sensitivity,
    };
    gyro_values_ = v;
    return v;
  }

  /// Get temperature
  /// @return The temperature in °C
  float get_temperature() const { return temperature_; }

  /// Read temperature
  /// @param ec The error code to set if an error occurs
  /// @return The temperature in °C
  float read_temperature(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);
    uint16_t raw = get_temperature_raw(ec);
    if (ec)
      return 0.0f;

    // Temperature calculation from datasheet: 23°C + (temp_raw / 512)
    float temp = 23.0f + (static_cast<float>(static_cast<int16_t>(raw)) / 512.0f);
    temperature_ = temp;
    return temp;
  }

  /// Get orientation data
  /// @return The orientation data in radians
  Value get_orientation() const { return orientation_; }

  /// Get gravity vector
  /// @return The gravity vector
  Value get_gravity_vector() const { return gravity_vector_; }

  /// Get accelerometer sensitivity
  /// @return The accelerometer sensitivity in LSB/g
  float get_accelerometer_sensitivity() const {
    return accelerometer_range_to_sensitivity(imu_config_.accelerometer_range);
  }

  /// Get gyroscope sensitivity
  /// @return The gyroscope sensitivity in LSB/(°/s)
  float get_gyroscope_sensitivity() const {
    return gyroscope_range_to_sensitivity(imu_config_.gyroscope_range);
  }

  /// Configure interrupts
  /// @param config The interrupt configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if interrupts were configured successfully, false otherwise
  bool configure_interrupts(const InterruptConfig &config, std::error_code &ec) {
    // Configure interrupt pin electrical behavior (Output Enable | Output Type | Active Level)
    // Bit 3 is Output Enable, which must be set to 1 for the pin to drive the line.
    uint8_t int_io_ctrl = (1 << 3) | 
                          (static_cast<uint8_t>(config.output_type) << 2) |
                          (static_cast<uint8_t>(config.active_level) << 1);

    auto pin_reg = (config.pin == InterruptPin::INT1) ? Register::INT1_IO_CTRL : Register::INT2_IO_CTRL;
    write_u8_to_register(static_cast<uint8_t>(pin_reg), int_io_ctrl, ec);
    if (ec) return false;

    // Map interrupt features to pins
    uint8_t int_map_data = 0;
    if (config.enable_data_ready) {
      // Map Data Ready interrupt to the selected pin
      // Bit 2: Data Ready -> INT1, Bit 6: Data Ready -> INT2
      int_map_data |= (config.pin == InterruptPin::INT1) ? (1 << 2) : (1 << 6);
    }

    write_u8_to_register(static_cast<uint8_t>(Register::INT_MAP_DATA), int_map_data, ec);
    if (ec) return false;

    return true;
  }

protected:
  // BMI270 specific constants
  static constexpr uint8_t BMI270_CHIP_ID = 0x24; ///< BMI270 chip ID
  static constexpr uint8_t FIFO_FLUSH_CMD = 0xB0; ///< FIFO flush command
  static constexpr uint8_t SOFT_RESET_CMD = 0xB6; ///< Soft reset command
  static constexpr uint8_t TMP_ENABLE = 0x08;     ///< Temperature sensor enable bit
  static constexpr uint8_t ACC_ENABLE = 0x04;     ///< Accelerometer enable bit
  static constexpr uint8_t GYR_ENABLE = 0x02;     ///< Gyroscope enable bit
  static constexpr uint8_t AUX_ENABLE = 0x01;     ///< Auxiliary sensor enable bit
  static constexpr uint8_t DRDY_ACC = 0x80;       ///< Accelerometer data ready bit
  static constexpr uint8_t DRDY_GYR = 0x40;       ///< Gyroscope data ready bit

  // Sensitivity constants (LSB/unit)
  static constexpr float ACCEL_2G_SENS = 16384.0f; ///< ±2g sensitivity
  static constexpr float ACCEL_4G_SENS = 8192.0f;  ///< ±4g sensitivity
  static constexpr float ACCEL_8G_SENS = 4096.0f;  ///< ±8g sensitivity
  static constexpr float ACCEL_16G_SENS = 2048.0f; ///< ±16g sensitivity

  static constexpr float GYRO_2000DPS_SENS = 16.4f; ///< ±2000°/s sensitivity
  static constexpr float GYRO_1000DPS_SENS = 32.8f; ///< ±1000°/s sensitivity
  static constexpr float GYRO_500DPS_SENS = 65.5f;  ///< ±500°/s sensitivity
  static constexpr float GYRO_250DPS_SENS = 131.0f; ///< ±250°/s sensitivity
  static constexpr float GYRO_125DPS_SENS = 262.0f; ///< ±125°/s sensitivity

  /// @brief Default configuration file for the BMI270
  static const uint8_t config_file[];
  static const size_t config_file_size;

  /// Register addresses based on BMI270 datasheet
  enum class Register : uint8_t {
    CHIP_ID = 0x00,            ///< Chip ID register
    ERR_REG = 0x02,            ///< Error register
    STATUS = 0x03,             ///< Status register
    DATA_0 = 0x04,             ///< Data register 0 (AUX_X_LSB)
    DATA_1 = 0x05,             ///< Data register 1 (AUX_X_MSB)
    DATA_2 = 0x06,             ///< Data register 2 (AUX_Y_LSB)
    DATA_3 = 0x07,             ///< Data register 3 (AUX_Y_MSB)
    DATA_4 = 0x08,             ///< Data register 4 (AUX_Z_LSB)
    DATA_5 = 0x09,             ///< Data register 5 (AUX_Z_MSB)
    DATA_6 = 0x0A,             ///< Data register 6 (AUX_R_LSB)
    DATA_7 = 0x0B,             ///< Data register 7 (AUX_R_MSB)
    ACC_DATA_START = 0x0C,     ///< Start of accelerometer data registers
    DATA_8 = 0x0C,             ///< Data register 8 (ACC_X_LSB)
    DATA_9 = 0x0D,             ///< Data register 9 (ACC_X_MSB)
    DATA_10 = 0x0E,            ///< Data register 10 (ACC_Y_LSB)
    DATA_11 = 0x0F,            ///< Data register 11 (ACC_Y_MSB)
    DATA_12 = 0x10,            ///< Data register 12 (ACC_Z_LSB)
    DATA_13 = 0x11,            ///< Data register 13 (ACC_Z_MSB)
    GYR_DATA_START = 0x12,     ///< Start of gyroscope data registers
    DATA_14 = 0x12,            ///< Data register 14 (GYR_X_LSB)
    DATA_15 = 0x13,            ///< Data register 15 (GYR_X_MSB)
    DATA_16 = 0x14,            ///< Data register 16 (GYR_Y_LSB)
    DATA_17 = 0x15,            ///< Data register 17 (GYR_Y_MSB)
    DATA_18 = 0x16,            ///< Data register 18 (GYR_Z_LSB)
    DATA_19 = 0x17,            ///< Data register 19 (GYR_Z_MSB)
    SENSORTIME_0 = 0x18,       ///< Sensor time 0
    SENSORTIME_1 = 0x19,       ///< Sensor time 1
    SENSORTIME_2 = 0x1A,       ///< Sensor time 2
    EVENT = 0x1B,              ///< Event register
    INT_STATUS_0 = 0x1C,       ///< Interrupt status 0
    INT_STATUS_1 = 0x1D,       ///< Interrupt status 1
    SC_OUT_0 = 0x1E,           ///< Step counter output 0
    SC_OUT_1 = 0x1F,           ///< Step counter output 1
    WR_GEST_ACT = 0x20,        ///< Wrist gesture and activity recognition output
    INTERNAL_STATUS = 0x21,    ///< Internal status
    TEMP_LSB = 0x22,           ///< Temperature LSB
    TEMP_MSB = 0x23,           ///< Temperature MSB
    FIFO_LENGTH_0 = 0x24,      ///< FIFO length LSB
    FIFO_LENGTH_1 = 0x25,      ///< FIFO length MSB
    FIFO_DATA = 0x26,          ///< FIFO data register
    FEAT_PAGE = 0x2F,          ///< Feature page register
    FEATURES_0 = 0x30,         ///< Features register 0
    ACC_CONF = 0x40,           ///< Accelerometer configuration
    ACC_RANGE = 0x41,          ///< Accelerometer range
    GYR_CONF = 0x42,           ///< Gyroscope configuration
    GYR_RANGE = 0x43,          ///< Gyroscope range
    AUX_CONF = 0x44,           ///< Auxiliary sensor configuration
    FIFO_DOWNS = 0x45,         ///< FIFO downsampling
    FIFO_WTM_0 = 0x46,         ///< FIFO watermark LSB
    FIFO_WTM_1 = 0x47,         ///< FIFO watermark MSB
    FIFO_CONFIG_0 = 0x48,      ///< FIFO configuration 0
    FIFO_CONFIG_1 = 0x49,      ///< FIFO configuration 1
    SATURATION = 0x4A,         ///< Saturation register
    INT1_IO_CTRL = 0x53,       ///< INT1 I/O control
    INT2_IO_CTRL = 0x54,       ///< INT2 I/O control
    INT_LATCH = 0x55,          ///< Interrupt latch configuration
    INT1_MAP_FEAT = 0x56,      ///< INT1 feature mapping
    INT2_MAP_FEAT = 0x57,      ///< INT2 feature mapping
    INT_MAP_DATA = 0x58,       ///< Interrupt data mapping
    INIT_CTRL = 0x59,          ///< Initialization control
    INIT_ADDR_0 = 0x5B,        ///< Initialization address LSB
    INIT_ADDR_1 = 0x5C,        ///< Initialization address MSB
    INIT_DATA = 0x5E,          ///< Initialization data
    ACC_SELF_TEST = 0x6D,      ///< Accelerometer self-test
    GYR_SELF_TEST_AXES = 0x6E, ///< Gyroscope self-test axes
    NV_CONF = 0x6A,            ///< Non-volatile memory configuration
    IF_CONF = 0x6B,            ///< Interface configuration
    DRV = 0x6C,                ///< Drive register
    ACC_OFF_COMP_0 = 0x71,     ///< Accelerometer offset compensation 0
    GYR_OFF_COMP_3 = 0x74,     ///< Gyroscope offset compensation 3
    GYR_OFF_COMP_6 = 0x77,     ///< Gyroscope offset compensation 6
    GYR_USR_GAIN_0 = 0x78,     ///< Gyroscope user gain 0
    PWR_CONF = 0x7C,           ///< Power configuration
    PWR_CTRL = 0x7D,           ///< Power control
    CMD = 0x7E,                ///< Command register
  };

  /// Load configuration file (BMI270 requires this for advanced features)
  /// @param ec The error code to set if an error occurs
  /// @return True if config file was loaded successfully, false otherwise
  /// @note This function MUST NOT be called more than once after POR or soft reset.
  bool load_config_file(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(base_mutex_);

    // BMI270 requires a configuration file to be loaded for advanced features
    logger_.info("Loading BMI270 configuration file ({} B), this may take some time...",
                 config_file_size);

    // write INIT_CTRL.init_ctrl = 0x00 to prepare config load
    clear_bits_in_register(static_cast<uint8_t>(Register::INIT_CTRL), 0x01, ec);
    if (ec) {
      logger_.error("Failed to clear INIT_CTRL: {}", ec.message());
      return false;
    }

    // upload config file:
    // - burst write init data to INIT_DATA, using configurable chunk size (defaults to 8 kB)
    const uint8_t *config_data = config_file;
    size_t burst_size = burst_write_size_;
    size_t config_size = config_file_size;
    size_t offset = 0;
    while (offset < config_size) {
      size_t chunk_size = std::min<size_t>(burst_size, config_size - offset);
      // first we have to write the addr of the init data (using the 16bit offset)
      uint16_t init_addr = static_cast<uint16_t>(offset);
      uint8_t addr_data[2] = {
          static_cast<uint8_t>((init_addr / 2) & 0x0F), // LSB
          static_cast<uint8_t>(((init_addr / 2) >> 4))  // MSB
      };
      write_many_to_register(static_cast<uint8_t>(Register::INIT_ADDR_0), addr_data, 2, ec);
      if (ec) {
        logger_.error("Failed to write INIT_ADDR: {}", ec.message());
        return false;
      }
      // then we write the chunk
      write_many_to_register(static_cast<uint8_t>(Register::INIT_DATA), config_data + offset,
                             chunk_size, ec);
      if (ec) {
        logger_.error("Failed to write configuration data at offset {}: {}", offset, ec.message());
        return false;
      }
      offset += chunk_size;
    }

    // write INIT_CTRL.init_ctrl = 0x01 to complete config load
    set_bits_in_register(static_cast<uint8_t>(Register::INIT_CTRL), 0x01, ec);
    if (ec) {
      logger_.error("Failed to set INIT_CTRL: {}", ec.message());
      return false;
    }

    // wait until INTERNAL_STATUS.message contains the value 0b0001, which will
    // happen after at most 20ms.
    uint8_t internal_status = 0;
    for (int i = 0; i < 20; ++i) {
      internal_status = read_u8_from_register(static_cast<uint8_t>(Register::INTERNAL_STATUS), ec);
      if (ec) {
        logger_.error("Failed to read internal status: {}", ec.message());
        return false;
      }
      if ((internal_status & 0x01) != 0) {
        break; // Configuration loaded successfully
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    if ((internal_status & 0x01) == 0) {
      logger_.error("Configuration file load timed out");
      return false;
    }
    logger_.info("Configuration file loaded successfully");
    return true;
  }

  /// Get raw accelerometer data
  /// @param ec The error code to set if an error occurs
  /// @return Raw accelerometer data
  RawValue get_accelerometer_raw(std::error_code &ec) {
    return get_raw_data(Register::ACC_DATA_START, ec);
  }

  /// Get raw gyroscope data
  /// @param ec The error code to set if an error occurs
  /// @return Raw gyroscope data
  RawValue get_gyroscope_raw(std::error_code &ec) {
    return get_raw_data(Register::GYR_DATA_START, ec);
  }

  /// Get raw temperature data
  /// @param ec The error code to set if an error occurs
  /// @return Raw temperature data
  uint16_t get_temperature_raw(std::error_code &ec) {
    uint8_t data[2];
    read_many_from_register(static_cast<uint8_t>(Register::TEMP_LSB), data, 2, ec);
    if (ec)
      return 0;
    return static_cast<uint16_t>((data[1] << 8) | data[0]);
  }

  /// Get raw data from register
  /// @param reg The starting register
  /// @param ec The error code to set if an error occurs
  /// @return Raw data
  RawValue get_raw_data(Register reg, std::error_code &ec) {
    uint8_t data[6];
    read_many_from_register(static_cast<uint8_t>(reg), data, 6, ec);
    if (ec)
      return {0, 0, 0};

    return {
        static_cast<int16_t>((data[1] << 8) | data[0]), // X
        static_cast<int16_t>((data[3] << 8) | data[2]), // Y
        static_cast<int16_t>((data[5] << 8) | data[4]), // Z
    };
  }

  /// Convert accelerometer range to sensitivity
  /// @param range The accelerometer range
  /// @return The sensitivity in LSB/g
  static float accelerometer_range_to_sensitivity(const AccelerometerRange &range) {
    switch (range) {
    case AccelerometerRange::RANGE_2G:
      return ACCEL_2G_SENS;
    case AccelerometerRange::RANGE_4G:
      return ACCEL_4G_SENS;
    case AccelerometerRange::RANGE_8G:
      return ACCEL_8G_SENS;
    case AccelerometerRange::RANGE_16G:
      return ACCEL_16G_SENS;
    default:
      return ACCEL_4G_SENS;
    }
  }

  /// Convert gyroscope range to sensitivity
  /// @param range The gyroscope range
  /// @return The sensitivity in LSB/(°/s)
  static float gyroscope_range_to_sensitivity(const GyroscopeRange &range) {
    switch (range) {
    case GyroscopeRange::RANGE_2000DPS:
      return GYRO_2000DPS_SENS;
    case GyroscopeRange::RANGE_1000DPS:
      return GYRO_1000DPS_SENS;
    case GyroscopeRange::RANGE_500DPS:
      return GYRO_500DPS_SENS;
    case GyroscopeRange::RANGE_250DPS:
      return GYRO_250DPS_SENS;
    case GyroscopeRange::RANGE_125DPS:
      return GYRO_125DPS_SENS;
    default:
      return GYRO_2000DPS_SENS;
    }
  }

  // Member variables
  filter_fn orientation_filter_{nullptr};
  ImuConfig imu_config_{};
  uint16_t burst_write_size_{0};
  Value accel_values_{};
  Value gyro_values_{};
  float temperature_{0.0f};
  Value orientation_{};
  Value gravity_vector_{};
  bool has_new_data_{false};
  MotionData motion_data_{};
  StepData step_data_{};
  WristData wrist_data_{};

}; // class Bmi270
} // namespace espp

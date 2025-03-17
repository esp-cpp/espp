#pragma once

#pragma once

#include "base_peripheral.hpp"

#include "icm20948_detail.hpp"

namespace espp {
/// @brief Class for the ICM20948 9-axis motion sensor
/// @tparam Interface The interface type of the ICM20948
///
/// The ICM-20948 IMU is a 9-axis motion sensor that combines a 3-axis
/// gyroscope, 3-axis accelerometer, and 3-axis magnetometer. The ICM-20948 is a
/// 16-bit sensor that can measure acceleration up to ±16g, angular velocity up
/// to ±2000°/s, and magnetic field up to ±4900 µT. The magnetic field sensor
/// used is the AK09916C.
///
/// Stats:
/// * Accelerometer range: ±2g, ±4g, ±8g, ±16g
/// * Accelerometer sensitivity: 16384, 8192, 4096, 2048 LSB/g
/// * Accelerometer output data rate: 4.5 Hz - 4.5 kHz
/// * Gyroscope range: ±250°/s, ±500°/s, ±1000°/s, ±2000°/s
/// * Gyroscope sensitivity: 131, 65.5, 32.8, 16.4 LSB/(°/s)
/// * Gyroscope output data rate: 4.4 - 9 kHz
/// * Magnetometer range: ±4900 µT
/// * Magnetometer sensitivity: 0.15 µT/LSB
/// * Magnetometer output data rate: 10 Hz - 100 Hz
///
/// Using the ADA and ACL lines, the ICM-20948 can be configured to control /
/// communicate with auxiliary I2C sensors.
///
/// The ICM-20948 has a FIFO buffer that can store up to 4 kB of data. The FIFO
/// buffer can be configured to store gyroscope, accelerometer, and temperature
/// data. The FIFO buffer can be used to store data for batch processing or to
/// reduce the number of interrupts generated by the ICM-20948.
///
/// The ICM-20948 has a Digital Motion Processor (DMP) that can be used to
/// offload sensor fusion calculations from the host processor. The DMP can be
/// used to calculate quaternion, rotation matrix, and Euler angles. The DMP can
/// also be used to detect gestures, such as tap, shake, and orientation.
///
/// The ICM-20948 has a wake-on-motion (WOM) feature that can be used to wake the
/// host processor when motion is detected. The WOM feature can be used to reduce
/// power consumption by putting the host processor to sleep when motion is not
/// detected.
///
/// The ICM-20948 has a pedometer feature that can be used to count the number of
/// steps taken by the user. The pedometer feature can be used to track the
/// number of steps taken by the user and to estimate the distance traveled by
/// the user.
///
/// You can find the datasheets for the ICM-20948 here:
/// - https://invensense.tdk.com/wp-content/uploads/2016/06/DS-000189-ICM-20948-v1.3.pdf
/// - https://www.y-ic.es/datasheet/78/SMDSW.020-2OZ.pdf
///
/// For migrating from MPU-9250 to ICM-20948, you can refer to the following
/// document:
/// -
/// https://invensense.tdk.com/wp-content/uploads/2018/10/AN-000146-v2.0-TDK_Migration_MPU_9250toICM-20948.pdf
///
/// Thanks and credits to the following sources:
/// - https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i
/// - https://github.com/wollewald/ICM20948_WE/blob/main/src/ICM20948_WE.h
///
/// \section icm20948_example Example
/// \snippet icm20948_example.cpp icm20948 example
template <icm20948::Interface Interface = icm20948::Interface::I2C>
class Icm20948 : public espp::BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C> {
  // Since the BasePeripheral is a dependent base class (e.g. its template
  // parameters depend on our template parameters), we need to use the `using`
  // keyword to bring in the functions / members we want to use, otherwise we
  // have to either use `this->` or explicitly scope each call, which clutters
  // the code / is annoying. This is needed because of the two phases of name
  // lookups for templates.
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::set_address;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::set_write;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::set_read;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::write_u8_to_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::write_u16_to_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::write_many_to_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::read_u8_from_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::read_u16_from_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::read_many_from_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::clear_bits_in_register;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::set_bits_in_register;
  using BasePeripheral<uint8_t,
                       Interface == icm20948::Interface::I2C>::set_bits_in_register_by_mask;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::read;
  using BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::logger_;

public:
  static constexpr uint8_t DEFAULT_ADDRESS =
      0x68; ///< Default I2C address of the ICM20948 with AD0 low
  static constexpr uint8_t DEFAULT_ADDRESS_AD0_HIGH =
      0x69; ///< Default I2C address of the ICM20948 with AD0 high

  static constexpr uint8_t AK09916C_ADDRESS = 0x0C; ///< I2C address of the AK09916C magnetometer

  using PowerMode = icm20948::PowerMode;                   ///< Power mode
  using DutyCycleMode = icm20948::DutyCycleMode;           ///< Duty cycle mode
  using FifoMode = icm20948::FifoMode;                     ///< FIFO mode
  using FifoType = icm20948::FifoType;                     ///< FIFO type
  using AccelerometerRange = icm20948::AccelerometerRange; ///< Accelerometer range
  using GyroscopeRange = icm20948::GyroscopeRange;         ///< Gyroscope range
  using DmpODR = icm20948::DmpODR;                         ///< DMP output data rate
  using TemperatureFilterBandwidth =
      icm20948::TemperatureFilterBandwidth;            ///< Temperature filter bandwidth
  using MagnetometerMode = icm20948::MagnetometerMode; ///< Magnetometer mode
  using AccelerometerFilterBandwidth =
      icm20948::AccelerometerFilterBandwidth; ///< Sensor filter bandwidth for the accelerometer
  using GyroscopeFilterBandwidth =
      icm20948::GyroscopeFilterBandwidth; ///< Sensor filter bandwidth for the gyroscope
  using AccelerometerAveraging = icm20948::AccelerometerAveraging; ///< Accelerometer averaging
  using GyroscopeAveraging = icm20948::GyroscopeAveraging;         ///< Gyroscope averaging
  using ImuConfig = icm20948::ImuConfig;                           ///< IMU configuration
  using RawValue = icm20948::RawValue;                             ///< Raw IMU data
  using Value = icm20948::Value;                                   ///< IMU data
  using InterruptDriveMode = icm20948::InterruptDriveMode;         ///< Interrupt drive mode
  using InterruptPolarity = icm20948::InterruptPolarity;           ///< Interrupt polarity
  using InterruptMode = icm20948::InterruptMode;                   ///< Interrupt mode
  using InterruptConfig = icm20948::InterruptConfig;               ///< Interrupt configuration

  /// Filter function for filtering 9-axis data into 3-axis orientation data
  /// @param dt The time step in seconds
  /// @param accel The accelerometer data
  /// @param gyro The gyroscope data
  /// @param mag The magnetometer data
  /// @return The filtered orientation data in radians
  typedef std::function<Value(float, const Value &, const Value &, const Value &)>
      filter_fn; ///< Filter function

  /// Range struct
  struct Range {
    float min; ///< Minimum value
    float max; ///< Maximum value
  };

  /// Configuration struct for the ICM20948
  struct Config {
    uint8_t device_address = DEFAULT_ADDRESS; ///< I2C address of the ICM20948
    BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::write_fn write =
        nullptr; ///< Write function
    BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>::read_fn read =
        nullptr;                                          ///< Read function
    ImuConfig imu_config;                                 ///< IMU configuration
    filter_fn orientation_filter = nullptr;               ///< Orientation filter function
    bool auto_init{true};                                 ///< Automatically initialize the ICM20948
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; ///< Log level
  };

  /// Constructor
  /// @param config The configuration
  explicit Icm20948(const Config &config);

  /// Initialize the ICM20948
  /// @param ec The error code to set if an error occurs
  /// @return True if the ICM20948 was initialized successfully, false otherwise
  bool init(std::error_code &ec);

  /// Get the device ID
  /// @param ec The error code to set if an error occurs
  /// @return The device ID
  uint8_t get_device_id(std::error_code &ec);

  /// Set the IMU configuration
  /// @param imu_config The IMU configuration
  /// @param ec The error code to set if an error occurs
  /// @return True if the configuration was set successfully, false otherwise
  bool set_config(const ImuConfig &imu_config, std::error_code &ec);

  /// Set whether the I2C master is enabled
  /// @param enable True to enable the I2C master, false to disable it
  /// @param ec The error code to set if an error occurs
  /// @return True if the I2C master was enabled or disabled successfully, false otherwise
  bool set_i2c_master_enabled(bool enable, std::error_code &ec);

  /////////////////////////////////
  // Configuration / Offsets
  /////////////////////////////////
  bool auto_offsets(std::error_code &ec);
  bool set_odr_align_enabled(bool enable, std::error_code &ec);
  bool set_accelerometer_offsets(const Range &x, const Range &y, const Range &z,
                                 std::error_code &ec);
  bool get_accelerometer_offsets(Range &x, Range &y, Range &z, std::error_code &ec);
  bool set_gyroscope_offsets(const float &x, const float &y, const float &z, std::error_code &ec);
  bool get_gyroscope_offsets(float &x, float &y, float &z, std::error_code &ec);

  /////////////////////////////////
  // Power / Sleep / Standby
  /////////////////////////////////
  bool set_power_mode(const PowerMode &mode, std::error_code &ec);
  bool set_low_power_enabled(bool enable, std::error_code &ec);
  bool set_low_power_duty_cycle_mode(const DutyCycleMode &mode, std::error_code &ec);
  bool set_gyroscope_average_in_low_power_mode(const GyroscopeAveraging &average,
                                               std::error_code &ec);
  bool set_accelerometer_average_in_low_power_mode(const AccelerometerAveraging &average,
                                                   std::error_code &ec);
  bool sleep(bool enable, std::error_code &ec);
  bool reset(std::error_code &ec);

  /////////////////////////////////
  // Accelerometer
  /////////////////////////////////
  bool enable_accelerometer(bool enable, std::error_code &ec);
  float get_accelerometer_sensitivity();
  float read_accelerometer_sensitivity(std::error_code &ec);
  AccelerometerRange get_accelerometer_range();
  AccelerometerRange read_accelerometer_range(std::error_code &ec);
  bool set_accelerometer_range(const AccelerometerRange &range, std::error_code &ec);
  bool set_accelerometer_dlpf_enabled(bool enable, std::error_code &ec);
  bool set_accelerometer_dlpf(const AccelerometerFilterBandwidth &bandwidth, std::error_code &ec);
  bool set_accelerometer_sample_rate_divider(uint16_t sample_rate_divider, std::error_code &ec);

  /////////////////////////////////
  // Gyroscope
  /////////////////////////////////
  bool enable_gyroscope(bool enable, std::error_code &ec);
  float get_gyroscope_sensitivity();
  float read_gyroscope_sensitivity(std::error_code &ec);
  GyroscopeRange get_gyroscope_range();
  GyroscopeRange read_gyroscope_range(std::error_code &ec);
  bool set_gyroscope_range(const GyroscopeRange &range, std::error_code &ec);
  bool set_gyroscope_dlpf_enabled(bool enable, std::error_code &ec);
  bool set_gyroscope_dlpf(const GyroscopeFilterBandwidth &bandwidth, std::error_code &ec);
  bool set_gyroscope_sample_rate_divider(uint8_t sample_rate_divider, std::error_code &ec);

  /////////////////////////////////
  // Temperature
  /////////////////////////////////
  bool set_temperature_dlpf(const TemperatureFilterBandwidth &bandwidth, std::error_code &ec);

  /////////////////////////////////
  // Magnetometer
  /////////////////////////////////
  bool init_magnetometer(std::error_code &ec);
  uint16_t get_magnetometer_device_id(std::error_code &ec);
  bool set_magnetometer_mode(const icm20948::MagnetometerMode &mode, std::error_code &ec);
  float get_magnetometer_sensitivity();
  bool reset_magnetometer(std::error_code &ec);

  /////////////////////////////////
  // Raw / Low level data
  /////////////////////////////////
  Value get_accelerometer();
  Value get_gyroscope();
  Value get_magnetometer();
  float get_temperature();
  Value read_accelerometer(std::error_code &ec);
  Value read_gyroscope(std::error_code &ec);
  Value read_magnetometer(std::error_code &ec);
  float read_temperature(std::error_code &ec);

  /////////////////////////////////
  // DMP
  /////////////////////////////////

  bool enable_dmp(bool enable, std::error_code &ec);
  bool reset_dmp(std::error_code &ec);

  /////////////////////////////////
  // Angles and Orientation
  /////////////////////////////////

  /// Update the accelerometer, gyroscope, temperature, and magnetometer values
  /// @param dt The time step in seconds
  /// @param ec The error code to set if an error occurs
  /// @return True if the values were updated successfully, false otherwise
  /// @note The values can be retrieved with get_accelerometer_values and
  ///       get_gyroscope_values, and the temperature can be retrieved with
  ///       get_temperature. The orientation can be retrieved with
  ///       get_orientation.
  bool update(float dt, std::error_code &ec);
  Value get_orientation();
  Value get_gravity_vector();
  Value get_angles();
  float get_pitch();
  float get_roll();
  float get_yaw();

  /////////////////////////////////
  // FIFO
  /////////////////////////////////
  bool enable_fifo(bool enable, std::error_code &ec);
  bool set_fifo_mode(const FifoMode &mode, std::error_code &ec);
  bool start_fifo(const FifoType &fifo_type, std::error_code &ec);
  bool stop_fifo(std::error_code &ec);
  bool reset_fifo(std::error_code &ec);
  uint16_t get_fifo_count(std::error_code &ec);

  /////////////////////////////////
  // Interrupts
  /////////////////////////////////
protected:
  static constexpr float RAD_TO_DEG = 57.27272727f;   ///< Radians to degrees
  static constexpr float DEG_TO_RAD = 0.01745329252f; ///< Degrees to radians

  static constexpr float TEMP_SENS = 333.87f; ///< Temperature sensitivity
  static constexpr float MAG_SENS = 0.1495f;  ///< Magnetometer sensitivity

  static constexpr uint8_t ICM20948_ID = 0xEA;      ///< ICM20948 ID
  static constexpr uint16_t AK09916C_ID_1 = 0x4809; ///< AK09916C ID 1 (company id)
  static constexpr uint16_t AK09916C_ID_2 = 0x0948; ///< AK09916C ID 2 (device id)

  static constexpr float GYRO_FS_2000_SENS = 16.4f; ///< Gyroscope sensitivity for ±2000°/s
  static constexpr float GYRO_FS_1000_SENS = 32.8f; ///< Gyroscope sensitivity for ±1000°/s
  static constexpr float GYRO_FS_500_SENS = 65.5f;  ///< Gyroscope sensitivity for ±500°/s
  static constexpr float GYRO_FS_250_SENS = 131.0f; ///< Gyroscope sensitivity for ±250°/s

  static constexpr float ACCEL_FS_16G_SENS = 2048.0f; ///< Accelerometer sensitivity for ±16g
  static constexpr float ACCEL_FS_8G_SENS = 4096.0f;  ///< Accelerometer sensitivity for ±8g
  static constexpr float ACCEL_FS_4G_SENS = 8192.0f;  ///< Accelerometer sensitivity for ±4g
  static constexpr float ACCEL_FS_2G_SENS = 16384.0f; ///< Accelerometer sensitivity for ±2g

  static constexpr uint8_t FIFO_FLUSH = (1 << 2); ///< FIFO flush bit, in register SIGNAL_PATH_RESET

  static constexpr uint8_t RESET = (1 << 7); ///< Reset bit, in register PWR_MGMT_1
  static constexpr uint8_t SLEEP = (1 << 6); ///< Sleep bit, in register PWR_MGMT_1

  static constexpr uint8_t I2C_MST_EN = (1 << 5); ///< I2C master enable bit, in register USER_CTRL
  static constexpr uint8_t LP_EN = (1 << 5);      ///< Low power enable bit, in register LP_CONFIG
  static constexpr uint8_t BYPASS_EN =
      (1 << 1); ///< I2C master bypass enable bit, in register INT_PIN_CFG

  static constexpr uint8_t GYRO_EN = 0x07;  ///< Gyroscope enable bits, in register PWR_MGMT_2
  static constexpr uint8_t ACCEL_EN = 0x38; ///< Accelerometer enable bits, in register PWR_MGMT_2

  static constexpr uint8_t FIFO_EN = (1 << 6); ///< FIFO enable bit, in register USER_CTRL

  static constexpr uint8_t BANK_SEL =
      0x7F; ///< Register bank select register (all banks). Bits [5:4] select the bank

  static const uint8_t I2C_SLVX_EN = 0x80;

  /// User register banks available
  /// Values are the bank select bits for the BANK_SEL register
  enum class Bank : uint8_t {
    _0 = 0x00, ///< Register bank 0
    _1 = 0x10, ///< Register bank 1
    _2 = 0x20, ///< Register bank 2
    _3 = 0x30, ///< Register bank 3
  };

  /// Register addresses
  enum class RegisterBank0 : uint8_t {
    /// USER BANK 0
    WHO_AM_I = 0x00,       ///< Who am I register
    USER_CTRL = 0x03,      ///< User control register
    LP_CONFIG = 0x05,      ///< Low power configuration register
    PWR_MGMT_1 = 0x06,     ///< Power management 1 register
    PWR_MGMT_2 = 0x07,     ///< Power management 2 register
    INT_PIN_CFG = 0x0F,    ///< Interrupt pin configuration register
    INT_ENABLE = 0x10,     ///< Interrupt enable register
    INT_ENABLE_1 = 0x11,   ///< Interrupt enable 1 register
    INT_ENABLE_2 = 0x12,   ///< Interrupt enable 2 register
    INT_ENABLE_3 = 0x13,   ///< Interrupt enable 3 register
    I2C_MST_STATUS = 0x17, ///< I2C master status register
    INT_STATUS = 0x19,     ///< Interrupt status register
    INT_STATUS_1 = 0x1A,   ///< Interrupt status 1 register
    INT_STATUS_2 = 0x1B,   ///< Interrupt status 2 register
    INT_STATUS_3 = 0x1C,   ///< Interrupt status 3 register
    DELAY_TIMEH = 0x28,    ///< Delay time high register
    DELAY_TIMEL = 0x29,    ///< Delay time low register

    ACCEL_DATA = 0x2D,   ///< Accelerometer start register
    ACCEL_XOUT_H = 0x2D, ///< Accelerometer X-axis output high byte register
    ACCEL_XOUT_L = 0x2E, ///< Accelerometer X-axis output low byte register
    ACCEL_YOUT_H = 0x2F, ///< Accelerometer Y-axis output high byte register
    ACCEL_YOUT_L = 0x30, ///< Accelerometer Y-axis output low byte register
    ACCEL_ZOUT_H = 0x31, ///< Accelerometer Z-axis output high byte register
    ACCEL_ZOUT_L = 0x32, ///< Accelerometer Z-axis output low byte register

    GYRO_DATA = 0x33,   ///< Gyroscope start register
    GYRO_XOUT_H = 0x33, ///< Gyroscope X-axis output high byte register
    GYRO_XOUT_L = 0x34, ///< Gyroscope X-axis output low byte register
    GYRO_YOUT_H = 0x35, ///< Gyroscope Y-axis output high byte register
    GYRO_YOUT_L = 0x36, ///< Gyroscope Y-axis output low byte register
    GYRO_ZOUT_H = 0x37, ///< Gyroscope Z-axis output high byte register
    GYRO_ZOUT_L = 0x38, ///< Gyroscope Z-axis output low byte register

    TEMP_DATA = 0x39,  ///< Temperature start register
    TEMP_OUT_H = 0x39, ///< Temperature output high byte register
    TEMP_OUT_L = 0x3A, ///< Temperature output low byte register

    MAG_DATA = 0x3B,         ///< Magnetometer start register
    EXT_SENS_DATA_00 = 0x3B, ///< External sensor data 00 register
    EXT_SENS_DATA_01 = 0x3C, ///< External sensor data 01 register
    EXT_SENS_DATA_02 = 0x3D, ///< External sensor data 02 register
    EXT_SENS_DATA_03 = 0x3E, ///< External sensor data 03 register
    EXT_SENS_DATA_04 = 0x3F, ///< External sensor data 04 register
    EXT_SENS_DATA_05 = 0x40, ///< External sensor data 05 register
    EXT_SENS_DATA_06 = 0x41, ///< External sensor data 06 register

    FIFO_EN_1 = 0x66,   ///< FIFO enable 1 register
    FIFO_EN_2 = 0x67,   ///< FIFO enable 2 register
    FIFO_RST = 0x68,    ///< FIFO reset register
    FIFO_MODE = 0x69,   ///< FIFO mode register
    FIFO_COUNTH = 0x70, ///< FIFO count high register
    FIFO_COUNTL = 0x71, ///< FIFO count low register
    FIFO_R_W = 0x72,    ///< FIFO read/write register

    DATA_RDY_STATUS = 0x74, ///< Data ready status register

    FIFO_CFG = 0x76, ///< FIFO configuration register
  };

  enum class RegisterBank1 : uint8_t {
    /// USER BANK 1
    SELF_TEST_X_GYRO = 0x02,  ///< X-axis gyroscope self-test register
    SELF_TEST_Y_GYRO = 0x03,  ///< Y-axis gyroscope self-test register
    SELF_TEST_Z_GYRO = 0x04,  ///< Z-axis gyroscope self-test register
    SELF_TEST_X_ACCEL = 0x0E, ///< X-axis accelerometer self-test register
    SELF_TEST_Y_ACCEL = 0x0F, ///< Y-axis accelerometer self-test register
    SELF_TEST_Z_ACCEL = 0x10, ///< Z-axis accelerometer self-test register

    ACCEL_X_OFFS_H = 0x77, ///< Accelerometer X-axis offset high byte register
    ACCEL_X_OFFS_L = 0x78, ///< Accelerometer X-axis offset low byte register
    ACCEL_Y_OFFS_H = 0x7A, ///< Accelerometer Y-axis offset high byte register
    ACCEL_Y_OFFS_L = 0x7B, ///< Accelerometer Y-axis offset low byte register
    ACCEL_Z_OFFS_H = 0x7D, ///< Accelerometer Z-axis offset high byte register
    ACCEL_Z_OFFS_L = 0x7E, ///< Accelerometer Z-axis offset low byte register

    TIMEBASE_CORRECTION_PLL = 0x28, ///< Timebase correction PLL register
  };

  enum class RegisterBank2 : uint8_t {
    /// USER BANK 2
    GYRO_SMPLRT_DIV = 0x00,    ///< Gyroscope sample rate divider register
    GYRO_CONFIG_1 = 0x01,      ///< Gyroscope configuration 1 register
    GYRO_CONFIG_2 = 0x02,      ///< Gyroscope configuration 2 register
    GYRO_OFFSETS_START = 0x03, ///< Gyroscope offsets start register
    GYRO_X_OFFS_H = 0x03,      ///< Gyroscope X-axis offset high byte register
    GYRO_X_OFFS_L = 0x04,      ///< Gyroscope X-axis offset low byte register
    GYRO_Y_OFFS_H = 0x05,      ///< Gyroscope Y-axis offset high byte register
    GYRO_Y_OFFS_L = 0x06,      ///< Gyroscope Y-axis offset low byte register
    GYRO_Z_OFFS_H = 0x07,      ///< Gyroscope Z-axis offset high byte register
    GYRO_Z_OFFS_L = 0x08,      ///< Gyroscope Z-axis offset low byte register
    ODR_ALIGN_EN = 0x09,       ///< ODR alignment enable register
    ACCEL_SMPLRT_DIV_1 = 0x10, ///< Accelerometer sample rate divider 1 register
    ACCEL_SMPLRT_DIV_2 = 0x11, ///< Accelerometer sample rate divider 2 register
    ACCEL_INTEL_CTRL = 0x12,   ///< Accelerometer interrupt control register
    ACCEL_WOM_THR = 0x13,      ///< Accelerometer wake-on-motion threshold register
    ACCEL_CONFIG = 0x14,       ///< Accelerometer configuration register
    ACCEL_CONFIG_2 = 0x15,     ///< Accelerometer configuration 2 register
    FSYNC_CONFIG = 0x52,       ///< FSYNC configuration register
    TEMP_CONFIG = 0x53,        ///< Temperature configuration register
    MOD_CTRL_USR = 0x54,       ///< Mode control user register
  };

  enum class RegisterBank3 : uint8_t {
    /// USER BANK 3
    I2C_MST_ODR_CONFIG = 0x00, ///< I2C master output data rate configuration register
    I2C_MST_CTRL = 0x01,       ///< I2C master control register
    I2C_MST_DELAY_CTRL = 0x02, ///< I2C master delay control register
    I2C_SLV0_ADDR = 0x03,      ///< I2C slave 0 address register
    I2C_SLV0_REG = 0x04,       ///< I2C slave 0 register register
    I2C_SLV0_CTRL = 0x05,      ///< I2C slave 0 control register
    I2C_SLV0_DO = 0x06,        ///< I2C slave 0 data out register
    I2C_SLV1_ADDR = 0x07,      ///< I2C slave 1 address register
    I2C_SLV1_REG = 0x08,       ///< I2C slave 1 register register
    I2C_SLV1_CTRL = 0x09,      ///< I2C slave 1 control register
    I2C_SLV1_DO = 0x0A,        ///< I2C slave 1 data out register
    I2C_SLV2_ADDR = 0x0B,      ///< I2C slave 2 address register
    I2C_SLV2_REG = 0x0C,       ///< I2C slave 2 register register
    I2C_SLV2_CTRL = 0x0D,      ///< I2C slave 2 control register
    I2C_SLV2_DO = 0x0E,        ///< I2C slave 2 data out register
    I2C_SLV3_ADDR = 0x0F,      ///< I2C slave 3 address register
    I2C_SLV3_REG = 0x10,       ///< I2C slave 3 register register
    I2C_SLV3_CTRL = 0x11,      ///< I2C slave 3 control register
    I2C_SLV3_DO = 0x12,        ///< I2C slave 3 data out register
    I2C_SLV4_ADDR = 0x13,      ///< I2C slave 4 address register
    I2C_SLV4_REG = 0x14,       ///< I2C slave 4 register register
    I2C_SLV4_CTRL = 0x15,      ///< I2C slave 4 control register
    I2C_SLV4_DO = 0x16,        ///< I2C slave 4 data out register
    I2C_SLV4_DI = 0x17,        ///< I2C slave 4 data in register
  };

  enum class Ak09916Register : uint8_t {
    WHO_AM_I_1 = 0x00, ///< Who am I register 1 (company id)
    WHO_AM_I_2 = 0x01, ///< Who am I register 2 (device id)
    STATUS_1 = 0x10,   ///< Status 1 register
    HXL = 0x11,        ///< X-axis low byte register
    HXH = 0x12,        ///< X-axis high byte register
    HYL = 0x13,        ///< Y-axis low byte register
    HYH = 0x14,        ///< Y-axis high byte register
    HZL = 0x15,        ///< Z-axis low byte register
    HZH = 0x16,        ///< Z-axis high byte register
    STATUS_2 = 0x18,   ///< Status 2 register
    CONTROL_2 = 0x31,  ///< Control 2 register
    CONTROL_3 = 0x32,  ///< Control 3 register
  };

  /// @brief Structure for the ICM20948 FIFO header
  struct FifoHeader {
    uint8_t odr_gyro : 1;  ///< 1: ODfor gyro is different fro this packet comapred to previous gyro
                           ///< packet
    uint8_t odr_accel : 1; ///< 1: ODfor accel is different fro this packet comapred to previous
                           ///< accel packet
    uint8_t timestamp_fsync : 2; ///< 0b00: no timestamp or fsync data, 0b01: reserved, 0b10: ODR
                                 ///< timestamp, 0b11: FSYNC timestamp
    uint8_t extended : 1;        ///< 1: packet contains 20-bit data for gyro and/or accel
    uint8_t has_gyro : 1;        ///< 1: Packet is sized so that it contains gyroscope data
    uint8_t has_accel : 1;       ///< 1: Packet is sized so that it contains accelerometer data
    uint8_t empty : 1;           ///< 1: Packet is empty, 0: packet contains sensor data
  } __attribute__((packed));

  /// @brief Struct for the FIFO Packet 1 data
  struct FifoPacket1 {
    FifoHeader header;             ///< FIFO header
    std::array<uint16_t, 3> accel; ///< Accelerometer data (x,y,z)
    uint8_t temperature;           ///< Temperature data
  };

  /// @brief Struct for the FIFO Packet 2 data
  struct FifoPacket2 {
    FifoHeader header;            ///< FIFO header
    std::array<uint16_t, 3> gyro; ///< Gyroscope data (x,y,z)
    uint8_t temperature;          ///< Temperature data
  };

  /// @brief Struct for the FIFO Packet 3 data
  struct FifoPacket3 {
    FifoHeader header;             ///< FIFO header
    std::array<uint16_t, 3> accel; ///< Accelerometer data (x,y,z)
    std::array<uint16_t, 3> gyro;  ///< Gyroscope data (x,y,z)
    uint8_t temperature;           ///< Temperature data
    uint16_t timestamp_us;         ///< Timestamp in microseconds
  };

  /// @brief Struct for the FIFO Packet 4 data
  struct FifoPacket4 {
    FifoHeader header;             ///< FIFO header
    std::array<uint16_t, 3> accel; ///< Accelerometer data (x,y,z) bits [19:4]
    std::array<uint16_t, 3> gyro;  ///< Gyroscope data (x,y,z) bits [19:4]
    uint16_t temperature;          ///< Temperature data
    uint16_t timestamp_us;         ///< Timestamp in microseconds
    uint8_t
        x_extension; ///< X-axis extension data (accelx [3:0] high nibble + gyrox [3:0] low nibble)
    uint8_t
        y_extension; ///< Y-axis extension data (accely [3:0] high nibble + gyroy [3:0] low nibble)
    uint8_t
        z_extension; ///< Z-axis extension data (accelz [3:0] high nibble + gyroz [3:0] low nibble)
  };

  static float accelerometer_range_to_sensitivty(const AccelerometerRange &range);
  static float gyroscope_range_to_sensitivty(const GyroscopeRange &range);

  uint8_t read_from_magnetometer(const Ak09916Register &reg, std::error_code &ec);
  bool write_to_magnetometer(const Ak09916Register &reg, uint8_t val, std::error_code &ec);
  bool enable_magnetometer_data_read(uint8_t reg_addr, uint8_t num_bytes, std::error_code &ec);

  uint16_t get_temperature_raw(std::error_code &ec);
  RawValue get_accelerometer_raw(std::error_code &ec);
  RawValue get_gyroscope_raw(std::error_code &ec);
  RawValue get_magnetometer_raw(std::error_code &ec);
  RawValue get_raw(RegisterBank0 reg, std::error_code &ec);

  bool select_bank(const Bank &bank, std::error_code &ec);

  Bank current_bank_{Bank::_0}; ///< Current register bank
  Value accel_values_{};
  Value gyro_values_{};
  Value mag_values_{};
  float temperature_{0.0f};
  Value orientation_{};
  Value gravity_vector_{};
  filter_fn orientation_filter_{nullptr};
  ImuConfig imu_config_{}; ///< IMU configuration
};                         // class Icm20948
} // namespace espp

// explicit template instantiation
extern template class espp::Icm20948<espp::icm20948::Interface::I2C>;
extern template class espp::Icm20948<espp::icm20948::Interface::SSI>;

#include "icm20948.hpp"

using namespace espp;

template <espp::icm20948::Interface I>
Icm20948<I>::Icm20948(const Icm20948<I>::Config &config)
    : BasePeripheral<uint8_t, I == icm20948::Interface::I2C>({}, "Icm20948", config.log_level)
    , orientation_filter_(config.orientation_filter)
    , imu_config_(config.imu_config) {
  if constexpr (I == icm20948::Interface::I2C) {
    set_address(config.device_address);
  }
  set_write(config.write);
  set_read(config.read);
  if (config.auto_init) {
    std::error_code ec;
    init(ec);
  }
}

template <espp::icm20948::Interface I> bool Icm20948<I>::init(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  auto device_id = get_device_id(ec);
  if (device_id != ICM20948_ID) {
    logger_.error("Invalid device ID: 0x{:02X}", device_id);
    return false;
  }

  // disable sleep
  if (!sleep(false, ec)) {
    return false;
  }

  // Align ODR
  if (!set_odr_align_enabled(true, ec)) {
    return false;
  }

  // enable the I2C master
  if (!set_i2c_master_enabled(true, ec)) {
    return false;
  }

  // turn on the accelerometer
  if (!enable_accelerometer(true, ec)) {
    return false;
  }

  // turn on the gyroscope
  if (!enable_gyroscope(true, ec)) {
    return false;
  }

  // initialize the magnetometer
  if (!init_magnetometer(ec)) {
    return false;
  }

  // set the configuration
  if (!set_config(imu_config_, ec)) {
    return false;
  }

  return !ec;
}

template <espp::icm20948::Interface I> uint8_t Icm20948<I>::get_device_id(std::error_code &ec) {
  return read_u8_from_register(static_cast<uint8_t>(RegisterBank0::WHO_AM_I), ec);
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_config(const Icm20948<I>::ImuConfig &imu_config, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // save the config
  imu_config_ = imu_config;

  // Set the ranges
  if (!set_accelerometer_range(imu_config.accelerometer_range, ec)) {
    return false;
  }
  if (!set_gyroscope_range(imu_config.gyroscope_range, ec)) {
    return false;
  }

  // set the sample rate dividers
  if (!set_accelerometer_sample_rate_divider(imu_config.accelerometer_sample_rate_divider, ec)) {
    return false;
  }
  if (!set_gyroscope_sample_rate_divider(imu_config.gyroscope_sample_rate_divider, ec)) {
    return false;
  }

  // now set the magnetometer operational mode
  if (!set_magnetometer_mode(imu_config.magnetometer_mode, ec)) {
    return false;
  }

  return true;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_i2c_master_enabled(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x20; // enable I2C master is bit 5 in USER_CTRL
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::USER_CTRL), bitmask,
                               enable ? bitmask : 0, ec);
  if (ec) {
    return false;
  }
  // select bank 3
  if (!select_bank(Bank::_3, ec)) {
    return false;
  }
  // set the i2c clock to 400kHz
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_MST_CTRL), 0x07, ec);
  return !ec;
}

/////////////////////////////////
// Configuration / Offsets
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_odr_align_enabled(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  uint8_t bitmask = 0x01; // enable is bit 0 in ODR_ALIGN_EN
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::ODR_ALIGN_EN), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_offsets(const float &x, const float &y, const float &z,
                                            std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 1
  if (!select_bank(Bank::_1, ec)) {
    return false;
  }
  // convert the offsets to raw values based on the sensitivity
  int16_t x_raw =
      static_cast<int16_t>(x / accelerometer_range_to_sensitivity(imu_config_.accelerometer_range));
  int16_t y_raw =
      static_cast<int16_t>(y / accelerometer_range_to_sensitivity(imu_config_.accelerometer_range));
  int16_t z_raw =
      static_cast<int16_t>(z / accelerometer_range_to_sensitivity(imu_config_.accelerometer_range));
  // build the data into an array to write as a single transaction
  uint8_t data[6] = {
      (uint8_t)((x_raw >> 8) & 0xFF), (uint8_t)(x_raw & 0xFF),
      (uint8_t)((y_raw >> 8) & 0xFF), (uint8_t)(y_raw & 0xFF),
      (uint8_t)((z_raw >> 8) & 0xFF), (uint8_t)(z_raw & 0xFF),
  };
  // write the offsets to the registers
  write_many_to_register(static_cast<uint8_t>(RegisterBank1::ACCEL_OFFSETS_START), data, 6, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::get_accelerometer_offsets(float &x, float &y, float &z, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 1
  if (!select_bank(Bank::_1, ec)) {
    return false;
  }
  // read the data from the registers
  uint8_t data[6];
  read_many_from_register(static_cast<uint8_t>(RegisterBank1::ACCEL_OFFSETS_START), data, 6, ec);
  if (ec) {
    return false;
  }
  // convert the data to floats
  int16_t x_raw = (data[0] << 8) | data[1];
  int16_t y_raw = (data[2] << 8) | data[3];
  int16_t z_raw = (data[4] << 8) | data[5];
  x = static_cast<float>(x_raw) *
      accelerometer_range_to_sensitivity(imu_config_.accelerometer_range);
  y = static_cast<float>(y_raw) *
      accelerometer_range_to_sensitivity(imu_config_.accelerometer_range);
  z = static_cast<float>(z_raw) *
      accelerometer_range_to_sensitivity(imu_config_.accelerometer_range);
  return true;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_offsets(const float &x, const float &y, const float &z,
                                        std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // convert the offsets to raw values based on the sensitivity
  int16_t x_raw =
      static_cast<int16_t>(x / gyroscope_range_to_sensitivty(imu_config_.gyroscope_range));
  int16_t y_raw =
      static_cast<int16_t>(y / gyroscope_range_to_sensitivty(imu_config_.gyroscope_range));
  int16_t z_raw =
      static_cast<int16_t>(z / gyroscope_range_to_sensitivty(imu_config_.gyroscope_range));

  // build the data into an array to write as a single transaction
  uint8_t data[6] = {
      (uint8_t)((x_raw >> 8) & 0xFF), (uint8_t)(x_raw & 0xFF),
      (uint8_t)((y_raw >> 8) & 0xFF), (uint8_t)(y_raw & 0xFF),
      (uint8_t)((z_raw >> 8) & 0xFF), (uint8_t)(z_raw & 0xFF),
  };

  // write the offsets to the registers
  write_many_to_register(static_cast<uint8_t>(RegisterBank2::GYRO_OFFSETS_START), data, 6, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::get_gyroscope_offsets(float &x, float &y, float &z, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // read the data from the registers
  uint8_t data[6];
  read_many_from_register(static_cast<uint8_t>(RegisterBank2::GYRO_OFFSETS_START), data, 6, ec);
  if (ec) {
    return false;
  }
  // convert the data to floats
  int16_t x_raw = (data[0] << 8) | data[1];
  int16_t y_raw = (data[2] << 8) | data[3];
  int16_t z_raw = (data[4] << 8) | data[5];
  x = static_cast<float>(x_raw) * gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
  y = static_cast<float>(y_raw) * gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
  z = static_cast<float>(z_raw) * gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
  return true;
}

/////////////////////////////////
// Power / Sleep / Standby
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_low_power_enabled(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x20; // enable low power is bit 5 in PWR_MGMT_1
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::PWR_MGMT_1), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_low_power_duty_cycle_mode(const DutyCycleMode &mode, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  // duty cycle mode is bits 4,5,6 in LP_CONFIG
  uint8_t bitmask = 0x70;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::LP_CONFIG), bitmask,
                               static_cast<uint8_t>(mode), ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_average_in_low_power_mode(const GyroscopeAveraging &average,
                                                          std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // bits 2:0 in GYRO_CONFIG_2
  uint8_t bitmask = 0x07;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::GYRO_CONFIG_2), bitmask,
                               static_cast<uint8_t>(average), ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_average_in_low_power_mode(const AccelerometerAveraging &average,
                                                              std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // bits 1:0 in ACCEL_CONFIG_2
  uint8_t bitmask = 0x03;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::ACCEL_CONFIG_2), bitmask,
                               static_cast<uint8_t>(average), ec);
  return !ec;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::sleep(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x40; // sleep is bit 6 in PWR_MGMT_1
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::PWR_MGMT_1), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::reset(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x80; // reset is bit 7 in PWR_MGMT_1
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::PWR_MGMT_1), bitmask, bitmask,
                               ec);
  return !ec;
}

/////////////////////////////////
// Accelerometer
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::enable_accelerometer(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  // accelerometer bits are 5:3 in PWR_MGMT_2
  uint8_t bitmask = 0x38;
  // NOTE: 111 is DISABLED, so we need to invert the enable flag
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::PWR_MGMT_2), bitmask,
                               !enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I> float Icm20948<I>::get_accelerometer_sensitivity() {
  return accelerometer_range_to_sensitivity(imu_config_.accelerometer_range);
}

template <espp::icm20948::Interface I>
float Icm20948<I>::read_accelerometer_sensitivity(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // read the data
  auto range = read_accelerometer_range(ec);
  if (ec) {
    return 0.0f;
  }
  // convert to sensitivity
  return accelerometer_range_to_sensitivity(range);
}

template <espp::icm20948::Interface I>
Icm20948<I>::AccelerometerRange Icm20948<I>::get_accelerometer_range() {
  return imu_config_.accelerometer_range;
}

template <espp::icm20948::Interface I>
Icm20948<I>::AccelerometerRange Icm20948<I>::read_accelerometer_range(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return AccelerometerRange::RANGE_2G;
  }
  // read the byte from the register
  uint8_t data = read_u8_from_register(static_cast<uint8_t>(RegisterBank2::ACCEL_CONFIG), ec);
  if (ec) {
    return AccelerometerRange::RANGE_2G;
  }
  // get the range
  AccelerometerRange range = static_cast<AccelerometerRange>(data & 0x06);
  // update the config
  imu_config_.accelerometer_range = range;
  return range;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_range(const AccelerometerRange &range, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // accelerometer range is bits 2:1 in ACCEL_CONFIG
  uint8_t bitmask = 0x06;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::ACCEL_CONFIG), bitmask,
                               static_cast<uint8_t>(range), ec);
  // update the config
  imu_config_.accelerometer_range = range;
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_dlpf_enabled(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  uint8_t bitmask = 0x01; // enable DLPF is bit 0 in ACCEL_CONFIG
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::ACCEL_CONFIG), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_dlpf(const AccelerometerFilterBandwidth &bandwidth,
                                         std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // bits 5:3 in ACCEL_CONFIG
  uint8_t bitmask = 0x38;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::ACCEL_CONFIG), bitmask,
                               static_cast<uint8_t>(bandwidth) << 3, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_accelerometer_sample_rate_divider(uint16_t sample_rate_divider,
                                                        std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // the sample rate bits 11:0 are in ACCEL_SMPLRT_DIV_1 and ACCEL_SMPLRT_DIV_2
  // make sure the value is within the range
  if (sample_rate_divider > 0x0FFF) {
    sample_rate_divider = 0x0FFF;
  }
  // write the value
  write_u16_to_register(static_cast<uint8_t>(RegisterBank2::ACCEL_SMPLRT_DIV_1),
                        sample_rate_divider, ec);
  return !ec;
}

/////////////////////////////////
// Gyroscope
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::enable_gyroscope(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  // gyroscope bits are 2:0 in PWR_MGMT_2
  uint8_t bitmask = 0x07;
  // NOTE: 111 is DISABLED, so we need to invert the enable flag
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::PWR_MGMT_2), bitmask,
                               !enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I> float Icm20948<I>::get_gyroscope_sensitivity() {
  return gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
}

template <espp::icm20948::Interface I>
float Icm20948<I>::read_gyroscope_sensitivity(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // read the data
  auto range = read_gyroscope_range(ec);
  if (ec) {
    return 0.0f;
  }
  // convert to sensitivity
  return gyroscope_range_to_sensitivty(range);
}

template <espp::icm20948::Interface I>
Icm20948<I>::GyroscopeRange Icm20948<I>::get_gyroscope_range() {
  return imu_config_.gyroscope_range;
}

template <espp::icm20948::Interface I>
Icm20948<I>::GyroscopeRange Icm20948<I>::read_gyroscope_range(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return GyroscopeRange::RANGE_250DPS;
  }
  // read the byte from the register
  uint8_t data = read_u8_from_register(static_cast<uint8_t>(RegisterBank2::GYRO_CONFIG_1), ec);
  if (ec) {
    return GyroscopeRange::RANGE_250DPS;
  }
  // get the range
  GyroscopeRange range = static_cast<GyroscopeRange>(data & 0x06);
  // update the config
  imu_config_.gyroscope_range = range;
  return range;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_range(const GyroscopeRange &range, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  uint8_t bitmask = 0x06;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::GYRO_CONFIG_1), bitmask,
                               static_cast<uint8_t>(range), ec);
  // update the config
  imu_config_.gyroscope_range = range;
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_dlpf_enabled(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  uint8_t bitmask = 0x01; // enable DLPF is bit 0 in GYRO_CONFIG_1
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::GYRO_CONFIG_2), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_dlpf(const GyroscopeFilterBandwidth &bandwidth,
                                     std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  // bits 5:3 in GYRO_CONFIG_1
  uint8_t bitmask = 0x38;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank2::GYRO_CONFIG_1), bitmask,
                               static_cast<uint8_t>(bandwidth) << 3, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_gyroscope_sample_rate_divider(uint8_t sample_rate_divider,
                                                    std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank2::GYRO_SMPLRT_DIV), sample_rate_divider,
                       ec);
  return !ec;
}

/////////////////////////////////
// Temperature
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_temperature_dlpf(const TemperatureFilterBandwidth &bandwidth,
                                       std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 2
  if (!select_bank(Bank::_2, ec)) {
    return false;
  }
  set_bits_in_register(static_cast<uint8_t>(RegisterBank2::TEMP_CONFIG),
                       static_cast<uint8_t>(bandwidth), ec);
  return !ec;
}

/////////////////////////////////
// Magnetometer
/////////////////////////////////

template <espp::icm20948::Interface I> bool Icm20948<I>::init_magnetometer(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // get the device ID and ensure it's correct
  uint16_t device_id = get_magnetometer_device_id(ec);
  if (ec) {
    return false;
  }
  // company ID is 0x48, device ID is 0x09
  if (device_id != AK09916C_ID) {
    logger_.error("Invalid magnetometer device ID: 0x{:02X}", device_id);
    return false;
  }
  return true;
}

template <espp::icm20948::Interface I>
uint16_t Icm20948<I>::get_magnetometer_device_id(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint8_t msb = read_from_magnetometer(Ak09916Register::WHO_AM_I_1, ec);
  if (ec) {
    return 0;
  }
  uint8_t lsb = read_from_magnetometer(Ak09916Register::WHO_AM_I_2, ec);
  if (ec) {
    return 0;
  }
  return (msb << 8) | lsb;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_magnetometer_mode(const icm20948::MagnetometerMode &mode,
                                        std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  if (!write_to_magnetometer(Ak09916Register::CONTROL_2, static_cast<uint8_t>(mode), ec)) {
    return false;
  }
  // make sure to enable reading the magnetometer data
  if (mode != icm20948::MagnetometerMode::POWER_DOWN) {
    if (!enable_magnetometer_data_read(static_cast<uint8_t>(Ak09916Register::HXL), 8, ec)) {
      return false;
    }
  }
  return true;
}

template <espp::icm20948::Interface I> float Icm20948<I>::get_magnetometer_sensitivity() {
  return MAG_SENS;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::reset_magnetometer(std::error_code &ec) {
  return write_to_magnetometer(Ak09916Register::CONTROL_3, 0x01, ec);
}

/////////////////////////////////
// Raw / Low level data
/////////////////////////////////
template <espp::icm20948::Interface I> Icm20948<I>::Value Icm20948<I>::get_accelerometer() {
  return accel_values_;
}

template <espp::icm20948::Interface I> Icm20948<I>::Value Icm20948<I>::get_gyroscope() {
  return gyro_values_;
}

template <espp::icm20948::Interface I> Icm20948<I>::Value Icm20948<I>::get_magnetometer() {
  return mag_values_;
}

template <espp::icm20948::Interface I> float Icm20948<I>::get_temperature() { return temperature_; }

template <espp::icm20948::Interface I>
Icm20948<I>::Value Icm20948<I>::read_accelerometer(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  RawValue raw = get_accelerometer_raw(ec);
  if (ec) {
    return {0.0f, 0.0f, 0.0f};
  }
  float sensitivity = get_accelerometer_sensitivity();
  Value value = {
      static_cast<float>(raw.x) / sensitivity,
      static_cast<float>(raw.y) / sensitivity,
      static_cast<float>(raw.z) / sensitivity,
  };
  // update values
  accel_values_ = value;
  return value;
}

template <espp::icm20948::Interface I>
Icm20948<I>::Value Icm20948<I>::read_gyroscope(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  RawValue raw = get_gyroscope_raw(ec);
  if (ec) {
    return {0.0f, 0.0f, 0.0f};
  }
  float sensitivity = get_gyroscope_sensitivity();
  Value value = {
      static_cast<float>(raw.x) / sensitivity,
      static_cast<float>(raw.y) / sensitivity,
      static_cast<float>(raw.z) / sensitivity,
  };
  // update values
  gyro_values_ = value;
  return value;
}

template <espp::icm20948::Interface I>
Icm20948<I>::Value Icm20948<I>::read_magnetometer(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  RawValue raw = get_magnetometer_raw(ec);
  if (ec) {
    return {0.0f, 0.0f, 0.0f};
  }
  float sensitivity = get_magnetometer_sensitivity();
  Value value = {
      static_cast<float>(raw.x) / sensitivity,
      static_cast<float>(raw.y) / sensitivity,
      static_cast<float>(raw.z) / sensitivity,
  };
  // update values
  mag_values_ = value;
  return value;
}

template <espp::icm20948::Interface I> float Icm20948<I>::read_temperature(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint16_t raw = get_temperature_raw(ec);
  if (ec) {
    return 0.0f;
  }
  float temp = static_cast<float>(raw) / 333.87f + 21.0f; /// 333.87 LSB/degC, 21 C offset
  // update values
  temperature_ = temp;
  return temp;
}

/////////////////////////////////
// DMP
/////////////////////////////////

template <espp::icm20948::Interface I>
bool Icm20948<I>::enable_dmp(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x80; // enable DMP is bit 7
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::USER_CTRL), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::reset_dmp(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x08; // reset DMP is bit 3
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::USER_CTRL), bitmask, bitmask,
                               ec);
  return !ec;
}

/////////////////////////////////
// Angles and Orientation
/////////////////////////////////

template <espp::icm20948::Interface I> bool Icm20948<I>::update(float dt, std::error_code &ec) {
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
  Value mag = read_magnetometer(ec);
  if (ec) {
    return false;
  }

  // if we have a filter function, then filter the data and update the
  // orientation
  if (orientation_filter_) {
    orientation_ = orientation_filter_(dt, accel, gyro, mag);
    // now calculate the gravity vector
    gravity_vector_ = {
        (float)(sinf(orientation_.pitch)),
        (float)(-sinf(orientation_.roll) * cosf(orientation_.pitch)),
        (float)(-cosf(orientation_.roll) * cosf(orientation_.pitch)),
    };
  }
  return true;
}

template <espp::icm20948::Interface I> Icm20948<I>::Value Icm20948<I>::get_orientation() {
  return orientation_;
}

template <espp::icm20948::Interface I> Icm20948<I>::Value Icm20948<I>::get_gravity_vector() {
  return gravity_vector_;
}

template <espp::icm20948::Interface I> float Icm20948<I>::get_pitch() { return orientation_.pitch; }

template <espp::icm20948::Interface I> float Icm20948<I>::get_roll() { return orientation_.roll; }

template <espp::icm20948::Interface I> float Icm20948<I>::get_yaw() { return orientation_.yaw; }

/////////////////////////////////
// FIFO
/////////////////////////////////
template <espp::icm20948::Interface I>
bool Icm20948<I>::enable_fifo(bool enable, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  uint8_t bitmask = 0x40; // enable FIFO is bit 6
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::USER_CTRL), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::set_fifo_mode(const FifoMode &mode, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  set_bits_in_register(static_cast<uint8_t>(RegisterBank0::FIFO_MODE), static_cast<uint8_t>(mode),
                       ec);
  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::start_fifo(const FifoType &fifo_type, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // TODO: implement this
  return !ec;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::stop_fifo(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // TODO: implement this
  return !ec;
}

template <espp::icm20948::Interface I> bool Icm20948<I>::reset_fifo(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return false;
  }
  // fifo reset are bits 4:0 in FIFO_RST. assert and hold to set fifo size to 0,
  // assert and de-assert to reset fifo
  uint8_t bitmask = 0x1F;
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::FIFO_RST), bitmask, bitmask, ec);
  set_bits_in_register_by_mask(static_cast<uint8_t>(RegisterBank0::FIFO_RST), bitmask, 0, ec);
  return !ec;
}

template <espp::icm20948::Interface I> uint16_t Icm20948<I>::get_fifo_count(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return 0;
  }
  return read_u16_from_register(static_cast<uint8_t>(RegisterBank0::FIFO_COUNTH), ec);
}

/////////////////////////////////
// Interrupts
/////////////////////////////////

/////////////////////////////////
// Utility
/////////////////////////////////

template <espp::icm20948::Interface I>
float Icm20948<I>::accelerometer_range_to_sensitivity(
    const Icm20948<I>::AccelerometerRange &range) {
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

template <espp::icm20948::Interface I>
float Icm20948<I>::gyroscope_range_to_sensitivty(const Icm20948<I>::GyroscopeRange &range) {
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

template <espp::icm20948::Interface I>
uint8_t Icm20948<I>::read_from_magnetometer(const Icm20948<I>::Ak09916Register &reg,
                                            std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 3
  if (!select_bank(Bank::_3, ec)) {
    return false;
  }
  // write the address of the magnetometer
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_ADDR), AK09916C_ADDRESS | 0x80,
                       ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_REG), static_cast<uint8_t>(reg),
                       ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_CTRL), I2C_SLVX_EN, ec);

  // wait for the write to complete
  while (read_u8_from_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_CTRL), ec) &
         I2C_SLVX_EN) {
    if (ec) {
      return false;
    }
  }

  // read the data
  return read_u8_from_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_DI), ec);
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::write_to_magnetometer(const Icm20948<I>::Ak09916Register &reg, uint8_t val,
                                        std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 3
  if (!select_bank(Bank::_3, ec)) {
    return false;
  }
  // write the address of the magnetometer
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_ADDR), AK09916C_ADDRESS, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_DO), val, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_REG), static_cast<uint8_t>(reg),
                       ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_CTRL), I2C_SLVX_EN, ec);

  // wait for the write to complete
  while (read_u8_from_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV4_CTRL), ec) &
         I2C_SLVX_EN) {
    if (ec) {
      return false;
    }
  }

  return !ec;
}

template <espp::icm20948::Interface I>
bool Icm20948<I>::enable_magnetometer_data_read(uint8_t reg_addr, uint8_t num_bytes,
                                                std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 3
  if (!select_bank(Bank::_3, ec)) {
    return false;
  }

  // write the address of the magnetometer
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV0_ADDR), AK09916C_ADDRESS | 0x80,
                       ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV0_REG), reg_addr, ec);
  if (ec) {
    return false;
  }
  write_u8_to_register(static_cast<uint8_t>(RegisterBank3::I2C_SLV0_CTRL), I2C_SLVX_EN | num_bytes,
                       ec);
  return !ec;
}

template <espp::icm20948::Interface I>
uint16_t Icm20948<I>::get_temperature_raw(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return 0;
  }
  return read_u16_from_register(static_cast<uint8_t>(RegisterBank0::TEMP_DATA), ec);
}

template <espp::icm20948::Interface I>
Icm20948<I>::RawValue Icm20948<I>::get_accelerometer_raw(std::error_code &ec) {
  return get_raw(RegisterBank0::ACCEL_DATA, ec);
}

template <espp::icm20948::Interface I>
Icm20948<I>::RawValue Icm20948<I>::get_gyroscope_raw(std::error_code &ec) {
  return get_raw(RegisterBank0::GYRO_DATA, ec);
}

template <espp::icm20948::Interface I>
Icm20948<I>::RawValue Icm20948<I>::get_magnetometer_raw(std::error_code &ec) {
  return get_raw(RegisterBank0::MAG_DATA, ec);
}

template <espp::icm20948::Interface I>
Icm20948<I>::RawValue Icm20948<I>::get_raw(Icm20948<I>::RegisterBank0 reg, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // select bank 0
  if (!select_bank(Bank::_0, ec)) {
    return {0, 0, 0};
  }
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

template <espp::icm20948::Interface I>
bool Icm20948<I>::select_bank(const Icm20948<I>::Bank &bank, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // if the bank is already selected, then return
  if (current_bank_ == bank) {
    return true;
  }
  // store the new bank
  current_bank_ = bank;
  // write the bank to the register
  write_u8_to_register(BANK_SEL, static_cast<uint8_t>(bank), ec);
  return !ec;
}

// explicit template instantiation
template class espp::Icm20948<espp::icm20948::Interface::I2C>;
template class espp::Icm20948<espp::icm20948::Interface::SSI>;

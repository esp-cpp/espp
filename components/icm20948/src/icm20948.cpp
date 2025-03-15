#include "icm20948.hpp"

using namespace espp;

Icm20948::Icm20948(const Config &config)
    : BasePeripheral<uint8_t, Interface == icm20948::Interface::I2C>({}, "Icm20948",
                                                                     config.log_level)
    , imu_config_(config.imu_config) {
  if constexpr (Interface == icm20948::Interface::I2C) {
    set_address(config.device_address);
  }
  set_write(config.write);
  set_read(config.read);
  if (config.auto_init) {
    std::error_code ec;
    init(ec);
  }
}

bool Icm20948::init(std::error_code &ec) {
  auto device_id = get_device_id(ec);
  if (device_id != ICM20948_ID) {
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

  return !ec;
}

uint8_t Icm20948::get_device_id(std::error_code &ec) {
  return read_u8_from_register(static_cast<uint8_t>(Register::WHO_AM_I), ec);
}

/////////////////////////////////
// Configuration / Offsets
/////////////////////////////////

bool Icm20948::auto_offsets(std::error_code &ec) { return !ec; }

bool Icm20948::set_accelerometer_offsets(const Range &x, const Range &y, const Range &z,
                                         std::error_code &ec) {
  return !ec;
}

bool Icm20948::get_accelerometer_offsets(Range &x, Range &y, Range &z, std::error_code &ec) {
  return !ec;
}

bool Icm20948::set_gyroscope_offsets(const float &x, const float &y, const float &z,
                                     std::error_code &ec) {
  return !ec;
}

bool Icm20948::get_gyroscope_offsets(float &x, float &y, float &z, std::error_code &ec) {
  return !ec;
}

/////////////////////////////////
// Power / Sleep / Standby
/////////////////////////////////

/////////////////////////////////
// Accelerometer
/////////////////////////////////

bool Icm20948::enable_accelerometer(bool enable, std::error_code &ec) {
  uint8_t bitmask = 0x38;
  set_bits_in_register_by_mask(static_cast<uint8_t>(Register::PWR_MGMT_2), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

float Icm20948::get_accelerometer_sensitivity() {
  return accelerometer_range_to_sensitivty(imu_config_.accelerometer_range);
}

float Icm20948::read_accelerometer_sensitivity(std::error_code &ec) {
  // read the data
  auto range = read_accelerometer_range(ec);
  if (ec) {
    return 0.0f;
  }
  // convert to sensitivity
  return accelerometer_range_to_sensitivty(range);
}

Icm20948::AccelerometerRange Icm20948::get_accelerometer_range() {
  return imu_config_.accelerometer_range;
}

Icm20948::AccelerometerRange Icm20948::read_accelerometer_range(std::error_code &ec) {
  // read the byte from the register
  uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::ACCEL_CONFIG0), ec);
  if (ec) {
    return AccelerometerRange::RANGE_2G;
  }
  // get the range
  AccelerometerRange range = static_cast<AccelerometerRange>((data >> 5) & 0x03);
  // update the config
  imu_config_.accelerometer_range = range;
  return range;
}

bool Icm20948::set_accelerometer_range(const AccelerometerRange &range, std::error_code &ec) {
  uint8_t bitmask = 0x06;
  set_bits_in_register_by_mask(static_cast<uint8_t>(Register::ACCEL_CONFIG), bitmask,
                               static_cast<uint8_t>(range) << 1, ec);
  // update the config
  imu_config_.accelerometer_range = range;
  return !ec;
}

bool Icm20948::set_accelerometer_dlpf(const SensorFilterBandwidth &bandwidth, std::error_code &ec) {
  return !ec;
}

bool Icm20948::set_accelerometer_odr(const AccelerometerODR &odr, std::error_code &ec) {
  return !ec;
}

/////////////////////////////////
// Gyroscope
/////////////////////////////////

bool Icm20948::enable_gyroscope(bool enable, std::error_code &ec) {
  uint8_t bitmask = 0x07;
  set_bits_in_register_by_mask(static_cast<uint8_t>(Register::PWR_MGMT_2), bitmask,
                               enable ? bitmask : 0, ec);
  return !ec;
}

float Icm20948::get_gyroscope_sensitivity() {
  return gyroscope_range_to_sensitivty(imu_config_.gyroscope_range);
}

float Icm20948::read_gyroscope_sensitivity(std::error_code &ec) {
  // read the data
  auto range = read_gyroscope_range(ec);
  if (ec) {
    return 0.0f;
  }
  // convert to sensitivity
  return gyroscope_range_to_sensitivty(range);
}

Icm20948::GyroscopeRange Icm20948::get_gyroscope_range() { return imu_config_.gyroscope_range; }

Icm20948::GyroscopeRange Icm20948::read_gyroscope_range(std::error_code &ec) {
  // read the byte from the register
  uint8_t data = read_u8_from_register(static_cast<uint8_t>(Register::GYRO_CONFIG0), ec);
  if (ec) {
    return GyroscopeRange::RANGE_250DPS;
  }
  // get the range
  GyroscopeRange range = static_cast<GyroscopeRange>((data >> 5) & 0x03);
  // update the config
  imu_config_.gyroscope_range = range;
  return range;
}

bool Icm20948::set_gyroscope_range(const GyroscopeRange &range, std::error_code &ec) {
  uint8_t bitmask = 0x06;
  set_bits_in_register_by_mask(static_cast<uint8_t>(Register::GYRO_CONFIG_1), bitmask,
                               static_cast<uint8_t>(range) << 1, ec);
  // update the config
  imu_config_.gyroscope_range = range;
  return !ec;
}

bool Icm20948::set_gyroscope_dlpf(const SensorFilterBandwidth &bandwidth, std::error_code &ec) {
  return !ec;
}

bool Icm20948::set_gyroscope_odr(const GyroscopeODR &odr, std::error_code &ec) { return !ec; }

/////////////////////////////////
// Temperature
/////////////////////////////////

bool Icm20948::set_temperature_dlpf(const TemperatureFilterBandwidth &bandwidth,
                                    std::error_code &ec) {
  set_bits_in_register(static_cast<uint8_t>(Register::TEMP_CONFIG), static_cast<uint8_t>(bandwidth),
                       ec);
  return !ec;
}

/////////////////////////////////
// Magnetometer
/////////////////////////////////
bool Icm20948::init_magnetometer(std::error_code &ec) { return !ec; }

uint16_t Icm20948::get_magnetometer_device_id(std::error_code &ec) { return 0; }

bool Icm20948::set_magnetometer_mode(const icm20948::MagnetometerMode &mode, std::error_code &ec) {
  return !ec;
}

bool Icm20948::reset_magnetometer(std::error_code &ec) { return !ec; }

/////////////////////////////////
// Raw / Low level data
/////////////////////////////////
Value Icm20948::get_accelerometer(std::error_code &ec) {
  Value value;
  return value;
}

Value Icm20948::get_gyroscope(std::error_code &ec) {
  Value value;
  return value;
}

Value Icm20948::get_magnetometer(std::error_code &ec) {
  Value value;
  return value;
}

float Icm20948::get_temperature(std::error_code &ec) { return 0.0f; }

/////////////////////////////////
// DMP
/////////////////////////////////

/////////////////////////////////
// Angles and Orientation
/////////////////////////////////
Value Icm20948::get_angles(std::error_code &ec) {
  Value value;
  return value;
}

float Icm20948::get_pitch(std::error_code &ec) { return 0.0f; }

float Icm20948::get_roll(std::error_code &ec) { return 0.0f; }

float Icm20948::get_yaw(std::error_code &ec) { return 0.0f; }

/////////////////////////////////
// FIFO
/////////////////////////////////
bool Icm20948::enable_fifo(bool enable, std::error_code &ec) {
  uint8_t bitmask = 0x06;
  set_bits_in_register_by_mask(static_cast<uint8_t>(Register::USER_CTRL), bitmask,
                               enable ? bitmask : 0, ec);
  // update the config
  imu_config_.gyroscope_range = range;
  return !ec;
}

bool Icm20948::set_fifo_mode(const FifoMode &mode, std::error_code &ec) {
  set_bits_in_register(static_cast<uint8_t>(Register::FIFO_MODE), static_cast<uint8_t>(mode), ec);
  return !ec;
}

bool Icm20948::start_fifo(const FifoType &fifo_type, std::error_code &ec) { return !ec; }

bool Icm20948::stop_fifo(std::error_code &ec) { return !ec; }

bool Icm20948::reset_fifo(std::error_code &ec) { return !ec; }

uint16_t Icm20948::get_fifo_count(std::error_code &ec) { return 0; }

/////////////////////////////////
// Interrupts
/////////////////////////////////

/////////////////////////////////
// Utility
/////////////////////////////////

float Icm20948::accelerometer_range_to_sensitivty(const Icm20948::AccelerometerRange &range) {
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

float Icm20948::gyroscope_range_to_sensitivty(const Icm20948::GyroscopeRange &range) {
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

uint16_t Icm20948::get_temperature_raw(std::error_code &ec) {
  return read_u16_from_register(static_cast<uint8_t>(Register::TEMP_DATA), ec);
}

Icm20948::RawValue Icm20948::get_accelerometer_raw(std::error_code &ec) {
  return get_raw(Register::ACCEL_DATA, ec);
}

Icm20948::RawValue Icm20948::get_gyroscope_raw(std::error_code &ec) {
  return get_raw(Register::GYRO_DATA, ec);
}

Icm20948::RawValue Icm20948::get_raw(Icm20948::Register reg, std::error_code &ec) {
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

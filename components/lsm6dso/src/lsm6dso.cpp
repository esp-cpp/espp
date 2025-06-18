#include "lsm6dso.hpp"

namespace espp {

template <lsm6dso::Interface Interface>
Lsm6dso<Interface>::Lsm6dso(const Config &config)
    : BasePeripheral<uint8_t, Interface == lsm6dso::Interface::I2C>({}, "Lsm6dso", config.log_level)
    , orientation_filter_(config.orientation_filter)
    , imu_config_(config.imu_config) {
  if constexpr (Interface == lsm6dso::Interface::I2C) {
    set_address(config.device_address);
  }
  set_write(config.write);
  set_read(config.read);
  if (config.auto_init) {
    std::error_code ec;
    init(ec);
  }
}

template <lsm6dso::Interface Interface> bool Lsm6dso<Interface>::init(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  auto device_id = get_device_id(ec);
  if (device_id != 0x6C && device_id != 0x6A) { // LSM6DSO IDs
    logger_.error("Invalid device ID: 0x{:02X}", device_id);
    return false;
  }
  // configure the accel / gyro
  if (!set_config(imu_config_, ec)) {
    logger_.error("Failed to set initial configuration: {}", ec.message());
    return false;
  }

  // enable accelerometer and gyroscope power modes
  if (!set_accelerometer_power_mode(true, ec)) {
    logger_.error("Failed to enable accelerometer power mode: {}", ec.message());
    return false;
  }
  if (!set_gyroscope_power_mode(true, ec)) {
    logger_.error("Failed to enable gyroscope power mode: {}", ec.message());
    return false;
  }

  return true;
}

template <lsm6dso::Interface Interface>
uint8_t Lsm6dso<Interface>::get_device_id(std::error_code &ec) {
  return read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::WHO_AM_I), ec);
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_config(const ImuConfig &config, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  imu_config_ = config;
  // Set accelerometer config
  if (!set_accelerometer_odr(config.accel_odr, ec)) {
    logger_.error("Failed to set accelerometer ODR: {}", ec.message());
    return false;
  }
  if (!set_accelerometer_range(config.accel_range, ec)) {
    logger_.error("Failed to set accelerometer range: {}", ec.message());
    return false;
  }
  // Set gyroscope config
  if (!set_gyroscope_odr(config.gyro_odr, ec)) {
    logger_.error("Failed to set gyroscope ODR: {}", ec.message());
    return false;
  }
  if (!set_gyroscope_range(config.gyro_range, ec)) {
    logger_.error("Failed to set gyroscope range: {}", ec.message());
    return false;
  }
  return true;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_accelerometer_odr(lsm6dso::AccelODR odr, std::error_code &ec) {
  logger_.info("Setting accelerometer ODR to {}", odr);
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // Update the CTRL1_XL register with the new ODR
  uint8_t ctrl1_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ec);
  if (ec)
    return false;
  ctrl1_xl = (ctrl1_xl & 0x0F) | (static_cast<uint8_t>(odr) << 4);
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ctrl1_xl, ec);
  if (ec)
    return false;
  // Store the ODR in the imu_config_ for later use
  imu_config_.accel_odr = odr;
  logger_.debug("Accelerometer ODR bits: 0b{:02b}", (ctrl1_xl >> 4) & 0x0F);
  logger_.info("Accelerometer ODR: {}", imu_config_.accel_odr);
  return true;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_accelerometer_range(lsm6dso::AccelRange range, std::error_code &ec) {
  logger_.info("Setting accelerometer range to {} ({})", range, static_cast<uint8_t>(range));
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // Update the CTRL1_XL register with the new range
  uint8_t ctrl1_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ec);
  if (ec)
    return false;
  ctrl1_xl = (ctrl1_xl & 0xF3) | (static_cast<uint8_t>(range) << 2);
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ctrl1_xl, ec);
  if (ec)
    return false;
  // Store the range in the imu_config_ for later use
  imu_config_.accel_range = range;
  logger_.debug("Accelerometer range bits: 0b{:02b}", (ctrl1_xl >> 2) & 0x03);
  logger_.info("Accelerometer range: {}", imu_config_.accel_range);
  return true;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_gyroscope_odr(lsm6dso::GyroODR odr, std::error_code &ec) {
  logger_.info("Setting gyroscope ODR to {}", odr);
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // Update the CTRL2_G register with the new ODR
  uint8_t ctrl2_g = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ec);
  if (ec)
    return false;
  ctrl2_g = (ctrl2_g & 0x0F) | (static_cast<uint8_t>(odr) << 4);
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ctrl2_g, ec);
  if (ec)
    return false;
  // Store the ODR in the imu_config_ for later use
  imu_config_.gyro_odr = odr;
  logger_.debug("Gyroscope ODR bits: 0b{:02b}", (ctrl2_g >> 4) & 0x0F);
  logger_.info("Gyroscope ODR: {}", imu_config_.gyro_odr);
  return true;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_gyroscope_range(lsm6dso::GyroRange range, std::error_code &ec) {
  logger_.info("Setting gyroscope range to {}", range);
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // Update the CTRL2_G register with the new range
  uint8_t ctrl2_g = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ec);
  if (ec)
    return false;
  ctrl2_g = (ctrl2_g & 0xF3) | (static_cast<uint8_t>(range) << 2);

  // 125 dps is special value, it means we have to set the FS_125 bit in CTRL2_G
  // register. If the value is NOT 125 dps, we need to ensure that the FS_125
  // bit is cleared.
  static constexpr uint8_t FS_125_BIT = 0b10; // bit 1 in CTRL2_G
  if (range == lsm6dso::GyroRange::DPS_125) {
    ctrl2_g |= FS_125_BIT; // set the FS_125 bit
  } else {
    ctrl2_g &= ~FS_125_BIT; // clear the FS_125 bit
  }

  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ctrl2_g, ec);
  if (ec)
    return false;
  // Store the range in the imu_config_ for later use
  imu_config_.gyro_range = range;
  logger_.debug("Gyroscope range bits: 0b{:02b}", (ctrl2_g >> 2) & 0x03);
  logger_.info("Gyroscope range: {}", imu_config_.gyro_range);
  return true;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::ImuConfig Lsm6dso<Interface>::get_config() const {
  return imu_config_;
}

template <lsm6dso::Interface Interface>
float Lsm6dso<Interface>::read_accelerometer_sensitivity(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // read the value of the CTRL1_XL register, update the sensitivity stored in
  // the config, and return the computed sensitivity
  uint8_t ctrl1_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ec);
  if (ec)
    return 0.0f;
  uint8_t range = (ctrl1_xl >> 2) & 0x03; // bits 2-3
  // store the range in the imu_config_ for later use
  imu_config_.accel_range = static_cast<lsm6dso::AccelRange>(range);
  logger_.debug("Accelerometer range bits: 0b{:02b}", range);
  logger_.info("Accelerometer range: {}", imu_config_.accel_range);
  return accelerometer_range_to_sensitivity(static_cast<lsm6dso::AccelRange>(range));
}

template <lsm6dso::Interface Interface>
float Lsm6dso<Interface>::read_gyroscope_sensitivity(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // read the value of the CTRL2_G register, update the sensitivity stored in
  // the config, and return the computed sensitivity
  uint8_t ctrl2_g = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ec);
  if (ec)
    return 0.0f;
  uint8_t range = (ctrl2_g >> 2) & 0x03; // bits 2-3
  // store the range in the imu_config_ for later use
  imu_config_.gyro_range = static_cast<lsm6dso::GyroRange>(range);
  logger_.debug("Gyroscope range bits: 0b{:02b}", range);
  logger_.info("Gyroscope range: {}", imu_config_.gyro_range);
  return gyroscope_range_to_sensitivity(static_cast<lsm6dso::GyroRange>(range));
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_accelerometer_power_mode(bool enable, std::error_code &ec) {
  logger_.info("Setting accelerometer power mode: {}", enable ? "ON" : "OFF");
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // get the current bits from the CTRL1_XL register
  // and set the ODR bits accordingly
  uint8_t ctrl1_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ec);
  if (ec)
    return false;
  uint8_t odr = enable ? static_cast<uint8_t>(imu_config_.accel_odr) : 0;
  // clear the ODR bits and set the new ODR
  ctrl1_xl = (ctrl1_xl & 0x0F) | (odr << 4);
  // write the updated CTRL1_XL register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ctrl1_xl, ec);
  return !ec;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_gyroscope_power_mode(bool enable, std::error_code &ec) {
  logger_.info("Setting gyroscope power mode: {}", enable ? "ON" : "OFF");
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // get the current bits from the CTRL2_G register
  // and set the ODR bits accordingly
  uint8_t ctrl2_g = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ec);
  if (ec)
    return false;
  uint8_t odr = enable ? static_cast<uint8_t>(imu_config_.gyro_odr) : 0;
  // clear the ODR bits and set the new ODR
  ctrl2_g = (ctrl2_g & 0x0F) | (odr << 4);
  // write the updated CTRL2_G register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL2_G), ctrl2_g, ec);
  return !ec;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_accelerometer_filter(uint8_t bw, AccelFilter accel_filter,
                                                  std::error_code &ec) {
  logger_.info("Setting accelerometer filter: bw={}, accel_filter={}", bw, accel_filter);
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // HPCF_XL_[2:0] bits in CTRL8_XL register are used for accelerometer filter
  if (bw > 0b111) {
    logger_.error("Invalid bandwidth value: {}", bw);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  // determine if we need to set LPF2_XL_EN bit (if lowpass or bandpass)
  bool lpf2_xl_en = (accel_filter == AccelFilter::LOWPASS || accel_filter == AccelFilter::BANDPASS);
  static constexpr uint8_t LPF2_XL_EN_BIT = 0b00000010; // bit 1
  uint8_t ctrl1_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ec);
  if (ec)
    return false;
  if (lpf2_xl_en) {
    ctrl1_xl |= LPF2_XL_EN_BIT; // set the LPF2_XL_EN bit
  } else {
    ctrl1_xl &= ~LPF2_XL_EN_BIT; // clear the LPF2_XL_EN bit
  }
  // write the updated CTRL1_XL register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL1_XL), ctrl1_xl, ec);
  if (ec)
    return false;

  logger_.debug("CTRL1_XL register after setting filter: 0b{:08b}", ctrl1_xl);

  // determine if we need to set the HP_SLOPE_XL_EN bit (if high pass or bandpass)
  bool hp_slope_xl_en =
      (accel_filter == AccelFilter::HIGHPASS || accel_filter == AccelFilter::BANDPASS);
  static constexpr uint8_t HP_SLOPE_XL_EN_BIT = 0b00000100; // bit 2

  // Set the bandwidth of the filter
  // read the CTRL8_XL register to preserve other bits
  uint8_t ctrl8_xl = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL8_XL), ec);
  if (ec)
    return false;
  // clear the CTRL8_XL bits for HPCF_XL_[2:0], which are bits 5-7
  static constexpr uint8_t HPCF_XL_MASK = 0b11100000; // bits 5-7
  ctrl8_xl &= ~HPCF_XL_MASK;                          // clear bits 5-7
  // set the CTRL8_XL bits for HPCF_XL_[2:0]
  ctrl8_xl |= (bw << 5) & HPCF_XL_MASK; // set bits 5-7 to the new bandwidth value
  // set the HP_SLOPE_XL_EN bit if needed
  if (hp_slope_xl_en) {
    ctrl8_xl |= HP_SLOPE_XL_EN_BIT; // set the HP_SLOPE_XL_EN bit
  } else {
    ctrl8_xl &= ~HP_SLOPE_XL_EN_BIT; // clear the HP_SLOPE_XL_EN bit
  }

  // write the updated CTRL8_XL register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL8_XL), ctrl8_xl, ec);
  if (ec)
    return false;

  logger_.debug("CTRL8_XL register after setting filter: 0b{:08b}", ctrl8_xl);

  return true;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::set_gyroscope_filter(uint8_t lpf1_bw, bool hpf_enabled, GyroHPF hpf_bw,
                                              std::error_code &ec) {
  logger_.info("Setting gyroscope filter: lpf1_bw={}, hpf_enabled={}, hpf_bw={}", lpf1_bw,
               hpf_enabled, hpf_bw);
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  // ensure lpf1_bw is only a 3-bit value
  if (lpf1_bw > 0b111) {
    logger_.error("Invalid bandwidth value: {}", lpf1_bw);
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  // read the CTRL6_C register to preserve other bits
  uint8_t ctrl6_c = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL6_C), ec);
  if (ec)
    return false;
  // clear the CTRL6_C bits for FTYPE[2:0]
  ctrl6_c &= 0xF8; // clear bits 2-0
  // set the CTRL6_C bits for FTYPE[2:0]
  ctrl6_c |= lpf1_bw; // set bits 2-0 to the new bandwidth value
  // write the updated CTRL6_C register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL6_C), ctrl6_c, ec);
  if (ec)
    return false;

  // now set the HPF enable bit in CTRL7_G
  uint8_t ctrl7_g = read_u8_from_register(static_cast<uint8_t>(lsm6dso::Register::CTRL7_G), ec);
  if (ec)
    return false;
  // Bit 6 in CTRL7_G is used for HPF enable
  static constexpr uint8_t HPF_ENABLE_BIT = 0b01000000; // bit 6
  if (hpf_enabled) {
    ctrl7_g |= HPF_ENABLE_BIT; // set the HPF enable bit
  } else {
    ctrl7_g &= ~HPF_ENABLE_BIT; // clear the HPF enable bit
  }
  // set the HPM0/1_G Bits in CTRL7_G
  // Bits 4-5 in CTRL7_G are used for HPM0/1_G
  static constexpr uint8_t HPM_MASK = 0b00110000; // bits 4-5
  ctrl7_g &= ~HPM_MASK;                           // clear bits 4-5
  ctrl7_g |=
      (static_cast<uint8_t>(hpf_bw) << 4) & HPM_MASK; // set bits 4-5 to the new HPF bandwidth
  // write the updated CTRL7_G register
  write_u8_to_register(static_cast<uint8_t>(lsm6dso::Register::CTRL7_G), ctrl7_g, ec);
  return !ec;
}

template <lsm6dso::Interface Interface>
bool Lsm6dso<Interface>::update(float dt, std::error_code &ec) {
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

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::RawValue Lsm6dso<Interface>::read_raw(lsm6dso::Register reg,
                                                                   std::error_code &ec) {
  RawValue data;
  read_many_from_register(static_cast<uint8_t>(reg), data.raw, 6, ec);
  if (ec) {
    return {0, 0, 0};
  }
  return data;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::get_accelerometer() const {
  return accel_values_;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::read_accelerometer(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  RawValue raw = read_accelerometer_raw(ec);
  if (ec) {
    return {0.0f, 0.0f, 0.0f};
  }
  float sensitivity = get_accelerometer_sensitivity();
  Value v = {
      static_cast<float>(raw.x) * sensitivity,
      static_cast<float>(raw.y) * sensitivity,
      static_cast<float>(raw.z) * sensitivity,
  };
  // update accel values
  accel_values_ = v;
  return v;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::get_gyroscope() const {
  return gyro_values_;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::read_gyroscope(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  RawValue raw = read_gyroscope_raw(ec);
  if (ec) {
    return {0.0f, 0.0f, 0.0f};
  }
  float sensitivity = get_gyroscope_sensitivity();
  Value v = {
      static_cast<float>(raw.x) * sensitivity,
      static_cast<float>(raw.y) * sensitivity,
      static_cast<float>(raw.z) * sensitivity,
  };
  // update gyro values
  gyro_values_ = v;
  return v;
}

template <lsm6dso::Interface Interface> float Lsm6dso<Interface>::get_temperature() const {
  return temperature_;
}

template <lsm6dso::Interface Interface>
float Lsm6dso<Interface>::read_temperature(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(base_mutex_);
  uint8_t data[2];
  read_many_from_register(static_cast<uint8_t>(lsm6dso::Register::OUT_TEMP_L), data, 2, ec);
  if (ec)
    return 0.0f;
  int16_t temp_raw = static_cast<int16_t>((data[1] << 8) | data[0]);
  temperature_ = 25.0f + (float)temp_raw / 16.0f; // per datasheet
  return temperature_;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::get_orientation() const {
  return orientation_;
}

template <lsm6dso::Interface Interface>
typename Lsm6dso<Interface>::Value Lsm6dso<Interface>::get_gravity_vector() const {
  return gravity_vector_;
}

// Explicit template instantiations
template class Lsm6dso<lsm6dso::Interface::I2C>;
template class Lsm6dso<lsm6dso::Interface::SPI>;

} // namespace espp

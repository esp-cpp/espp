#include "esp-box.hpp"

using namespace espp;

bool EspBox::initialize_imu(const EspBox::Imu::filter_fn &orientation_filter,
                            const EspBox::Imu::ImuConfig &imu_config) {
  if (imu_) {
    logger_.warn("IMU already initialized, not initializing again!");
    return false;
  }

  std::error_code ec;
  imu_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = Imu::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!imu_i2c_device_) {
    logger_.error("Could not initialize IMU I2C device: {}", ec.message());
    return false;
  }

  Imu::Config config{
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = espp::make_i2c_addressed_write(imu_i2c_device_),
      .read = espp::make_i2c_addressed_read(imu_i2c_device_),
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      .auto_init = true,
  };

  logger_.info("Initializing IMU with default configuration");
  imu_ = std::make_shared<Imu>(config);

  // configure the dmp
  // turn on DMP
  if (!imu_->set_dmp_power_save(false, ec)) {
    logger_.error("Failed to set DMP power save mode: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }
  if (!imu_->dmp_initialize(ec)) {
    logger_.error("Failed to initialize DMP: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }
  if (!imu_->set_dmp_odr(icm42607::DmpODR::ODR_25_HZ, ec)) {
    logger_.error("Failed to set DMP ODR: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }
  // set filters for the accel / gyro
  static constexpr auto filter_bw = icm42607::SensorFilterBandwidth::BW_16_HZ;
  if (!imu_->set_accelerometer_filter(filter_bw, ec)) {
    logger_.error("Failed to set accel filter: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }
  if (!imu_->set_gyroscope_filter(filter_bw, ec)) {
    logger_.error("Failed to set gyro filter: {}", ec.message());
    imu_.reset();
    imu_i2c_device_.reset();
    return false;
  }

  return true;
}

std::shared_ptr<EspBox::Imu> EspBox::imu() const { return imu_; }

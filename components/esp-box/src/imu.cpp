#include "esp-box.hpp"

using namespace espp;

bool EspBox::initialize_imu() {
  if (imu_) {
    logger_.warn("IMU already initialized, not initializing again!");
    return false;
  }

  Imu::Config config{
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3),
      .imu_config =
          {
              .accelerometer_range = Imu::AccelerometerRange::RANGE_2G,
              .accelerometer_odr = Imu::AccelerometerODR::ODR_400_HZ,
              .gyroscope_range = Imu::GyroscopeRange::RANGE_2000DPS,
              .gyroscope_odr = Imu::GyroscopeODR::ODR_400_HZ,
          },
      .auto_init = true,
  };

  logger_.info("Initializing IMU with default configuration");
  imu_ = std::make_shared<Imu>(config);

  // configure the dmp
  std::error_code ec;
  // turn on DMP
  if (!imu_->set_dmp_power_save(false, ec)) {
    logger_.error("Failed to set DMP power save mode: {}", ec.message());
    return false;
  }
  if (!imu_->dmp_initialize(ec)) {
    logger_.error("Failed to initialize DMP: {}", ec.message());
    return false;
  }
  if (!imu_->set_dmp_odr(icm42607::DmpODR::ODR_25_HZ, ec)) {
    logger_.error("Failed to set DMP ODR: {}", ec.message());
    return false;
  }
  // set filters for the accel / gyro
  static constexpr auto filter_bw = icm42607::SensorFilterBandwidth::BW_16_HZ;
  if (!imu_->set_accelerometer_filter(filter_bw, ec)) {
    logger_.error("Failed to set accel filter: {}", ec.message());
    return false;
  }
  if (!imu_->set_gyroscope_filter(filter_bw, ec)) {
    logger_.error("Failed to set gyro filter: {}", ec.message());
    return false;
  }

  return true;
}

std::shared_ptr<EspBox::Imu> EspBox::imu() const { return imu_; }

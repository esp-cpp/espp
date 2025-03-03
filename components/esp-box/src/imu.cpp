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

  return true;
}

std::shared_ptr<EspBox::Imu> EspBox::imu() const { return imu_; }

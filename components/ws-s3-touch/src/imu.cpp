#include "ws-s3-touch.hpp"

using namespace espp;

bool WsS3Touch::initialize_imu(const WsS3Touch::Imu::filter_fn &orientation_filter,
                               const WsS3Touch::Imu::ImuConfig &imu_config) {
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
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      .auto_init = true,
  };

  logger_.info("Initializing IMU with default configuration");
  imu_ = std::make_shared<Imu>(config);
  return true;
}

std::shared_ptr<WsS3Touch::Imu> WsS3Touch::imu() const { return imu_; }

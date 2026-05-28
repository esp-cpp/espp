#include "ws-s3-touch.hpp"

using namespace espp;

bool WsS3Touch::initialize_imu(const WsS3Touch::Imu::filter_fn &orientation_filter,
                               const WsS3Touch::Imu::ImuConfig &imu_config) {
  if (imu_) {
    logger_.warn("IMU already initialized, not initializing again!");
    return false;
  }

  std::error_code ec;
  auto imu_device = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = Imu::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!imu_device) {
    logger_.error("Could not initialize IMU I2C device: {}", ec.message());
    return false;
  }

  Imu::Config config{
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = espp::make_i2c_addressed_write(imu_device),
      .read = espp::make_i2c_addressed_read(imu_device),
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      .auto_init = true,
  };

  logger_.info("Initializing IMU with default configuration");
  imu_ = std::make_shared<Imu>(config);
  return true;
}

std::shared_ptr<WsS3Touch::Imu> WsS3Touch::imu() const { return imu_; }

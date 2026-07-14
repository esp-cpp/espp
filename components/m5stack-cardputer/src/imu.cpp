#include "m5stack-cardputer.hpp"

using namespace espp;

////////////////////////
// IMU Functions      //
////////////////////////

bool M5StackCardputer::initialize_imu(const Imu::filter_fn &orientation_filter,
                                      const Imu::ImuConfig &imu_config) {
  logger_.info("Initializing IMU");
  if (imu_) {
    logger_.warn("IMU already initialized, not initializing again!");
    return false;
  }
  if (variant() != Variant::ADV) {
    logger_.error("The IMU (BMI270) is only present on the Cardputer ADV");
    return false;
  }
  if (!internal_i2c_) {
    logger_.error("Internal I2C bus not available, cannot initialize the IMU");
    return false;
  }

  using namespace std::placeholders;
  imu_ = std::make_shared<Imu>(Imu::Config{
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = std::bind(&I2c::write, internal_i2c_.get(), _1, _2, _3),
      .read = std::bind(&I2c::read, internal_i2c_.get(), _1, _2, _3),
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      .auto_init = false,
      .log_level = get_log_level(),
  });

  std::error_code ec;
  if (!imu_->init(ec)) {
    logger_.error("Failed to initialize the IMU: {}", ec.message());
    imu_.reset();
    return false;
  }
  return true;
}

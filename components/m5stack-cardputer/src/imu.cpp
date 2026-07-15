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

  // the BMI270 can be strapped to either address; probe to find it
  uint8_t address = Imu::DEFAULT_ADDRESS;
  if (!internal_i2c_->probe_device(address)) {
    address = Imu::DEFAULT_ADDRESS_SDO_HIGH;
    if (!internal_i2c_->probe_device(address)) {
      logger_.error("No BMI270 found on the internal I2C bus (probed {:#04x} and {:#04x})",
                    Imu::DEFAULT_ADDRESS, Imu::DEFAULT_ADDRESS_SDO_HIGH);
      return false;
    }
  }
  logger_.info("Found BMI270 at {:#04x}", address);

  using namespace std::placeholders;
  imu_ = std::make_shared<Imu>(Imu::Config{
      .device_address = address,
      .write = std::bind(&I2c::write, internal_i2c_.get(), _1, _2, _3),
      .read = std::bind(&I2c::read, internal_i2c_.get(), _1, _2, _3),
      .imu_config = imu_config,
      .orientation_filter = orientation_filter,
      // upload the BMI270's 8kB config file in small chunks: a single
      // full-size write would exceed the internal I2C bus timeout (and use a
      // large stack buffer)
      .burst_write_size = 128,
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

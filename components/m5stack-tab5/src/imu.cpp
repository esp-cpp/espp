#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_imu(const Imu::filter_fn &orientation_filter) {
  if (imu_) {
    logger_.warn("IMU already initialized");
    return true;
  }

  logger_.info("Initializing BMI270 6-axis IMU");

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

  // Create BMI270 instance
  imu_ = std::make_shared<Imu>(Imu::Config{
      .write = espp::make_i2c_addressed_write(imu_i2c_device_),
      .read = espp::make_i2c_addressed_read(imu_i2c_device_),
      .imu_config =
          {
              .accelerometer_range = Imu::AccelerometerRange::RANGE_4G,
              .accelerometer_odr = Imu::AccelerometerODR::ODR_100_HZ,
              .accelerometer_bandwidth = Imu::AccelerometerBandwidth::NORMAL_AVG4,
              .gyroscope_range = Imu::GyroscopeRange::RANGE_1000DPS,
              .gyroscope_odr = Imu::GyroscopeODR::ODR_100_HZ,
              .gyroscope_bandwidth = Imu::GyroscopeBandwidth::NORMAL_MODE,
              .gyroscope_performance_mode = Imu::GyroscopePerformanceMode::PERFORMANCE_OPTIMIZED,
          },
      .orientation_filter = orientation_filter,
      .auto_init = true,
      .log_level = espp::Logger::Verbosity::WARN});

  logger_.info("BMI270 IMU initialized successfully");
  return true;
}

} // namespace espp

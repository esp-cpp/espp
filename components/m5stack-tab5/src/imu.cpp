#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_imu(const Imu::filter_fn &orientation_filter) {
  if (imu_) {
    logger_.warn("IMU already initialized");
    return true;
  }

  logger_.info("Initializing BMI270 6-axis IMU");

  // Create BMI270 instance
  imu_ = std::make_shared<Imu>(Imu::Config{
      .write = std::bind_front(&I2c::write, &internal_i2c_),
      .read = std::bind_front(&I2c::read, &internal_i2c_),
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

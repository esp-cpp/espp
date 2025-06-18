#include <chrono>
#include <cmath>
#include <vector>

#include "i2c.hpp"
#include "kalman_filter.hpp"
#include "logger.hpp"
#include "lsm6dso.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "LSM6DSO Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting LSM6DSO example!");

  //! [lsm6dso example]
  using Imu = espp::Lsm6dso<espp::lsm6dso::Interface::I2C>;

  // I2C config (customize as needed)
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;     // Set in sdkconfig
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO; // Set in sdkconfig
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO; // Set in sdkconfig
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});

  // make the orientation filter to compute orientation from accel + gyro
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<2> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);

  auto kalman_filter_fn = [](float dt, const Imu::Value &accel,
                             const Imu::Value &gyro) -> Imu::Value {
    // Apply Kalman filter
    float accelRoll = atan2(accel.y, accel.z);
    float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y)}, dt);
    kf.update({accelRoll, accelPitch});
    float roll, pitch;
    std::tie(roll, pitch) = kf.get_state();
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = 0.0f;
    return orientation;
  };

  // Madgwick filter for orientation
  static constexpr float beta = 0.1f;
  static espp::MadgwickFilter madgwick(beta);
  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel,
                               const Imu::Value &gyro) -> Imu::Value {
    madgwick.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x),
                    espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
    float roll, pitch, yaw;
    madgwick.get_euler(roll, pitch, yaw);
    Imu::Value orientation{};
    orientation.roll = espp::deg_to_rad(roll);
    orientation.pitch = espp::deg_to_rad(pitch);
    orientation.yaw = espp::deg_to_rad(yaw);
    return orientation;
  };

  // IMU config
  Imu::Config config{
      .device_address = Imu::DEFAULT_I2C_ADDRESS,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .imu_config =
          {
              .accel_range = Imu::AccelRange::RANGE_2G,
              .accel_odr = Imu::AccelODR::ODR_416_HZ,
              .gyro_range = Imu::GyroRange::DPS_2000,
              .gyro_odr = Imu::GyroODR::ODR_416_HZ,
          },
      .orientation_filter = kalman_filter_fn,
      .auto_init = true,
      .log_level = espp::Logger::Verbosity::INFO,
  };

  logger.info("Creating LSM6DSO IMU");
  Imu imu(config);

  std::error_code ec;

  // set the accel / gyro on-chip filters
  static constexpr uint8_t accel_filter_bandwidth = 0b001; // ODR / 10
  static constexpr uint8_t gyro_lpf_bandwidth = 0b001;     // ODR / 3
  static constexpr bool gyro_hpf_enabled = false;          // disable high-pass filter on gyro
  static constexpr auto gyro_hpf_bandwidth = Imu::GyroHPF::HPF_0_26_HZ; // 0.26Hz
  if (!imu.set_accelerometer_filter(accel_filter_bandwidth, Imu::AccelFilter::LOWPASS, ec)) {
    logger.error("Failed to set accelerometer filter: {}", ec.message());
  }
  // set the gyroscope filter to have lowpass
  if (!imu.set_gyroscope_filter(gyro_lpf_bandwidth, gyro_hpf_enabled, gyro_hpf_bandwidth, ec)) {
    logger.error("Failed to set gyroscope filter: {}", ec.message());
  }

  // make a task to read out the IMU data and print it to console
  espp::Task imu_task({.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
                         static auto start = std::chrono::steady_clock::now();

                         auto now = esp_timer_get_time(); // time in microseconds
                         static auto t0 = now;
                         auto t1 = now;
                         float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
                         t0 = t1;

                         std::error_code ec;
                         // update the imu data
                         if (!imu.update(dt, ec)) {
                           return false;
                         }

                         // get accel
                         auto accel = imu.get_accelerometer();
                         auto gyro = imu.get_gyroscope();
                         auto temp = imu.get_temperature();
                         auto orientation = imu.get_orientation();
                         auto gravity_vector = imu.get_gravity_vector();

                         [[maybe_unused]] auto t2 = esp_timer_get_time(); // time in microseconds

                         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
                         float roll = madgwick_orientation.roll;
                         float pitch = madgwick_orientation.pitch;
                         float yaw = madgwick_orientation.yaw;
                         float vx = sin(pitch);
                         float vy = -cos(pitch) * sin(roll);
                         float vz = -cos(pitch) * cos(roll);

                         // print time and raw IMU data
                         std::string text = "";
                         text += fmt::format("{:.3f},", now / 1'000'000.0f);
                         text += fmt::format("{:02.3f},{:02.3f},{:02.3f},", (float)accel.x,
                                             (float)accel.y, (float)accel.z);
                         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)gyro.x,
                                             (float)gyro.y, (float)gyro.z);
                         text += fmt::format("{:02.1f},", temp);
                         // print kalman filter outputs
                         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)orientation.x,
                                             (float)orientation.y, (float)orientation.z);
                         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)gravity_vector.x,
                                             (float)gravity_vector.y, (float)gravity_vector.z);
                         // print madgwick filter outputs
                         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", roll, pitch, yaw);
                         text += fmt::format("{:03.3f},{:03.3f},{:03.3f}", vx, vy, vz);

                         fmt::print("{}\n", text);

                         // fmt::print("IMU update took {:.3f} ms\n", (t2 - t0) / 1000.0f);

                         // sleep first in case we don't get IMU data and need to exit early
                         {
                           std::unique_lock<std::mutex> lock(m);
                           cv.wait_until(lock, start + 10ms);
                         }

                         return false;
                       },
                       .task_config = {
                           .name = "IMU",
                           .stack_size_bytes = 6 * 1024,
                           .priority = 10,
                           .core_id = 0,
                       }});

  // print the header for the IMU data (for plotting)
  fmt::print("% Time (s), "
             // raw IMU data (accel, gyro, temp)
             "Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), "
             "Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), "
             "Temp (C), "
             // kalman filter outputs
             "Kalman Roll (rad), Kalman Pitch (rad), Kalman Yaw (rad), "
             "Kalman Gravity X, Kalman Gravity Y, Kalman Gravity Z, "
             // madgwick filter outputs
             "Madgwick Roll (rad), Madgwick Pitch (rad), Madgwick Yaw (rad), "
             "Madgwick Gravity X, Madgwick Gravity Y, Madgwick Gravity Z\n");

  logger.info("Starting IMU task");
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [lsm6dso example]
}

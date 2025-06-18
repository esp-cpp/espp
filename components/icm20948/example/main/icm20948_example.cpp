#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "icm20948.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

#include "timer.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ICM20948 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [icm20948 example]
  using Imu = espp::Icm20948<espp::icm20948::Interface::I2C>;

  // make the i2c we'll use to communicate
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);
  logger.info("I2C clock speed: {} Hz", i2c_clock_speed);
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});

  std::vector<uint8_t> found_addresses;
  for (uint8_t address = 0; address < 128; address++) {
    if (i2c.probe_device(address)) {
      found_addresses.push_back(address);
    }
  }
  // print out the addresses that were found
  logger.info("Found devices at addresses: {::#02x}", found_addresses);

  // set the address of the IMU
  uint8_t imu_address = found_addresses[0];

  // make the orientation filter to compute orientation from accel + gyro
  static constexpr float angle_noise = 0.001f;
  static constexpr float rate_noise = 0.1f;
  static espp::KalmanFilter<3> kf;
  kf.set_process_noise(rate_noise);
  kf.set_measurement_noise(angle_noise);
  auto kalman_filter_fn = [](float dt, const Imu::Value &accel, const Imu::Value &gyro,
                             const Imu::Value &mag) -> Imu::Value {
    // Apply Kalman filter
    float accelRoll = atan2(accel.y, accel.z);
    float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
    // implement yaw from magnetometer
    float accelYaw = atan2(mag.y, mag.x);
    kf.predict({espp::deg_to_rad(gyro.x), espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z)}, dt);
    kf.update({accelRoll, accelPitch, accelYaw});
    float roll, pitch, yaw;
    std::tie(roll, pitch, yaw) = kf.get_state();
    // return the computed orientation
    Imu::Value orientation{};
    orientation.roll = roll;
    orientation.pitch = pitch;
    orientation.yaw = yaw;
    return orientation;
  };

  static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
  static espp::MadgwickFilter madgwick(beta);
  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel, const Imu::Value &gyro,
                               const Imu::Value &mag) -> Imu::Value {
    // Apply Madgwick filter
    madgwick.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x),
                    espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z), mag.x, mag.y, mag.z);
    float roll, pitch, yaw;
    madgwick.get_euler(roll, pitch, yaw);
    // return the computed orientation
    Imu::Value orientation{};
    orientation.pitch = espp::deg_to_rad(pitch);
    orientation.roll = espp::deg_to_rad(roll);
    orientation.yaw = espp::deg_to_rad(yaw);
    return orientation;
  };

  // make the IMU config
  Imu::Config config{
      .device_address = imu_address,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .imu_config =
          {
              .accelerometer_range = Imu::AccelerometerRange::RANGE_2G,
              .gyroscope_range = Imu::GyroscopeRange::RANGE_250DPS,
              .accelerometer_sample_rate_divider = 9, // 1kHz / (1 + 9) = 100Hz
              .gyroscope_sample_rate_divider = 9,     // 1kHz / (1 + 9) = 100Hz
              .magnetometer_mode = Imu::MagnetometerMode::CONTINUOUS_MODE_100_HZ,
          },
      .orientation_filter = kalman_filter_fn,
      .auto_init = true,
  };

  // create the IMU
  logger.info("Creating IMU");
  Imu imu(config);

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
             "Madgwick Roll (rad), Madgwick Pitch (rad), Magwick Yaw (rad), "
             "Madgwick Gravity X, Madgwick Gravity Y, Madgwick Gravity Z\n");

  logger.info("Starting IMU timer");

  // make a task to read out the IMU data and print it to console
  espp::Timer imu_timer({.period = 15ms,
                         .callback = [&]() -> bool {
                           auto now = esp_timer_get_time(); // time in microseconds
                           static auto t0 = now;
                           auto t1 = now;
                           float dt = (t1 - t0) / 1'000'000.0f; // convert us to s
                           t0 = t1;

                           std::error_code ec;
                           // update the imu data
                           if (!imu.update(dt, ec)) {
                             logger.error("Failed to update IMU: {}", ec.message());
                             return false;
                           }

                           // get accel
                           auto accel = imu.get_accelerometer();
                           auto gyro = imu.get_gyroscope();
                           auto mag = imu.get_magnetometer();
                           auto temp = imu.get_temperature();
                           auto orientation = imu.get_orientation();
                           auto gravity_vector = imu.get_gravity_vector();

                           // print time and raw IMU data
                           std::string text = "";
                           text += fmt::format("{:.3f},", now / 1'000'000.0f);
                           text += fmt::format("{:02.3f},{:02.3f},{:02.3f},", (float)accel.x,
                                               (float)accel.y, (float)accel.z);
                           text +=
                               fmt::format("{:03.3f},{:03.3f},{:03.3f},", espp::deg_to_rad(gyro.x),
                                           espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
                           text += fmt::format("{:02.1f},", temp);
                           // print kalman filter outputs
                           text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)orientation.x,
                                               (float)orientation.y, (float)orientation.z);
                           text +=
                               fmt::format("{:03.3f},{:03.3f},{:03.3f},", (float)gravity_vector.x,
                                           (float)gravity_vector.y, (float)gravity_vector.z);

                           auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro, mag);
                           float roll = madgwick_orientation.roll;
                           float pitch = madgwick_orientation.pitch;
                           float yaw = madgwick_orientation.yaw;
                           float vx = sin(pitch);
                           float vy = -cos(pitch) * sin(roll);
                           float vz = -cos(pitch) * cos(roll);

                           // print madgwick filter outputs
                           text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", roll, pitch, yaw);
                           text += fmt::format("{:03.3f},{:03.3f},{:03.3f}", vx, vy, vz);

                           fmt::print("{}\n", text);

                           return false;
                         },
                         .task_config = {
                             .name = "IMU",
                             .stack_size_bytes = 6 * 1024,
                             .priority = 10,
                             .core_id = 0,
                         }});

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [icm20948 example]
}

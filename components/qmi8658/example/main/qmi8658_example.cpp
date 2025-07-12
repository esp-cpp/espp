#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"
#include "qmi8658.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "QMI8658 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [qmi8658 example]
  using Imu = espp::Qmi8658<espp::qmi8658::Interface::I2C>;

  // make the i2c we'll use to communicate
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
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

  static constexpr float beta = 0.1f;
  static espp::MadgwickFilter madgwick(beta);
  auto madgwick_filter_fn = [](float dt, const Imu::Value &accel,
                               const Imu::Value &gyro) -> Imu::Value {
    // Apply Madgwick filter
    madgwick.update(dt, accel.x, accel.y, accel.z, espp::deg_to_rad(gyro.x),
                    espp::deg_to_rad(gyro.y), espp::deg_to_rad(gyro.z));
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
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .imu_config =
          {
              .accelerometer_range = Imu::AccelerometerRange::RANGE_8G,
              .accelerometer_odr = Imu::ODR::ODR_250_HZ,
              .gyroscope_range = Imu::GyroscopeRange::RANGE_512_DPS,
              .gyroscope_odr = Imu::ODR::ODR_250_HZ,
          },
      .orientation_filter = kalman_filter_fn,
      .auto_init = true,
  };

  // create the IMU
  logger.info("Creating IMU");
  Imu imu(config);

  // make a task to read out the IMU data and print it to console
  espp::Task imu_task({.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
                         // sleep first in case we don't get IMU data and need to exit early
                         {
                           std::unique_lock<std::mutex> lock(m);
                           cv.wait_for(lock, 10ms);
                         }

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

                         auto madgwick_orientation = madgwick_filter_fn(dt, accel, gyro);
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

  logger.info("Starting IMU task");
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [qmi8658 example]
}

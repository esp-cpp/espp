#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "icm42607.hpp"
#include "kalman_filter.hpp"
#include "madgwick_filter.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ICM42607 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [icm42607 example]
  using Imu = espp::Icm42607<espp::icm42607::Interface::I2C>;

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

  // make the IMU config
  Imu::Config config{
      .device_address = Imu::DEFAULT_ADDRESS,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .imu_config =
          {
              .accelerometer_range = Imu::AccelerometerRange::RANGE_2G,
              .accelerometer_odr = Imu::AccelerometerODR::ODR_400_HZ,
              .gyroscope_range = Imu::GyroscopeRange::RANGE_2000DPS,
              .gyroscope_odr = Imu::GyroscopeODR::ODR_400_HZ,
          },
      .auto_init = true,
  };

  // create the IMU
  logger.info("Creating IMU");
  Imu imu(config);
  std::error_code ec;

  // turn on DMP
  logger.info("Turning on DMP");
  if (!imu.set_dmp_power_save(false, ec)) {
    logger.error("Failed to set DMP power save mode: {}", ec.message());
    return;
  }

  // initialize the DMP
  logger.info("Initializing DMP");
  if (!imu.dmp_initialize(ec)) {
    logger.error("Failed to initialize DMP: {}", ec.message());
    return;
  }

  // set the DMP output data rate
  logger.info("Setting DMP output data rate (ODR)");
  if (!imu.set_dmp_odr(espp::icm42607::DmpODR::ODR_25_HZ, ec)) {
    logger.error("Failed to set DMP ODR: {}", ec.message());
    return;
  }

  // set filters for the accel / gyro
  logger.info("Setting accel and gyro filters");
  static constexpr auto filter_bw = espp::icm42607::SensorFilterBandwidth::BW_16_HZ;
  if (!imu.set_accelerometer_filter(filter_bw, ec)) {
    logger.error("Failed to set accel filter: {}", ec.message());
    return;
  }
  if (!imu.set_gyroscope_filter(filter_bw, ec)) {
    logger.error("Failed to set gyro filter: {}", ec.message());
    return;
  }

  // make a task to read out the IMU data and print it to console
  espp::Task imu_task(
      {.callback = [&imu](std::mutex &m, std::condition_variable &cv) -> bool {
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
         // get accel
         auto accel = imu.get_accelerometer(ec);
         if (ec) {
           return false;
         }
         auto gyro = imu.get_gyroscope(ec);
         if (ec) {
           return false;
         }
         auto temp = imu.get_temperature(ec);
         if (ec) {
           return false;
         }

         std::string text = "";
         text += fmt::format("{:.3f},", now / 1'000'000.0f);
         text += fmt::format("{:02.3f},{:02.3f},{:02.3f},", accel.x, accel.y, accel.z);
         text += fmt::format("{:03.3f},{:03.3f},{:03.3f},", gyro.x, gyro.y, gyro.z);
         text += fmt::format("{:02.1f},", temp);

         float roll = 0, pitch = 0;

         // with only the accelerometer + gyroscope, we can't get yaw :(
         static constexpr float angle_noise = 0.001f;
         static constexpr float rate_noise = 0.1f;
         static espp::KalmanFilter<2> kf;
         kf.set_process_noise(rate_noise);
         kf.set_measurement_noise(angle_noise);
         static constexpr float beta = 0.1f; // higher = more accelerometer, lower = more gyro
         static espp::MadgwickFilter f(beta);

         f.update(dt, accel.x, accel.y, accel.z, gyro.x * M_PI / 180.0f, gyro.y * M_PI / 180.0f,
                  gyro.z * M_PI / 180.0f);

         float rollRad = roll * M_PI / 180.0f;
         float pitchRad = pitch * M_PI / 180.0f;

         float vx = sin(pitchRad);
         float vy = -cos(pitchRad) * sin(rollRad);
         float vz = -cos(pitchRad) * cos(rollRad);

         text += fmt::format("{:03.3f},{:03.3f},", roll, pitch);
         text += fmt::format("{:03.3f},{:03.3f},{:03.3f}", vx, vy, vz);

         // Apply Kalman filter
         float accelPitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z));
         float accelRoll = atan2(accel.y, accel.z);
         kf.predict({float(gyro.x * M_PI / 180.0f), float(gyro.y * M_PI / 180.0f)}, dt);
         kf.update({accelPitch, accelRoll});
         auto state = kf.get_state();
         pitch = state[0];
         roll = state[1];

         rollRad = roll;
         pitchRad = pitch;

         vx = sin(pitchRad);
         vy = -cos(pitchRad) * sin(rollRad);
         vz = -cos(pitchRad) * cos(rollRad);

         text += fmt::format("{:03.3f},{:03.3f},", roll, pitch);
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
  fmt::print("% Time (s), Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), Gyro X (rad/s), Gyro "
             "Y (rad/s), Gyro Z (rad/s), Temp (C), Roll (deg), Pitch (deg), Gravity X, Gravity Y, "
             "Gravity Z, Roll (KF), Pitch (KF), Gravity X (KF), Gravity Y (KF), Gravity Z (KF)\n");

  logger.info("Starting IMU task");
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [icm42607 example]
}

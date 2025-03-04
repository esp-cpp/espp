#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "icm42607.hpp"
#include "kalman_filter.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ICM42607 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [icm42607 example]
  using Imu = espp::Icm42607<espp::icm42607::Interface::I2C>;

  // make the i2c we'll use to communicate
  static constexpr auto internal_i2c_port = I2C_NUM_0;
  static constexpr auto internal_i2c_clock_speed = 400 * 1000;
  static constexpr gpio_num_t internal_i2c_sda = GPIO_NUM_8;
  static constexpr gpio_num_t internal_i2c_scl = GPIO_NUM_18;
  espp::I2c i2c{{.port = internal_i2c_port,
                 .sda_io_num = internal_i2c_sda,
                 .scl_io_num = internal_i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE}};

  // initialize the IMU
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

  Imu imu(config);
  std::error_code ec;
  // turn on DMP
  if (!imu.set_dmp_power_save(false, ec)) {
    logger.error("Failed to set DMP power save mode: {}", ec.message());
    return;
  }
  if (!imu.dmp_initialize(ec)) {
    logger.error("Failed to initialize DMP: {}", ec.message());
    return;
  }
  if (!imu.set_dmp_odr(espp::icm42607::DmpODR::ODR_25_HZ, ec)) {
    logger.error("Failed to set DMP ODR: {}", ec.message());
    return;
  }
  // set filters for the accel / gyro
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

         float roll = 0, pitch = 0;

         // with only the accelerometer + gyroscope, we can't get yaw :(
         static espp::KalmanFilter kalmanPitch;
         static espp::KalmanFilter kalmanRoll;

         // Compute pitch and roll from accelerometer
         float accelPitch =
             atan2(accel.y, sqrt(accel.x * accel.x + accel.z * accel.z)) * 180.0f / M_PI;
         float accelRoll = atan2(-accel.x, accel.z) * 180.0f / M_PI;

         // Apply Kalman filter
         pitch = kalmanPitch.update(accelPitch, gyro.y, dt);
         roll = kalmanRoll.update(accelRoll, gyro.x, dt);

         std::string text = "";
         text += fmt::format("Accel: {:02.2f} {:02.2f} {:02.2f}\n", accel.x, accel.y, accel.z);
         text += fmt::format("Gyro: {:03.2f} {:03.2f} {:03.2f}\n", gyro.x, gyro.y, gyro.z);
         text += fmt::format("Angle: {:03.2f} {:03.2f}\n", roll, pitch);
         text += fmt::format("Temp: {:02.1f} C\n", temp);

         float rollRad = roll * M_PI / 180.0f;
         float pitchRad = pitch * M_PI / 180.0f;

         float vx = sin(pitchRad);
         float vy = -cos(pitchRad) * sin(rollRad);
         float vz = -cos(pitchRad) * cos(rollRad);

         int x0 = 50;
         int y0 = 50;
         int x1 = x0 - 50 * vy;
         int y1 = y0 - 50 * vx;

         return false;
       },
       .task_config = {
           .name = "IMU",
           .stack_size_bytes = 6 * 1024,
           .priority = 10,
           .core_id = 0,
       }});
  imu_task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [icm42607 example]
}

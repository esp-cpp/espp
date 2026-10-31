#include <chrono>
#include <sdkconfig.h>

#include "bno055.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "BNO055 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [bno055 example]
  // make the i2c we'll use to communicate
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
  logger.info("Creating I2C on port {} with SDA {} and SCL {}", i2c_port, i2c_sda, i2c_scl);
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});

  static constexpr uint8_t address = CONFIG_EXAMPLE_I2C_ADDRESS;
  std::error_code ec;
  auto imu_device =
      i2c.add_device<uint8_t>({.device_address = address,
                               .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                               .scl_speed_hz = i2c.config().clk_speed,
                               .log_level = espp::Logger::Verbosity::WARN},
                              ec);
  if (!imu_device) {
    logger.error("Failed to create BNO055 I2C device: {}", ec.message());
    return;
  }

  // create the IMU; auto_init resets it and enters IMU (accel + gyro) fusion
  // mode, which takes about 700 ms
  espp::Bno055 imu({.device_address = address,
                    .probe = espp::make_i2c_addressed_probe(imu_device),
                    .write = espp::make_i2c_addressed_write(imu_device),
                    .write_then_read = espp::make_i2c_addressed_write_then_read(imu_device),
                    .mode = espp::Bno055::OperatingMode::IMU,
                    .use_external_crystal = true,
                    .auto_init = true,
                    .log_level = espp::Logger::Verbosity::INFO});
  if (!imu.initialized()) {
    logger.error("BNO055 not found at address {:#04x}", address);
    return;
  }

  // print the header for the IMU data (for plotting)
  fmt::print("% Time (s), "
             "Accel X (m/s^2), Accel Y (m/s^2), Accel Z (m/s^2), "
             "Gyro X (rad/s), Gyro Y (rad/s), Gyro Z (rad/s), "
             "Heading (rad), Roll (rad), Pitch (rad), "
             "Quat W, Quat X, Quat Y, Quat Z, "
             "Temp (C), Calib Sys, Calib Gyro, Calib Accel, Calib Mag\n");

  // read the IMU at 100 Hz and print one CSV row per sample
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    auto start = std::chrono::high_resolution_clock::now();

    std::error_code ec;
    if (!imu.update(ec)) {
      logger.error("Failed to update BNO055: {}", ec.message());
    } else {
      auto data = imu.data();
      std::string text = fmt::format("{:.3f},", esp_timer_get_time() / 1'000'000.0f);
      text += fmt::format("{:.3f},{:.3f},{:.3f},", data.accel_mps2.x, data.accel_mps2.y,
                          data.accel_mps2.z);
      text += fmt::format("{:.4f},{:.4f},{:.4f},", data.gyro_radps.x, data.gyro_radps.y,
                          data.gyro_radps.z);
      text += fmt::format("{:.4f},{:.4f},{:.4f},", data.euler.heading, data.euler.roll,
                          data.euler.pitch);
      text += fmt::format("{:.4f},{:.4f},{:.4f},{:.4f},", data.orientation.w, data.orientation.x,
                          data.orientation.y, data.orientation.z);
      text += fmt::format("{},{},{},{},{}", data.temperature_c, data.calibration.system,
                          data.calibration.gyro, data.calibration.accel, data.calibration.mag);
      fmt::print("{}\n", text);
    }

    std::unique_lock<std::mutex> lk(m);
    cv.wait_until(lk, start + 10ms);
    return false;
  };

  espp::Task task({.callback = task_fn,
                   .task_config = {.name = "BNO055 Task", .stack_size_bytes = 6 * 1024},
                   .log_level = espp::Logger::Verbosity::WARN});
  task.start();

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [bno055 example]
}

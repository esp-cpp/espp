#include <chrono>
#include <string>
#include <thread>

#include "i2c.hpp"
#include "logger.hpp"
#include "timer.hpp"
#include "vl53l5cx.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "VL53L5CX example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [vl53l5cx example]
  // make the i2c we'll use to communicate
  static constexpr auto i2c_port = I2C_NUM_0;
  static constexpr auto i2c_clock_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ;
  static constexpr gpio_num_t i2c_sda = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO;
  static constexpr gpio_num_t i2c_scl = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO;
  logger.info("Creating I2C on port {} with SDA {} and SCL {} at {} Hz", i2c_port, i2c_sda, i2c_scl,
              i2c_clock_speed);
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .clk_speed = i2c_clock_speed});
  std::error_code ec;
  // 16-bit register addresses, sent MSB first. The transaction timeout must
  // cover the driver's longest transfers: 512-byte firmware chunks and
  // ~1.4 KB result frames (a few tens of ms at 400 kHz).
  auto device = i2c.add_device<uint16_t>({.device_address = espp::Vl53l5cx::DEFAULT_ADDRESS,
                                          .timeout_ms = 100,
                                          .scl_speed_hz = i2c.config().clk_speed,
                                          .log_level = espp::Logger::Verbosity::WARN},
                                         ec);
  if (!device) {
    logger.error("Failed to add the VL53L5CX I2C device: {}", ec.message());
    return;
  }

  // the constructor reboots the sensor and uploads its firmware (about two
  // seconds at 400 kHz)
  espp::Vl53l5cx sensor(espp::Vl53l5cx::Config{
      .device_address = espp::Vl53l5cx::DEFAULT_ADDRESS,
      .write = espp::make_i2c_addressed_write<uint16_t>(device),
      .read = espp::make_i2c_addressed_read<uint16_t>(device),
      .log_level = espp::Logger::Verbosity::INFO,
  });
  if (!sensor.is_initialized()) {
    logger.error("The VL53L5CX did not initialize; check the wiring and the address");
    return;
  }

  // resolution first (it bounds the ranging frequency), then the frame rate
#if CONFIG_EXAMPLE_RESOLUTION_8X8
  const auto resolution = espp::Vl53l5cx::Resolution::ZONES_8X8;
#else
  const auto resolution = espp::Vl53l5cx::Resolution::ZONES_4X4;
#endif
  if (!sensor.set_resolution(resolution, ec)) {
    logger.error("Failed to set the resolution: {}", ec.message());
    return;
  }
  if (!sensor.set_ranging_frequency_hz(CONFIG_EXAMPLE_RANGING_FREQUENCY_HZ, ec)) {
    logger.error("Failed to set the ranging frequency: {}", ec.message());
    return;
  }
  if (!sensor.start_ranging(ec)) {
    logger.error("Failed to start ranging: {}", ec.message());
    return;
  }

  // poll for frames and print the distance grid (mm; "----" where the zone
  // has no valid target)
  espp::Vl53l5cx::RangingData frame;
  auto poll_fn = [&]() {
    std::error_code ec;
    if (!sensor.is_data_ready(ec)) {
      return false; // nothing new yet (or an I2C error, logged by the driver)
    }
    if (!sensor.get_ranging_data(frame, ec)) {
      logger.error("Failed to read the frame: {}", ec.message());
      return false;
    }
    std::string grid;
    for (size_t row = 0; row < frame.width(); row++) {
      for (size_t col = 0; col < frame.width(); col++) {
        const size_t zone = frame.zone_index(row, col);
        grid += frame.is_valid(zone) ? fmt::format("{:5d}", frame.distance_mm[zone]) : " ----";
      }
      grid += "\n";
    }
    fmt::print("{}\n", grid);
    return false; // keep polling
  };
  espp::Timer timer({.period = 5ms,
                     .callback = poll_fn,
                     .task_config = {.name = "vl53l5cx poll", .stack_size_bytes = 6 * 1024},
                     .log_level = espp::Logger::Verbosity::WARN});
  //! [vl53l5cx example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

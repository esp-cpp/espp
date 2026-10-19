#include <chrono>
#include <map>
#include <string>
#include <thread>

#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"
#include "vl53l5cx.hpp"

using namespace std::chrono_literals;

namespace {
int invalid_total(const std::map<espp::Vl53l5cx::TargetStatus, int> &invalid) {
  int total = 0;
  for (const auto &[_status, count] : invalid)
    total += count;
  return total;
}
} // namespace

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
  // The transaction timeout must cover the driver's longest transfers (512-byte
  // firmware chunks, ~1.4 KB result frames): derived from the clock
  static constexpr int i2c_timeout_ms = espp::Vl53l5cx::i2c_timeout_ms(i2c_clock_speed);
  espp::I2c i2c({.port = i2c_port,
                 .sda_io_num = i2c_sda,
                 .scl_io_num = i2c_scl,
                 .sda_pullup_en = GPIO_PULLUP_ENABLE,
                 .scl_pullup_en = GPIO_PULLUP_ENABLE,
                 .timeout_ms = i2c_timeout_ms,
                 .clk_speed = i2c_clock_speed});
  std::error_code ec;

  // the constructor reboots the sensor and uploads its firmware (about two
  // seconds at 400 kHz). The callbacks take the address from the driver, so
  // set_i2c_address() keeps working (a per-device adapter would not).
  espp::Vl53l5cx sensor(espp::Vl53l5cx::Config {
    .device_address = espp::Vl53l5cx::DEFAULT_ADDRESS,
#if CONFIG_EXAMPLE_VARIANT_VL53L7CX
    .variant = espp::Vl53l5cx::Variant::VL53L7CX,
#else
      .variant = espp::Vl53l5cx::Variant::VL53L5CX,
#endif
    .write = [&i2c](uint8_t address, const uint8_t *data,
                    size_t length) { return i2c.write(address, data, length); },
    .read = [&i2c](uint8_t address, uint8_t *data,
                   size_t length) { return i2c.read(address, data, length); },
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
  // has no valid target, with a summary of why). A plain task rather than a
  // periodic timer: a frame read is ~1.4 KB of I2C (about 40 ms at 400 kHz),
  // far longer than a sensible data-ready poll interval.
  espp::Vl53l5cx::RangingData frame;
  espp::Task poll_task({
      .callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
        std::error_code ec;
        if (!sensor.is_data_ready(ec)) {
          // nothing new yet (an I2C error is logged by the driver); check again soon
          std::unique_lock<std::mutex> lock(m);
          cv.wait_for(lock, 5ms);
          return false;
        }
        if (!sensor.get_ranging_data(frame, ec)) {
          logger.error("Failed to read the frame: {}", ec.message());
          return false;
        }
        std::string grid;
        std::map<espp::Vl53l5cx::TargetStatus, int> invalid;
        for (size_t row = 0; row < frame.width(); row++) {
          for (size_t col = 0; col < frame.width(); col++) {
            const size_t zone = frame.zone_index(row, col);
            if (frame.is_valid(zone)) {
              grid += fmt::format("{:5d}", frame.distance_mm[zone]);
            } else {
              grid += " ----";
              invalid[frame.status(zone)]++;
            }
          }
          grid += "\n";
        }
        std::string why;
        for (const auto &[status, count] : invalid) {
          why += fmt::format("  {} x {} ({})", count, static_cast<int>(status),
                             espp::Vl53l5cx::target_status_name(status));
        }
        fmt::print("{}{} / {} zones valid{}\n\n", grid, frame.num_zones() - invalid_total(invalid),
                   frame.num_zones(), why.empty() ? "" : ";" + why);
        return false; // keep polling
      },
      .task_config = {.name = "vl53l5cx poll", .stack_size_bytes = 6 * 1024},
      .log_level = espp::Logger::Verbosity::WARN,
  });
  poll_task.start();
  //! [vl53l5cx example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

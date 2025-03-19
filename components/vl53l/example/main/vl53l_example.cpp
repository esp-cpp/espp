#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "logger.hpp"
#include "timer.hpp"
#include "vl53l.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "VL53L4CX example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (vl53l)
  {
    logger.info("Starting example!");

    //! [vl53l example]
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

    // make the actual test object
    espp::Vl53l vl53l(
        espp::Vl53l::Config{.device_address = espp::Vl53l::DEFAULT_ADDRESS,
                            .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                               std::placeholders::_2, std::placeholders::_3),
                            .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                              std::placeholders::_2, std::placeholders::_3),
                            .log_level = espp::Logger::Verbosity::WARN});

    std::error_code ec;
    // set the timing budget to 10ms, which must be shorter than the
    // inter-measurement period. We'll log every 20ms so this guarantees we get
    // new data every time
    if (!vl53l.set_timing_budget_ms(10, ec)) {
      logger.error("Failed to set inter measurement period: {}", ec.message());
      return;
    }
    // set the inter-measurement period to 10ms, so we should be sure to get new
    // data each measurement
    if (!vl53l.set_inter_measurement_period_ms(10, ec)) {
      logger.error("Failed to set inter measurement period: {}", ec.message());
      return;
    }
    // tell it to start ranging
    if (!vl53l.start_ranging(ec)) {
      logger.error("Failed to start ranging: {}", ec.message());
      return;
    }

    // make the task which will read the vl53l
    fmt::print("%time (s), distance (m)\n");
    auto read_task_fn = [&vl53l]() {
      auto now = esp_timer_get_time();
      static auto start = now;
      float elapsed = (float)(now - start) / 1e6;
      std::error_code ec;
      // wait for the data to be ready
      while (!vl53l.is_data_ready(ec)) {
        std::this_thread::sleep_for(1ms);
      }
      // clear the interrupt so we can get another reading
      if (!vl53l.clear_interrupt(ec)) {
        logger.error("Failed to clear interrupt: {}", ec.message());
        return false;
      }
      auto meters = vl53l.get_distance_meters(ec);
      if (ec) {
        logger.error("Failed to get distance: {}", ec.message());
        return false;
      }
      fmt::print("{:.3f}, {:.3f}\n", elapsed, meters);
      // we don't want to stop, so return false
      return false;
    };

    espp::Timer timer({.period = 20ms,
                       .callback = read_task_fn,
                       .task_config =
                           {
                               .name = "VL53L4CX",
                               .stack_size_bytes{4 * 1024},
                           },
                       .log_level = espp::Logger::Verbosity::INFO});
    //! [vl53l example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

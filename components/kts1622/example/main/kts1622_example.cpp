#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "kts1622.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  espp::Logger logger({.tag = "KTS1622 example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting kts1622 example");

  //! [kts1622 example]
  // make the I2C that we'll use to communicate
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = 1000 * 1000, // 1MHz
  });

  // now make the kts1622 which handles GPIO
  espp::Kts1622 kts1622(
      {.device_address = espp::Kts1622::DEFAULT_ADDRESS,
       // set P0_0 - P0_7 to be inputs
       .port_0_direction_mask = 0b11111111,
       // set P1_0 - P1_7 to be inputs
       .port_1_direction_mask = 0b11111111,
       .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
       .write_then_read =
           std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
       .auto_init = false,
       .log_level = espp::Logger::Verbosity::INFO});
  std::error_code ec;
  kts1622.initialize(
      ec); // Initialized separately from the constructor since we set auto_init to false
  if (ec) {
    logger.error("kts1622 initialization failed: {}", ec.message());
    return;
  }

  // and finally, make the task to periodically poll the kts1622 and print
  // the state. NOTE: the Kts1622 does not internally manage its own state
  // update, so whatever rate we use here is the rate at which the state will
  // update.
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    // returns the pins as P0_0 lsb, P1_7 msb
    auto pins = kts1622.get_pins(ec);
    if (ec) {
      logger.error("kts1622 get pins failed: {}", ec.message());
      return true; // stop the task
    }
    // equivalent to:
    // auto pins = (kts1622.get_pins(espp::Kts1622::Port::PORT1, ec) << 8) |
    // kts1622.get_pins(espp::Kts1622::Port::PORT0, ec);
    fmt::print("{:.3f}, {:#x}\n", seconds, pins);
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 50ms);
    }
    // don't want to stop the task
    return false;
  };
  auto task = espp::Task({.callback = task_fn,
                          .task_config =
                              {
                                  .name = "Kts1622 Task",
                                  .stack_size_bytes = 5 * 1024,
                              },
                          .log_level = espp::Logger::Verbosity::WARN});
  fmt::print("% time(s), pin values\n");
  task.start();
  //! [kts1622 example]
  while (true) {
    std::this_thread::sleep_for(100ms);
  }

  logger.info("Kts1622 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "pcf85063.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "PCF85063 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [pcf85063_example]

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

  // now make the pcf85063
  espp::Pcf85063 rtc({.device_address = espp::Pcf85063::DEFAULT_ADDRESS,
                      .write = std::bind_front(&espp::I2c::write, &i2c),
                      .read = std::bind_front(&espp::I2c::read, &i2c)});

  // set the time
  std::tm time;
  time.tm_sec = 0;
  time.tm_min = 0;
  time.tm_hour = 0;
  time.tm_mday = 1;
  time.tm_mon = 0;
  time.tm_year = 2023 - 1900;
  std::error_code ec;
  rtc.set_time(time, ec);
  if (ec) {
    logger.error("Failed to set time: {}", ec.message());
    return;
  }
  logger.info("Time set to: {}.{:02d}.{:02d} {:02d}:{:02d}:{:02d}", time.tm_year + 1900,
              time.tm_mon + 1, time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);

  // now start a task to read the time every second
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) -> bool {
    std::error_code ec;
    auto time = rtc.get_time(ec);
    if (ec) {
      logger.error("Failed to get time: {}", ec.message());
      return true; // stop the task
    }
    logger.info("Time: {}.{:02d}.{:02d} {:02d}:{:02d}:{:02d}", time.tm_year + 1900, time.tm_mon + 1,
                time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec);
    // use the mutex and condition variable to perform an interruptible sleep
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, 1s, [] { return false; }); // wait for 1 second or until notified
    // don't stop the task
    return false;
  };

  auto task = espp::Task({.callback = task_fn,
                          .task_config =
                              {
                                  .name = "pcf85063",
                                  .stack_size_bytes = 4096,
                              },
                          .log_level = espp::Logger::Verbosity::INFO});
  task.start();
  //! [pcf85063_example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

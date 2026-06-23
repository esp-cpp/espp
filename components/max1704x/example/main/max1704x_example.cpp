#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "logger.hpp"
#include "max1704x.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  //! [max1704x example]
  espp::Logger logger({.tag = "Max1704x example", .level = espp::Logger::Verbosity::INFO});
  // make the I2C that we'll use to communicate
  logger.info("initializing i2c driver...");
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });
  std::error_code ec;
  auto max1704x_device =
      i2c.add_device<uint8_t>({.device_address = espp::Max1704x::DEFAULT_ADDRESS,
                               .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                               .scl_speed_hz = i2c.config().clk_speed,
                               .log_level = espp::Logger::Verbosity::WARN},
                              ec);
  if (!max1704x_device) {
    logger.error("MAX1704x I2C device initialization failed: {}", ec.message());
    return;
  }
  // now make the max1704x which handles GPIO
  espp::Max1704x max1704x({.write = espp::make_i2c_addressed_write(max1704x_device),
                           .read = espp::make_i2c_addressed_read(max1704x_device),
                           .log_level = espp::Logger::Verbosity::WARN});

  // and finally, make the task to periodically poll the max1704x and print
  // the state. NOTE: the Max1704x does not internally manage its own state
  // update, so whatever rate we use here is the rate at which the state will
  // update.
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 50ms);
    }
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto voltage = max1704x.get_battery_voltage(ec);
    if (ec) {
      return false;
    }
    auto soc = max1704x.get_battery_percentage(ec);
    if (ec) {
      return false;
    }
    auto charge_rate = max1704x.get_battery_charge_rate(ec);
    if (ec) {
      return false;
    }
    fmt::print("{:0.2f}, {:0.2f}, {:0.2f}, {:0.2f}\n", seconds, voltage, soc, charge_rate);
    // don't want to stop the task
    return false;
  };
  auto task = espp::Task({.callback = task_fn,
                          .task_config =
                              {
                                  .name = "Max1704x Task",
                                  .stack_size_bytes = 5 * 1024,
                              },
                          .log_level = espp::Logger::Verbosity::WARN});
  fmt::print("%time(s), voltage (V), SoC (%), Charge Rate (%/hr)\n");
  task.start();
  //! [max1704x example]
  while (true) {
    std::this_thread::sleep_for(100ms);
  }
}

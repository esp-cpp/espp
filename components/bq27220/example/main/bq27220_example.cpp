#include <chrono>
#include <vector>

#include "bq27220.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  //! [bq27220 example]
  espp::Logger logger({.tag = "Bq27220 example", .level = espp::Logger::Verbosity::INFO});
  // make the I2C that we'll use to communicate
  logger.info("initializing i2c driver...");
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });
  std::error_code ec;
  auto bq27220_device =
      i2c.add_device<uint8_t>({.device_address = espp::Bq27220::DEFAULT_ADDRESS,
                               .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                               .scl_speed_hz = i2c.config().clk_speed,
                               .log_level = espp::Logger::Verbosity::WARN},
                              ec);
  if (!bq27220_device) {
    logger.error("BQ27220 I2C device initialization failed: {}", ec.message());
    return;
  }
  // now make the bq27220 which handles the fuel gauge
  espp::Bq27220 bq27220({.write = espp::make_i2c_addressed_write(bq27220_device),
                         .read = espp::make_i2c_addressed_read(bq27220_device),
                         .log_level = espp::Logger::Verbosity::WARN});

  // and finally, make the task to periodically poll the bq27220 and print
  // the state.
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 1s);
    }
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    auto voltage = bq27220.get_voltage_mv(ec);
    if (ec) {
      return false;
    }
    auto current = bq27220.get_current_ma(ec);
    if (ec) {
      return false;
    }
    auto soc = bq27220.get_state_of_charge(ec);
    if (ec) {
      return false;
    }
    auto temperature = bq27220.get_temperature_celsius(ec);
    if (ec) {
      return false;
    }
    fmt::print("{:0.2f}, {}, {}, {}, {:0.2f}\n", seconds, voltage, current, soc, temperature);
    // don't want to stop the task
    return false;
  };
  auto task = espp::Task({.callback = task_fn,
                          .task_config =
                              {
                                  .name = "Bq27220 Task",
                                  .stack_size_bytes = 5 * 1024,
                              },
                          .log_level = espp::Logger::Verbosity::WARN});
  fmt::print("%time(s), voltage (mV), current (mA), SoC (%), Temperature (C)\n");
  task.start();
  //! [bq27220 example]
  while (true) {
    std::this_thread::sleep_for(100ms);
  }
}

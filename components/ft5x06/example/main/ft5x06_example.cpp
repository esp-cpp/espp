#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "ft5x06.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting ft5x06 example\n");
    //! [ft5x06 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });
    std::error_code ec;
    auto ft5x06_device =
        i2c.add_device<uint8_t>({.device_address = espp::Ft5x06::DEFAULT_ADDRESS,
                                 .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                                 .scl_speed_hz = i2c.config().clk_speed,
                                 .log_level = espp::Logger::Verbosity::WARN},
                                ec);
    if (!ft5x06_device) {
      fmt::print("ft5x06 I2C device initialization failed: {}\n", ec.message());
      return;
    }
    // now make the ft5x06 which decodes the data
    espp::Ft5x06 ft5x06({.write = espp::make_i2c_addressed_write(ft5x06_device),
                         .read_register = espp::make_i2c_addressed_read_register(ft5x06_device),
                         .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the ft5x06 and print
    // the state
    auto task_fn = [&ft5x06](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // update the cached touch state
      bool new_data = ft5x06.update(ec);
      if (ec) {
        fmt::print("Could not update state\n");
        return false;
      }
      if (!new_data) {
        return false;
      }
      auto state = ft5x06.touch_state();
      auto point = state.primary_point();
      uint8_t num_touch_points = state.num_touch_points;
      uint16_t x = point.x;
      uint16_t y = point.y;
      fmt::print("num_touch_points: {}, x: {}, y: {}\n", num_touch_points, x, y);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Ft5x06 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [ft5x06 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Ft5x06 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

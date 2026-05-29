#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "cst816.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting cst816 example\n");
    //! [cst816 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .timeout_ms = 100,
        .clk_speed = 400 * 1000,
    });

    bool has_cst816 = i2c.probe_device(espp::Cst816::DEFAULT_ADDRESS);
    fmt::print("Touchpad probe: {}\n", has_cst816);
    std::error_code ec;
    auto cst816_device =
        i2c.add_device<uint8_t>({.device_address = espp::Cst816::DEFAULT_ADDRESS,
                                 .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                                 .scl_speed_hz = i2c.config().clk_speed,
                                 .log_level = espp::Logger::Verbosity::WARN},
                                ec);
    if (!cst816_device) {
      fmt::print("CST816 I2C device initialization failed: {}\n", ec.message());
      return;
    }

    // now make the cst816 which decodes the data
    espp::Cst816 cst816({.write = espp::make_i2c_addressed_write(cst816_device),
                         .read = espp::make_i2c_addressed_read(cst816_device),
                         .log_level = espp::Logger::Verbosity::WARN});

    // and finally, make the task to periodically poll the cst816 and print
    // the state
    auto task_fn = [&cst816](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // update the state
      bool new_data = cst816.update(ec);
      if (ec) {
        fmt::print("Could not update state\n");
        return false;
      }
      if (!new_data) {
        return false; // don't stop the task
      }
      // get the state
      bool home_pressed = false;
      home_pressed = cst816.get_home_button_state();
      fmt::print("home_pressed: {}\n", home_pressed);
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      cst816.get_touch_point(&num_touch_points, &x, &y);
      fmt::print("num_touch_points: {}, x: {}, y: {}\n", num_touch_points, x, y);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "Cst816 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [cst816 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Cst816 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

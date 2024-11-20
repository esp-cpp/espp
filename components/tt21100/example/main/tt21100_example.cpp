#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "task.hpp"
#include "tt21100.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting tt2100 example\n");
    //! [tt21100 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });
    // now make the tt21100
    auto tt21100 = espp::Tt21100({
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
    });

    // and finally, make the task to periodically poll the tt21100 and print
    // the state
    auto task_fn = [&tt21100](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      bool new_data = tt21100.update(ec);
      if (ec) {
        fmt::print("TT21100 update failed: {}\n", ec.message());
        return true; // stop the task
      }
      if (!new_data) {
        // no new data, so don't bother printing
        return false; // don't stop the task
      }
      // get the state
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      tt21100.get_touch_point(&num_touch_points, &x, &y);
      bool button_state = tt21100.get_home_button_state();
      if (num_touch_points == 0 && !button_state) {
        // no touch points, so don't bother printing
      } else if (num_touch_points == 0 && button_state) {
        fmt::print("home button pressed\n");
      } else {
        fmt::print("num_touch_points: {}, x: {}, y: {}, button state: {}\n", num_touch_points, x, y,
                   button_state);
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "TT21100 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [tt21100 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("tt21100 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

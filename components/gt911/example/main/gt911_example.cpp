#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "gt911.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting gt911 example\n");
    //! [gt911 example]
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

    bool has_gt911_5d = i2c.probe_device(0x5d);
    bool has_gt911_14 = i2c.probe_device(0x14);
    uint8_t address = has_gt911_5d ? 0x5d : 0x14;
    fmt::print("Touchpad probe: {}\n", has_gt911_5d || has_gt911_14);
    fmt::print("       address: {:#02x}\n", address);

    // now make the gt911 which decodes the data
    espp::Gt911 gt911({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3),
                       .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                         std::placeholders::_2, std::placeholders::_3),
                       .address = address,
                       .log_level = espp::Logger::Verbosity::WARN});

    // and finally, make the task to periodically poll the gt911 and print
    // the state
    auto task_fn = [&gt911](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // update the state
      bool new_data = gt911.update(ec);
      if (ec) {
        fmt::print("Could not update state\n");
        return false;
      }
      if (!new_data) {
        return false; // don't stop the task
      }
      // get the state
      bool home_pressed = false;
      home_pressed = gt911.get_home_button_state();
      fmt::print("home_pressed: {}\n", home_pressed);
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      gt911.get_touch_point(&num_touch_points, &x, &y);
      if (ec) {
        fmt::print("Could not get touch point\n");
        return false;
      }
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
                            .task_config = {.name = "Gt911 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [gt911 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Gt911 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

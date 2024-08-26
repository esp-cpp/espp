#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "chsc6x.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting chsc6x example\n");
    //! [chsc6x example]
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

    bool has_chsc6x = i2c.probe_device(espp::Chsc6x::DEFAULT_ADDRESS);
    fmt::print("Touchpad probe: {}\n", has_chsc6x);

    // now make the chsc6x which decodes the data
    espp::Chsc6x chsc6x({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                            std::placeholders::_2, std::placeholders::_3),
                         .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                           std::placeholders::_2, std::placeholders::_3),
                         .log_level = espp::Logger::Verbosity::WARN});

    // and finally, make the task to periodically poll the chsc6x and print
    // the state
    auto task_fn = [&chsc6x](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // update the state
      bool new_data = chsc6x.update(ec);
      if (ec) {
        fmt::print("Could not update state\n");
        return false;
      }
      if (!new_data) {
        return false; // don't stop the task
      }
      // get the state
      uint8_t num_touch_points = 0;
      uint16_t x = 0, y = 0;
      chsc6x.get_touch_point(&num_touch_points, &x, &y);
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
    auto task = espp::Task(
        {.name = "Chsc6x Task", .callback = task_fn, .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [chsc6x example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Chsc6x example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

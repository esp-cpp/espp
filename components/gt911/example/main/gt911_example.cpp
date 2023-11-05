#include <chrono>
#include <vector>

#include "gt911.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting gt911 example, press select & start together to quit!\n");
    //! [gt911 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_8,
    });
    // now make the gt911 which decodes the data
    espp::Gt911 gt911({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                          std::placeholders::_2, std::placeholders::_3),
                       .write_read = std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1,
                                               std::placeholders::_2, std::placeholders::_3,
                                               std::placeholders::_4, std::placeholders::_5),
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
      uint8_t num_touch_points;
      uint16_t x, y;
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
        cv.wait_for(lk, 500ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task(
        {.name = "Gt911 Task", .callback = task_fn, .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [gt911 example]
    while (!true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Gt911 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

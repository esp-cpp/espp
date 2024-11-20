#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "bm8563.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting tt2100 example\n");
    //! [bm8563 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });
    // now make the bm8563
    auto bm8563 = espp::Bm8563({
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .write_then_read =
            std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
    });

    std::error_code ec;

    // set the time
    espp::Bm8563::DateTime date_time = {.date =
                                            {
                                                .year = 2023,
                                                .month = 11,
                                                .weekday = 4,
                                                .day = 23,
                                            },
                                        .time = {.hour = 8, .minute = 15, .second = 30}};
    bm8563.set_date_time(date_time, ec);
    if (ec) {
      fmt::print("Error setting date time: {}\n", ec.message());
    } else {
      fmt::print("Date time set to {}\n", date_time);
    }

    // and finally, make the task to periodically poll the bm8563 and print
    // the state
    auto task_fn = [&bm8563](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // Get the date time and print it
      auto date_time = bm8563.get_date_time(ec);
      if (ec) {
        fmt::print("Error getting date time: {}\n", ec.message());
      } else {
        fmt::print("Date time is {}\n", date_time);
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 1s);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "BM8563 Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [bm8563 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("bm8563 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

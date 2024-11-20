#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "qwiicnes.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting qwiicnes example, press select & start together to quit!\n");
    //! [qwiicnes example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    // now make the qwiicnes which decodes the data
    espp::QwiicNes qwiicnes(
        {.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3),
         .read_register =
             std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
         .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the qwiicnes and print
    // the state
    auto task_fn = [&quit_test, &qwiicnes](std::mutex &m, std::condition_variable &cv) {
      static espp::QwiicNes::ButtonState last_button_state;
      std::error_code ec;
      qwiicnes.update(ec);
      if (ec) {
        fmt::print("QwiicNes update failed: {}\n", ec.message());
        return true; // stop the task
      }
      auto button_state = qwiicnes.get_button_state();
      if (button_state != last_button_state) {
        fmt::print("Button state: {}\n", button_state);
        quit_test = button_state.select && button_state.start;
        last_button_state = button_state;
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
                            .task_config = {.name = "Qwiicnes Task"},
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [qwiicnes example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Qwiicnes example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <chrono>
#include <vector>

#include <driver/gpio.h>

#include "i2c.hpp"
#include "t_keyboard.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting tkeyboard example\n");
    //! [tkeyboard example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_18,
        .scl_io_num = GPIO_NUM_8,
    });
    // now make the tkeyboard which decodes the data
    espp::TKeyboard tkeyboard({.write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1,
                                                  std::placeholders::_2, std::placeholders::_3),
                               .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1,
                                                 std::placeholders::_2, std::placeholders::_3),
                               .log_level = espp::Logger::Verbosity::WARN});

    // on the LilyGo T-Deck, the peripheral power control pin must be set high
    // to enable peripheral power
    auto power_ctrl = GPIO_NUM_10;
    gpio_set_direction(power_ctrl, GPIO_MODE_OUTPUT);
    gpio_set_level(power_ctrl, 1);

    do {
      fmt::print("Waiting for tkeyboard to boot up...\n");
      std::this_thread::sleep_for(500ms);
    } while (!i2c.probe_device(espp::TKeyboard::DEFAULT_ADDRESS));

    fmt::print("Tkeyboard ready!\n");

    // and finally, make the task to periodically poll the tkeyboard and print
    // the state
    auto task_fn = [&tkeyboard](std::mutex &m, std::condition_variable &cv) {
      std::error_code ec;
      // update the state
      char key = tkeyboard.get_key(ec);
      if (ec) {
        fmt::print("Could not get key\n");
        return false;
      }
      if (!key) {
        return false; // don't stop the task
      }
      fmt::print("Key: '{}'\n", key);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 5ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task({.name = "Tkeyboard Task",
                            .callback = task_fn,
                            .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [tkeyboard example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Tkeyboard example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

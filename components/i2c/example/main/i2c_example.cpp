#include <chrono>
#include <vector>

#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting ft5x06 example, press select & start together to quit!\n");
    //! [i2c example]
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_22, // qwiic sda on the qtpy
        .scl_io_num = GPIO_NUM_19, // qwiic scl on the qtpy
    });
    // finally, make the task to periodically query the bus
    auto task_fn = [&i2c](std::mutex &m, std::condition_variable &cv) {
      static constexpr uint8_t device_address = 0x58;
      static constexpr uint8_t register_address = 0x58;
      uint8_t read_data[1];
      bool success = i2c.read_at_register(device_address, register_address, read_data, 1);
      if (success) {
        fmt::print("read data: {:#04x}\n", read_data[0]);
      } else {
        fmt::print("read failed\n");
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      return false; // don't stop the task
    };
    auto task = espp::Task(
        {.name = "I2c Task", .callback = task_fn, .log_level = espp::Logger::Verbosity::WARN});
    task.start();
    //! [i2c example]
    while (!true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Ft5x06 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

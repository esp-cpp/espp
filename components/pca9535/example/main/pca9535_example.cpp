#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "pca9535.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting pca9535 example!\n");
    //! [pca9535 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    std::error_code ec;
    auto pca9535_device =
        i2c.add_device<uint8_t>({.device_address = espp::Pca9535::DEFAULT_ADDRESS,
                                 .timeout_ms = static_cast<int>(i2c.config().timeout_ms),
                                 .scl_speed_hz = i2c.config().clk_speed,
                                 .log_level = espp::Logger::Verbosity::WARN},
                                ec);
    if (!pca9535_device) {
      fmt::print("PCA9535 I2C device initialization failed: {}\n", ec.message());
      return;
    }
    // now make the pca9535 which handles GPIO. Port 0 is all inputs, and Port 1
    // is all outputs.
    espp::Pca9535 pca9535({.port_0_direction_mask = 0xFF, // all inputs on port 0
                           .port_1_direction_mask = 0x00, // all outputs on port 1
                           .write = espp::make_i2c_addressed_write(pca9535_device),
                           .read_register = espp::make_i2c_addressed_read_register(pca9535_device),
                           .log_level = espp::Logger::Verbosity::WARN});
    // and finally, make the task to periodically poll the pca9535 and print the
    // state. NOTE: the Pca9535 does not internally manage its own state update,
    // so whatever rate we use here is the rate at which the state will update.
    auto task_fn = [&pca9535](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      static uint8_t output = 0;
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      std::error_code ec;
      // read the inputs on port 0
      auto p0_pins = pca9535.get_pins(espp::Pca9535::Port::PORT0, ec);
      if (ec) {
        fmt::print("get_pins failed: {}\n", ec.message());
      } else {
        // toggle the output on port 1
        output = ~output;
        pca9535.set_pins(espp::Pca9535::Port::PORT1, output, ec);
        if (ec) {
          fmt::print("set_pins failed: {}\n", ec.message());
        } else {
          fmt::print("{:.3f}, {:#x}, {:#x}\n", seconds, p0_pins, output);
        }
      }
      // Always sleep (even after an error) so a persistently-failing device does
      // not spin this task in a tight loop. Sleeping this way also lets the sleep
      // exit early when the task is being stopped / destroyed.
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Pca9535 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("%time(s), port_0 pins, port_1 output\n");
    task.start();
    //! [pca9535 example]
    while (true) {
      std::this_thread::sleep_for(1s);
    }
  }
}

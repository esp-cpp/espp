#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "mcp23x17.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting mcp23x17 example, press button on B7 quit!\n");
    //! [mcp23x17 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    // now make the mcp23x17 which handles GPIO
    espp::Mcp23x17 mcp23x17(
        {.port_0_direction_mask = (1 << 0), // input on A0
         .port_0_interrupt_mask = (1 << 0), // interrupt on A0
         .port_1_direction_mask = (1 << 7), // input on B7
         .port_1_interrupt_mask = (1 << 7), // interrupt on B7
         .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                            std::placeholders::_3),
         .read_register =
             std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                       std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
         .log_level = espp::Logger::Verbosity::WARN});
    // set pull up on the input pins
    std::error_code ec;
    mcp23x17.set_pull_up(espp::Mcp23x17::Port::PORT0, (1 << 0), ec);
    if (ec) {
      fmt::print("set_pull_up failed: {}\n", ec.message());
    }
    mcp23x17.set_pull_up(espp::Mcp23x17::Port::PORT1, (1 << 7), ec);
    if (ec) {
      fmt::print("set_pull_up failed: {}\n", ec.message());
    }
    // and finally, make the task to periodically poll the mcp23x17 and print
    // the state. NOTE: the Mcp23x17 does not internally manage its own state
    // update, so whatever rate we use here is the rate at which the state will
    // update.
    auto task_fn = [&quit_test, &mcp23x17](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      std::error_code ec;
      auto a_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::PORT0, ec);
      if (ec) {
        fmt::print("get_pins failed: {}\n", ec.message());
        return false;
      }
      auto b_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::PORT1, ec);
      if (ec) {
        fmt::print("get_pins failed: {}\n", ec.message());
        return false;
      }
      bool on = !(a_pins & (1 << 0));
      if (on) {
        mcp23x17.set_pins(espp::Mcp23x17::Port::PORT1, (1 << 3), ec);
      } else {
        mcp23x17.set_pins(espp::Mcp23x17::Port::PORT1, 0x00, ec);
      }
      if (ec) {
        fmt::print("set_pins failed: {}\n", ec.message());
        return false;
      }
      fmt::print("{:.3f}, {:#x}, {:#x}\n", seconds, a_pins, b_pins);
      quit_test = !(b_pins & (1 << 7));
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config =
                                {
                                    .name = "Mcp23x17 Task",
                                    .stack_size_bytes = 5 * 1024,
                                },
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("%time(s), port_a pins, port_b pins\n");
    task.start();
    //! [mcp23x17 example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
  }

  fmt::print("Mcp23x17 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

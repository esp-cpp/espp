#include <chrono>
#include <sdkconfig.h>

#include "i2c.hpp"
#include "pi4ioe5v.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting PI4IOE5V example...\n");

  //! [pi4ioe5v_example]
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });

  // Configure PI4IOE5V: single 8-bit port as outputs
  espp::Pi4ioe5v exp({
      .device_address = espp::Pi4ioe5v::DEFAULT_ADDRESS,
      .direction_mask = 0x00, // all outputs
      .output_state = 0xFF,   // initial state
      .probe = std::bind(&espp::I2c::probe_device, &i2c, std::placeholders::_1),
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read_register =
          std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .write_then_read =
          std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  std::error_code ec;
  exp.initialize(ec);
  if (ec) {
    fmt::print("PI4IOE5V init failed: {}\n", ec.message());
    return;
  }

  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static uint8_t value = 0xAA;
    value ^= 0xFF; // toggle pattern
    exp.write_outputs(value, ec);
    if (!ec) {
      uint8_t inputs = exp.read_inputs(ec);
      if (!ec) {
        fmt::print("OUT=0x{:02X} IN=0x{:02X}\n", value, inputs);
      }
    }
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 500ms);
    }
    return false;
  };

  espp::Task task({.callback = task_fn,
                   .task_config = {.name = "PI4IOE5V Task", .stack_size_bytes = 4 * 1024},
                   .log_level = espp::Logger::Verbosity::WARN});
  task.start();
  //! [pi4ioe5v_example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

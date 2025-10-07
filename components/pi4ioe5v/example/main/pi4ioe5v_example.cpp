#include <chrono>
#include <sdkconfig.h>

#include "i2c.hpp"
#include "logger.hpp"
#include "pi4ioe5v.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  espp::Logger logger({.tag = "PI4IOE5V example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting PI4IOE5V example...");

  //! [pi4ioe5v_example]
  // make the I2C that we'll use to communicate
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
  });

  // Configure PI4IOE5V: 8-bit port with mixed inputs/outputs
  espp::Pi4ioe5v exp({
      .device_address = espp::Pi4ioe5v::DEFAULT_ADDRESS,
      .direction_mask = 0xF0, // upper 4 bits as outputs, lower 4 bits as inputs
      .interrupt_mask = 0xFF, // all interrupts disabled for now
      .initial_output = 0xA0, // initial pattern for outputs
      .write = std::bind_front(&espp::I2c::write, &i2c),
      .write_then_read = std::bind_front(&espp::I2c::write_read, &i2c),
      .auto_init = false,
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;
  // Initialize separately from the constructor since we set auto_init to false
  if (!exp.initialize(ec)) {
    logger.error("PI4IOE5V initialization failed: {}", ec.message());
    return;
  }

  // Configure pull resistors for input pins (lower 4 bits)
  exp.set_pull_resistor_for_pin(0xF0, espp::Pi4ioe5v::PullResistor::PULL_UP, ec);
  if (ec) {
    logger.error("Failed to configure pull resistors: {}", ec.message());
  }

  // Create task to periodically toggle outputs and read inputs
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    static uint8_t output_pattern = 0x50;

    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();

    // Toggle output pattern on upper 4 bits
    output_pattern ^= 0xF0;
    exp.output(output_pattern, ec);
    if (ec) {
      logger.error("Failed to set outputs: {}", ec.message());
      return true; // stop the task
    }

    // Read all pins
    auto pins = exp.get_pins(ec);
    if (ec) {
      logger.error("Failed to read pins: {}", ec.message());
      return true; // stop the task
    }

    // Read current output state
    auto outputs = exp.get_output(ec);
    if (ec) {
      logger.error("Failed to read outputs: {}", ec.message());
      return true; // stop the task
    }

    fmt::print("{:.3f}, inputs: {:#04x}, outputs: {:#04x}\n", seconds, pins, outputs);

    // NOTE: sleeping in this way allows the sleep to exit early when the
    // task is being stopped / destroyed
    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 500ms);
    }
    // don't want to stop the task
    return false;
  };

  auto task = espp::Task({.callback = task_fn,
                          .task_config = {.name = "PI4IOE5V Task", .stack_size_bytes = 4 * 1024},
                          .log_level = espp::Logger::Verbosity::WARN});
  fmt::print("% time(s), inputs (lower 4 bits), outputs (upper 4 bits)\n");
  task.start();
  //! [pi4ioe5v_example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

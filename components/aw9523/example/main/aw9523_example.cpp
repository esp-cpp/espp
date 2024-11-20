#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "aw9523.hpp"
#include "i2c.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {

  fmt::print("Starting aw9523 example, controls:\n"
             "\tP0_0 -> increase P1_5 brightness\n"
             "\tP0_1 -> increase P1_6 brightness\n"
             "\tP0_2 -> decrease P1_5 brightness\n"
             "\tP0_3 -> decrease P1_6 brightness\n"
             "\tP1_0 -> decrease P1_7 brightness\n"
             "\tP1_1 -> increase P1_7 brightness\n"
             "\n");
  //! [aw9523 example]
  // make the I2C that we'll use to communicate
  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });
  // now make the aw9523 which handles GPIO
  espp::Aw9523 aw9523(
      {// since this uses LEDs, both of the address pins are pulled up in
       // hardware to have the LEDs default to off
       .device_address = espp::Aw9523::DEFAULT_ADDRESS | 0b11,
       // set P0_0 - P0_5 to be inputs
       .port_0_direction_mask = 0b00111111,
       // set P1_0 - P1_1 to be inputs
       .port_1_direction_mask = 0b00000011,
       .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
       .write_then_read =
           std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                     std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
       .log_level = espp::Logger::Verbosity::WARN});
  std::error_code ec;
  aw9523.initialize(ec); // Initialized separately from the constructor.
  if (ec) {
    fmt::print("aw9523 initialization failed: {}\n", ec.message());
    return;
  }
  // set P1_5, P1_6, and P1_7 to be leds
  int r_led = (1 << 13);
  int g_led = (1 << 14);
  int b_led = (1 << 15);
  // for the port led mask, 0 = LED, 1 = GPIO so we invert the pins above
  uint16_t leds = ~(r_led | g_led | b_led);
  aw9523.configure_led(leds, ec);
  if (ec) {
    fmt::print("aw9523 led configuration failed: {}\n", ec.message());
    return;
  }
  // and finally, make the task to periodically poll the aw9523 and print
  // the state. NOTE: the Aw9523 does not internally manage its own state
  // update, so whatever rate we use here is the rate at which the state will
  // update.
  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    static auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto seconds = std::chrono::duration<float>(now - start).count();
    static uint8_t r_brightness = 0;
    static uint8_t g_brightness = 0;
    static uint8_t b_brightness = 0;
    // returns the pins as P0_0 lsb, P1_7 msb
    auto pins = aw9523.get_pins(ec);
    if (ec) {
      fmt::print("aw9523 get pins failed: {}\n", ec.message());
      return true; // stop the task
    }
    // equivalent to:
    // auto pins = (aw9523.get_pins(espp::Aw9523::Port::PORT1, ec) << 8) |
    // aw9523.get_pins(espp::Aw9523::Port::PORT0, ec);
    int r_up = (pins & (1 << 0));   // P0_0
    int g_up = (pins & (1 << 1));   // P0_1
    int b_up = (pins & (1 << 9));   // P1_0
    int r_down = (pins & (1 << 2)); // P0_2
    int g_down = (pins & (1 << 3)); // P0_3
    int b_down = (pins & (1 << 8)); // P1_8
    // use the buttons to modify the brightness values (with wrap)
    static uint8_t increment = 1;
    if (r_up) {
      r_brightness += increment;
    } else if (r_down) {
      r_brightness -= increment;
    }
    if (g_up) {
      g_brightness += increment;
    } else if (g_down) {
      g_brightness -= increment;
    }
    if (b_up) {
      b_brightness += increment;
    } else if (b_down) {
      b_brightness -= increment;
    }
    aw9523.led(r_led, r_brightness, ec);
    aw9523.led(g_led, g_brightness, ec);
    aw9523.led(b_led, b_brightness, ec);
    if (ec) {
      fmt::print("aw9523 led failed: {}\n", ec.message());
      return true; // stop the task
    }
    fmt::print("{:.3f}, {:#x}, {}, {}, {}\n", seconds, pins, r_brightness, g_brightness,
               b_brightness);
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
                                  .name = "Aw9523 Task",
                                  .stack_size_bytes = 5 * 1024,
                              },
                          .log_level = espp::Logger::Verbosity::WARN});
  fmt::print("%time(s), pin values, r, g, b\n");
  task.start();
  //! [aw9523 example]
  while (true) {
    std::this_thread::sleep_for(100ms);
  }

  fmt::print("Aw9523 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

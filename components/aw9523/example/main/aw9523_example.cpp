#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "aw9523.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

static constexpr auto I2C_NUM = I2C_NUM_1;
static constexpr auto I2C_SCL_IO = GPIO_NUM_19;
static constexpr auto I2C_SDA_IO = GPIO_NUM_22;
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = 10;

extern "C" void app_main(void) {
  {
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
    i2c_config_t i2c_cfg;
    fmt::print("initializing i2c driver...\n");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the aw9523
    auto aw9523_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                            I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        fmt::print("I2C WRITE ERROR: '{}'\n", esp_err_to_name(err));
      }
    };

    auto aw9523_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr, 1, data, data_len,
                                              I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        fmt::print("I2C READ ERROR: '{}'\n", esp_err_to_name(err));
      }
    };
    // now make the aw9523 which handles GPIO
    espp::Aw9523 aw9523({// since this uses LEDs, both of the address pins are pulled up in
                         // hardware to have the LEDs default to off
                         .device_address = espp::Aw9523::DEFAULT_ADDRESS | 0b11,
                         // set P0_0 - P0_5 to be inputs
                         .port_0_direction_mask = 0b00111111,
                         // set P1_0 - P1_1 to be inputs
                         .port_1_direction_mask = 0b00000011,
                         .write = aw9523_write,
                         .read = aw9523_read,
                         .log_level = espp::Logger::Verbosity::WARN});
    // set P1_5, P1_6, and P1_7 to be leds
    int r_led = (1 << 13);
    int g_led = (1 << 14);
    int b_led = (1 << 15);
    // for the port led mask, 0 = LED, 1 = GPIO so we invert the pins above
    uint16_t leds = ~(r_led | g_led | b_led);
    aw9523.configure_led(leds);
    // and finally, make the task to periodically poll the aw9523 and print
    // the state. NOTE: the Aw9523 does not internally manage its own state
    // update, so whatever rate we use here is the rate at which the state will
    // update.
    auto task_fn = [&aw9523, &r_led, &g_led, &b_led](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      static uint8_t r_brightness = 0;
      static uint8_t g_brightness = 0;
      static uint8_t b_brightness = 0;
      // returns the pins as P0_0 lsb, P1_7 msb
      auto pins = aw9523.get_pins();
      // equivalent to:
      // auto pins = (aw9523.get_pins(espp::Aw9523::Port::PORT1) << 8) |
      // aw9523.get_pins(espp::Aw9523::Port::PORT0);
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
      aw9523.led(r_led, r_brightness);
      aw9523.led(g_led, g_brightness);
      aw9523.led(b_led, b_brightness);
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
    auto task = espp::Task({.name = "Aw9523 Task",
                            .callback = task_fn,
                            .stack_size_bytes = 5 * 1024,
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("%time(s), pin values, r, g, b\n");
    task.start();
    //! [aw9523 example]
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
    // now clean up the i2c driver
    i2c_driver_delete(I2C_NUM);
  }

  fmt::print("Aw9523 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "mcp23x17.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

#define I2C_NUM (I2C_NUM_1)
#define I2C_SCL_IO (GPIO_NUM_40)
#define I2C_SDA_IO (GPIO_NUM_41)
#define I2C_FREQ_HZ (400 * 1000)
#define I2C_TIMEOUT_MS (10)

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting mcp23x17 example, press button on B7 quit!\n");
    //! [mcp23x17 example]
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
    // make some lambda functions we'll use to read/write to the mcp23x17
    auto mcp23x17_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto mcp23x17_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr, 1, data, data_len,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    // now make the mcp23x17 which handles GPIO
    espp::Mcp23x17 mcp23x17({.port_a_direction_mask = (1 << 0), // input on A0
                             .port_a_interrupt_mask = (1 << 0), // interrupt on A0
                             .port_b_direction_mask = (1 << 7), // input on B7
                             .port_b_interrupt_mask = (1 << 7), // interrupt on B7
                             .write = mcp23x17_write,
                             .read = mcp23x17_read,
                             .log_level = espp::Logger::Verbosity::WARN});
    // set pull up on the input pins
    mcp23x17.set_pull_up(espp::Mcp23x17::Port::A, (1 << 0));
    mcp23x17.set_pull_up(espp::Mcp23x17::Port::B, (1 << 7));
    // and finally, make the task to periodically poll the mcp23x17 and print
    // the state. NOTE: the Mcp23x17 does not internally manage its own state
    // update, so whatever rate we use here is the rate at which the state will
    // update.
    auto task_fn = [&quit_test, &mcp23x17](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now - start).count();
      auto a_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::A);
      auto b_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::B);
      bool on = !(a_pins & (1 << 0));
      if (on) {
        mcp23x17.set_pins(espp::Mcp23x17::Port::B, (1 << 3));
      } else {
        mcp23x17.set_pins(espp::Mcp23x17::Port::B, 0x00);
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
    auto task = espp::Task({.name = "Mcp23x17 Task",
                            .callback = task_fn,
                            .stack_size_bytes = 5 * 1024,
                            .log_level = espp::Logger::Verbosity::WARN});
    fmt::print("%time(s), port_a pins, port_b pins\n");
    task.start();
    //! [mcp23x17 example]
    while (!quit_test) {
      std::this_thread::sleep_for(100ms);
    }
    // now clean up the i2c driver
    i2c_driver_delete(I2C_NUM);
  }

  fmt::print("Mcp23x17 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "task.hpp"
#include "aw9523.hpp"

using namespace std::chrono_literals;

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_40)
#define I2C_SDA_IO      (GPIO_NUM_41)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

extern "C" void app_main(void) {
  {
    std::atomic<bool> quit_test = false;
    fmt::print("Starting aw9523 example, press button on B7 quit!\n");
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
    if (err != ESP_OK) printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0);
    if (err != ESP_OK) printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the aw9523
    auto aw9523_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_to_device(I2C_NUM,
                                 dev_addr,
                                 data,
                                 data_len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };

    auto aw9523_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
      i2c_master_write_read_device(I2C_NUM,
                                   dev_addr,
                                   &reg_addr,
                                   1,
                                   data,
                                   data_len,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    // now make the aw9523 which handles GPIO
    espp::Aw9523 aw9523({
        .port_0_direction_mask = (1 << 0),
        .port_0_interrupt_mask = (1 << 0),
        .port_1_direction_mask = (1 << 7),
        .port_1_interrupt_mask = (1 << 7),
        .write = aw9523_write,
        .read = aw9523_read,
        .log_level = espp::Logger::Verbosity::WARN
      });
    // and finally, make the task to periodically poll the aw9523 and print
    // the state. NOTE: the Aw9523 does not internally manage its own state
    // update, so whatever rate we use here is the rate at which the state will
    // update.
    auto task_fn = [&quit_test, &aw9523](std::mutex& m, std::condition_variable& cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds = std::chrono::duration<float>(now-start).count();
      auto pins = aw9523.get_pins();
      bool on = !(pins & (1 << 0));
      if (on) {
        aw9523.set_pins(espp::Aw9523::Port::PORT1, (1 << 3));
      } else {
        aw9523.set_pins(espp::Aw9523::Port::PORT1, 0x00);
      }
      fmt::print("{:.3f}, {:#x}\n",
                 seconds,
                 pins
                 );
      quit_test = !(pins & (1<<15));
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 50ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({
        .name = "Aw9523 Task",
        .callback = task_fn,
        .stack_size_bytes = 5*1024,
        .log_level = espp::Logger::Verbosity::WARN
      });
    fmt::print("%time(s), port_a pins, port_b pins\n");
    task.start();
    //! [aw9523 example]
    while (!quit_test) {
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

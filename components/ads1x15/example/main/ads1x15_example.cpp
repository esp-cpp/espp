#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "ads1x15.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_40)
#define I2C_SDA_IO      (GPIO_NUM_41)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

extern "C" void app_main(void) {
  float num_seconds_to_run = 5.0f;
  // This example shows using the i2c adc (ads1x15)
  {
    fmt::print("Running i2c adc example for {} seconds!\n", num_seconds_to_run);
    //! [ads1x15 example]
    // make the I2C that we'll use to communicate
    i2c_config_t i2c_cfg;
    fmt::print("initializing i2c driver...\n");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO; // pin 3 on the joybonnet
    i2c_cfg.scl_io_num = I2C_SCL_IO; // pin 5 on the joybonnet
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK) printf("config i2c failed\n");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0);
    if (err != ESP_OK) printf("install i2c driver failed\n");
    // make some lambda functions we'll use to read/write to the i2c adc
    auto ads_write = [](uint8_t reg_addr, uint16_t value) {
      uint8_t write_buf[3] = {reg_addr, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
      i2c_master_write_to_device(I2C_NUM,
                                 espp::Ads1x15::ADDRESS,
                                 write_buf,
                                 3,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    auto ads_read = [](uint8_t reg_addr) -> uint16_t {
      uint8_t read_data[2];
      i2c_master_write_read_device(I2C_NUM,
                                   espp::Ads1x15::ADDRESS,
                                   &reg_addr,
                                   1, // size of addr
                                   read_data,
                                   2,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      return (read_data[0] << 8) | read_data[1];
    };
    // make the actual ads class
    espp::Ads1x15 ads(espp::Ads1x15::Ads1015Config{
        .write = ads_write,
        .read = ads_read
      });
    // make the task which will get the raw data from the I2C ADC
    auto ads_read_task_fn = [&ads](std::mutex& m, std::condition_variable& cv) {
      auto x_mv = ads.sample_mv(1); // channel 1
      auto y_mv = ads.sample_mv(0); // channel 0
      fmt::print("X: {:.3f}, Y: {:.3f}\n", x_mv, y_mv);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 200ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto ads_task = espp::Task::make_unique({
        .name = "ADS",
        .callback = ads_read_task_fn,
        .stack_size_bytes{4*1024},
        .log_level = espp::Logger::Verbosity::INFO
      });
    ads_task->start();
    //! [ads1x15 example]
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<float>(now-start).count();
    while (elapsed < num_seconds_to_run) {
      std::this_thread::sleep_for(100ms);
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now-start).count();
    }
  }
  // now clean up the i2c driver (by now the task will have stopped, because we
  // left its scope.
  i2c_driver_delete(I2C_NUM);

  fmt::print("ADS1x15 example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

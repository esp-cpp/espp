#include <chrono>
#include <vector>

#include "driver/i2c.h"

#include "drv2605.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

#define I2C_NUM (I2C_NUM_1)
#define I2C_SCL_IO (GPIO_NUM_19)
#define I2C_SDA_IO (GPIO_NUM_22)
#define I2C_FREQ_HZ (400 * 1000)
#define I2C_TIMEOUT_MS (10)

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "drv2605 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c haptic motor driver (drv2605)
  {
    logger.info("Running i2c adc example!");
    //! [drv2605 example]
    // make the I2C that we'll use to communicate
    i2c_config_t i2c_cfg;
    logger.info("initializing i2c driver...");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      logger.error("config i2c failed");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      logger.error("install i2c driver failed");
    // make some lambda functions we'll use to read/write to the i2c adc
    auto i2c_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                            I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      return err == ESP_OK;
    };
    auto i2c_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_read_device(I2C_NUM, dev_addr, &reg_addr,
                                              1, // size of addr
                                              data, data_len, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      return err == ESP_OK;
    };
    // make the actual drv2605 class
    espp::Drv2605 drv2605(espp::Drv2605::Config{.device_address = espp::Drv2605::DEFAULT_ADDRESS,
                                                .write = i2c_write,
                                                .read = i2c_read,
                                                .motor_type = espp::Drv2605::MotorType::LRA});
    std::error_code ec;
    // we're using an ERM motor, so select an ERM library (1-5).
    // drv2605.select_library(1, ec);
    // we're using an LRA motor, so select an LRA library (6).
    drv2605.select_library(6, ec);
    if (ec) {
      logger.error("select library failed: {}", ec.message());
    }
    // make the task which will cycle through all the waveforms
    auto task_fn = [&drv2605](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();
      static uint8_t waveform = 0;
      std::error_code ec;
      drv2605.set_waveform(0, (espp::Drv2605::Waveform)waveform, ec);
      if (ec) {
        logger.error("set waveform failed: {}", ec.message());
        return false;
      }
      drv2605.set_waveform(1, espp::Drv2605::Waveform::END, ec);
      if (ec) {
        logger.error("set waveform failed: {}", ec.message());
        return false;
      }
      drv2605.start(ec);
      if (ec) {
        logger.error("start failed: {}", ec.message());
        return false;
      }
      waveform++;
      if (waveform > (uint8_t)espp::Drv2605::Waveform::MAX) {
        waveform = 0;
      }
      logger.info("{:.3f}, {}", elapsed, waveform);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 1s);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task::make_unique({.name = "example",
                                         .callback = task_fn,
                                         .stack_size_bytes{4 * 1024},
                                         .log_level = espp::Logger::Verbosity::INFO});
    task->start();
    //! [drv2605 example]
    logger.info("%time (s), waveform");
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }
  // now clean up the i2c driver (by now the task will have stopped, because we
  // left its scope.
  i2c_driver_delete(I2C_NUM);

  logger.info("Drv2605 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

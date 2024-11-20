#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "drv2605.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "drv2605 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c haptic motor driver (drv2605)
  {
    logger.info("Running i2c adc example!");
    //! [drv2605 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
    // make the actual drv2605 class
    espp::Drv2605 drv2605(espp::Drv2605::Config{
        .device_address = espp::Drv2605::DEFAULT_ADDRESS,
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read_register =
            std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
        .motor_type = espp::Drv2605::MotorType::LRA});
    std::error_code ec;
    // we're using an ERM motor, so select an ERM library (1-5).
    // drv2605.select_library(1, ec);
    // we're using an LRA motor, so select an LRA library (6).
    drv2605.select_library(espp::Drv2605::Library::LRA, ec);
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
      drv2605.stop(ec);
      if (ec) {
        logger.error("stop failed: {}", ec.message());
        return false;
      }
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
    auto task = espp::Task::make_unique({.callback = task_fn,
                                         .task_config =
                                             {
                                                 .name = "example",
                                                 .stack_size_bytes{4 * 1024},
                                             },
                                         .log_level = espp::Logger::Verbosity::INFO});
    task->start();
    //! [drv2605 example]
    logger.info("%time (s), waveform");
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("Drv2605 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

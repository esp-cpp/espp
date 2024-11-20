#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "ads1x15.hpp"
#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "ads1015 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (ads1x15)
  {
    logger.info("Running i2c adc example!");
    //! [ads1x15 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_1,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO, // pin 3 on the joybonnet
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO, // pin 5 on the joybonnet
    });
    // make the actual ads class
    espp::Ads1x15 ads(espp::Ads1x15::Ads1015Config{
        .device_address = espp::Ads1x15::DEFAULT_ADDRESS,
        .write = [&i2c](uint8_t addr, const uint8_t *data,
                        size_t len) { return i2c.write(addr, data, len); },
        .read = [&i2c](uint8_t addr, uint8_t *data,
                       size_t len) { return i2c.read(addr, data, len); },
    });
    // make the task which will get the raw data from the I2C ADC
    auto ads_read_task_fn = [&ads](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();
      std::error_code ec;
      auto x_mv = ads.sample_mv(1, ec); // channel 1
      if (ec) {
        logger.error("error reading channel 1: {}", ec.message());
        return false;
      }
      auto y_mv = ads.sample_mv(0, ec); // channel 0
      if (ec) {
        logger.error("error reading channel 0: {}", ec.message());
        return false;
      }
      logger.info("{:.3f}, {:.3f}, {:.3f}", elapsed, x_mv, y_mv);
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
    auto ads_task = espp::Task::make_unique({.callback = ads_read_task_fn,
                                             .task_config =
                                                 {
                                                     .name = "ADS",
                                                     .stack_size_bytes{4 * 1024},
                                                 },
                                             .log_level = espp::Logger::Verbosity::INFO});
    ads_task->start();
    //! [ads1x15 example]
    logger.info("%time (s), x, y");
    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("ADS1x15 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

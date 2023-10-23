#include <chrono>
#include <vector>

#include "driver/gpio.h"
#include "driver/i2c.h"

#include "logger.hpp"
#include "task.hpp"
#include "tla2528.hpp"

using namespace std::chrono_literals;

// This example is designed to be run on a QtPy ESP32
static constexpr auto I2C_NUM = (I2C_NUM_1);
static constexpr auto I2C_SCL_IO = (GPIO_NUM_19);
static constexpr auto I2C_SDA_IO = (GPIO_NUM_22);
static constexpr auto I2C_FREQ_HZ = (400 * 1000);
static constexpr auto I2C_TIMEOUT_MS = (10);

static QueueHandle_t gpio_evt_queue;

static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "tla2528 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (tla2528)
  {
    logger.info("Starting example!");

    //! [tla2528 example]
    // make the I2C that we'll use to communicate
    i2c_config_t i2c_cfg;
    logger.info("initializing i2c driver...");
    memset(&i2c_cfg, 0, sizeof(i2c_cfg));
    i2c_cfg.sda_io_num = I2C_SDA_IO;
    i2c_cfg.scl_io_num = I2C_SCL_IO;
    i2c_cfg.mode = I2C_MODE_MASTER;
    i2c_cfg.sda_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_cfg.scl_pullup_en = GPIO_PULLUP_DISABLE;
    i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
    auto err = i2c_param_config(I2C_NUM, &i2c_cfg);
    if (err != ESP_OK)
      logger.error("config i2c failed");
    err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK)
      logger.error("install i2c driver failed");

    // make some lambda functions we'll use to read/write to the i2c adc
    auto tla_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_write_to_device(I2C_NUM, dev_addr, data, data_len,
                                            I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C WRITE ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };
    auto tla_read = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
      auto err = i2c_master_read_from_device(I2C_NUM, dev_addr, data, data_len,
                                             I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      if (err != ESP_OK) {
        logger.error("I2C READ ERROR: to {:#04x} '{}'", dev_addr, esp_err_to_name(err));
      }
    };

    // make the actual tla class
    espp::Tla2528 tla(espp::Tla2528::Config{
        // Address pin is connected via 11k to ADC_DECAP, so the default address
        // of 0x10 becomes 0x16
        .device_address = espp::Tla2528::DEFAULT_ADDRESS | 0x06,
        .mode = espp::Tla2528::Mode::AUTO_SEQ,
        .analog_inputs = {espp::Tla2528::Channel::CH6, espp::Tla2528::Channel::CH7},
        .digital_inputs = {},
        .digital_outputs = {},
        // enable oversampling / averaging
        .oversampling_ratio = espp::Tla2528::OversamplingRatio::OSR_32,
        .write = tla_write,
        .read = tla_read,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // set the digital output drive mode to open-drain
    tla.set_digital_output_mode(espp::Tla2528::Channel::CH7, espp::Tla2528::OutputMode::OPEN_DRAIN);

    // make the task which will get the raw data from the I2C ADC
    fmt::print("%time (s), x (mV), y (mV)\n");
    auto tla_read_task_fn = [&tla](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();

      // get the analog input data
      auto all_mv = tla.get_all_mv();
      auto y_mv = all_mv[0]; // the first channel is channel 6 (Y axis)
      auto x_mv = all_mv[1]; // the second channel is channel 7 (X axis)

      // // alternatively we could get the analog data in a map
      // auto mapped_mv = tla.get_all_mv_map();
      // x_mv = mapped_mv[espp::Tla2528::Channel::CH7];
      // y_mv = mapped_mv[espp::Tla2528::Channel::CH6];

      // use fmt to print so it doesn't have the prefix and can be used more
      // easily as CSV (for plotting using uart_serial_plotter)
      fmt::print("{:.3f}, {:.3f}, {:.3f}\n", elapsed, x_mv, y_mv);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        using namespace std::chrono_literals;
        std::unique_lock<std::mutex> lk(m);
        cv.wait_until(lk, now + 10ms);
      }
      // we don't want to stop, so return false
      return false;
    };

    auto tla_task = espp::Task::make_unique({.name = "TLA",
                                             .callback = tla_read_task_fn,
                                             .stack_size_bytes{8 * 1024},
                                             .log_level = espp::Logger::Verbosity::INFO});
    tla_task->start();
    //! [tla2528 example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }
  // now clean up the i2c driver (by now the task will have stopped, because we
  // left its scope.
  i2c_driver_delete(I2C_NUM);

  logger.info("TLA2528 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

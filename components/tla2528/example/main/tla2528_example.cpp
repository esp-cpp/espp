#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "logger.hpp"
#include "task.hpp"
#include "tla2528.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "tla2528 example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the i2c adc (tla2528)
  {
    logger.info("Starting example!");

    //! [tla2528 example]
    // make the I2C that we'll use to communicate
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });

    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 0; address < 128; address++) {
      logger.debug("probing address {:#02x}", address);
      if (i2c.probe_device(address)) {
        found_addresses.push_back(address);
      }
    }
    // print out the addresses that were found
    logger.info("Found devices at addresses: {::#02x}", found_addresses);

    if (found_addresses.size() == 0) {
      logger.error("No devices found!");
      return;
    }

    // make sure one of the TLA addresses is in the list, and use one if it was found
    //
    // NOTE: the TLA2528 has 8 addresses, so we need to check for them all
    // 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17
    //
    // Example: Address pin is connected via 11k to ADC_DECAP, so the default
    // address of 0x10 becomes 0x16: espp::Tla2528::DEFAULT_ADDRESS | 0x06
    const uint8_t tla_addresses[] = {0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    bool found = false;
    uint8_t tla_address = 0;
    for (auto address : tla_addresses) {
      if (std::find(found_addresses.begin(), found_addresses.end(), address) !=
          found_addresses.end()) {
        tla_address = address;
        found = true;
        break;
      }
    }

    if (!found) {
      logger.error("No TLA2528 found!");
      return;
    }

    logger.info("Found TLA2528 at address {:#02x}", tla_address);

    static std::vector<espp::Tla2528::Channel> channels = {
        espp::Tla2528::Channel::CH0, espp::Tla2528::Channel::CH1, espp::Tla2528::Channel::CH2,
        espp::Tla2528::Channel::CH3, espp::Tla2528::Channel::CH4, espp::Tla2528::Channel::CH5,
        espp::Tla2528::Channel::CH6, espp::Tla2528::Channel::CH7};

    // make the actual tla class
    espp::Tla2528 tla(espp::Tla2528::Config{
        .device_address = tla_address,
        .mode = espp::Tla2528::Mode::AUTO_SEQ,
        .analog_inputs = channels,
        .digital_inputs = {},
        .digital_outputs = {},
        // enable oversampling / averaging
        .oversampling_ratio = espp::Tla2528::OversamplingRatio::NONE,
        .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                           std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                          std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // calibrate the ADC
    //
    // NOTE: this is not strictly necessary, but improves the accuracy of the
    //       ADC.
    auto cal_start_us = esp_timer_get_time();
    std::error_code ec;
    tla.calibrate(ec);
    auto cal_end_us = esp_timer_get_time();
    if (ec) {
      logger.error("error calibrating TLA2528: {}", ec.message());
      return;
    }
    logger.info("calibration took {} us", cal_end_us - cal_start_us);

    // make the task which will get the raw data from the I2C ADC
    fmt::print("%time (s), ntc (mV), x (mV), y (mV)\n");
    auto tla_read_task_fn = [&tla](std::mutex &m, std::condition_variable &cv) {
      static auto start = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - start).count();

      // get the analog input data individually; NOTE: this only works if you have configured the
      // TLA2528 to use MANUAL mode
      // auto ch0_mv = tla.get_mv(channels[0]);
      // auto ch1_mv = tla.get_mv(channels[1]);
      // auto ch2_mv = tla.get_mv(channels[2]);

      // Could also read them all at once; NOTE: this only works if you have configured the
      // TLA2528 to use AUTO_SEQ mode (which is more efficient)
      // auto all_mv = tla.get_all_mv();
      // auto ch0_mv = all_mv[0];
      // auto ch1_mv = all_mv[1];
      // auto ch2_mv = all_mv[2];

      // Could also use the mapped version; NOTE: this only works if you have configured the
      // TLA2528 to use AUTO_SEQ mode (which is more efficient)
      std::error_code ec;
      auto all_mv_map = tla.get_all_mv_map(ec);
      if (ec) {
        logger.error("error reading TLA2528: {}", ec.message());
        return false;
      }

      // use fmt to print so it doesn't have the prefix and can be used more
      // easily as CSV (for plotting using uart_serial_plotter)
      fmt::print("{:.3f}", elapsed);
      for (auto &[ch, mv] : all_mv_map) {
        fmt::print(", {:.1f}", mv);
      }
      fmt::print("\n");

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

    auto tla_task = espp::Task::make_unique({.callback = tla_read_task_fn,
                                             .task_config =
                                                 {
                                                     .name = "TLA",
                                                     .stack_size_bytes{8 * 1024},
                                                 },
                                             .log_level = espp::Logger::Verbosity::INFO});
    tla_task->start();
    //! [tla2528 example]

    while (true) {
      std::this_thread::sleep_for(100ms);
    }
  }

  logger.info("TLA2528 example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

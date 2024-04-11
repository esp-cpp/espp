#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "logger.hpp"

#include "i2c_menu.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "I2C Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

  {
#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
    //! [i2c menu example]
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
        .log_level = espp::Logger::Verbosity::INFO,
    });
    // now make a menu for it
    espp::I2cMenu i2c_menu(i2c);
    cli::Cli cli(i2c_menu.get());
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });
    espp::Cli input(cli);
    input.SetInputHistorySize(10);

    input.Start(); // As this is in the primary thread, we hold here until cli
                   // is complete. This is a blocking call and will not return until
                   // the user enters the `exit` command.
                   //! [i2c menu example]
#else
    logger.warn("C++ exceptions are not enabled, skipping I2C menu example");
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
  }

  {
    //! [i2c example]
    espp::I2c i2c({
        .port = I2C_NUM_0,
        .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    });

    // probe the bus for all addresses and store the ones that were found /
    // responded. NOTE: this will take a while to run, as it will probe all 128
    // possible addresses and the hard-coded timeout on the I2C (inside ESP-IDF)
    // is 1 second (I2C_CMD_ALIVE_INTERVAL_TICK within
    // esp-idf/components/driver/i2c/i2c.c).
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 0; address < 128; address++) {
      if (i2c.probe_device(address)) {
        found_addresses.push_back(address);
      }
    }
    // print out the addresses that were found
    logger.info("Found devices at addresses: {::#02x}", found_addresses);

    static constexpr uint8_t device_address = CONFIG_EXAMPLE_I2C_DEVICE_ADDR;
    static constexpr uint8_t register_address = CONFIG_EXAMPLE_I2C_DEVICE_REG_ADDR;
    bool device_found = i2c.probe_device(device_address);
    if (device_found) {
      logger.info("Found device with address {:#02x}", device_address);
      std::vector<uint8_t> read_data(CONFIG_EXAMPLE_I2C_DEVICE_REG_SIZE, 0);
      bool success = i2c.read_at_register(device_address, register_address, read_data.data(),
                                          read_data.size());
      if (success) {
        logger.info("read data: {::#02x}", read_data);
      } else {
        logger.error("read failed");
      }
    } else {
      logger.error("Could not find device with address {:#02x}", device_address);
    }
    //! [i2c example]
  }

  logger.info("I2C example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

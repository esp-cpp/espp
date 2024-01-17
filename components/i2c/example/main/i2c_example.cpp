#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "i2c.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "I2C Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");
  //! [i2c example]
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
    });
  static constexpr uint8_t device_address = CONFIG_EXAMPLE_I2C_DEVICE_ADDR;
  static constexpr uint8_t register_address = CONFIG_EXAMPLE_I2C_DEVICE_REG_ADDR;
  bool device_found = i2c.probe_device(device_address);
  if (device_found) {
    logger.info("Found device with address {:#02x}", device_address);
    std::vector<uint8_t> read_data(CONFIG_EXAMPLE_I2C_DEVICE_REG_SIZE, 0);
    bool success = i2c.read_at_register(device_address, register_address, read_data.data(), read_data.size());
    if (success) {
      logger.info("read data: {::#02x}", read_data);
    } else {
      logger.error("read failed");
    }
  } else {
    logger.error("Could not find device with address {:#02x}", device_address);
  }
  //! [i2c example]
  logger.info("I2C example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

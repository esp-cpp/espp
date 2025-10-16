#include <chrono>
#include <sdkconfig.h>
#include <vector>

#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API)
#include "i2c.hpp"
#include "i2c_menu.hpp"
#endif
#if defined(CONFIG_ESPP_I2C_USE_NEW_API)
#include "i2c_master.hpp"
#include "i2c_master_device_menu.hpp"
#include "i2c_master_menu.hpp"
#endif
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "I2C Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting");

//////////////////////////////////
// I2C Master Bus/Device Example using the Legacy API
//////////////////////////////////
#if defined(CONFIG_ESPP_I2C_USE_LEGACY_API)
  {
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

    input.Start();
    //! [i2c menu example]
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

    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 1; address < 128; address++) {
      if (i2c.probe_device(address)) {
        found_addresses.push_back(address);
      }
    }
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
#endif // CONFIG_ESPP_I2C_USE_LEGACY_API

//////////////////////////////////
// I2C Master Bus/Device Example using the New API
//////////////////////////////////
#if defined(CONFIG_ESPP_I2C_USE_NEW_API)
  {
    //! [i2c master bus creation example]
    logger.info("[NEW API] Starting I2C MasterBus/Device CLI example");
    using espp::I2cMasterBus;
    using espp::I2cMasterDevice;
    std::error_code ec;
    I2cMasterBus bus({
        .port = -1, // auto-select
        .sda_io_num = CONFIG_EXAMPLE_I2C_SDA_GPIO,
        .scl_io_num = CONFIG_EXAMPLE_I2C_SCL_GPIO,
        .clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
        .log_level = espp::Logger::Verbosity::DEBUG,
    });
    if (!bus.init(ec)) {
      logger.error("[NEW API] Failed to init bus: {}", ec.message());
      return;
    }
    //! [i2c master bus creation example]

    //! [i2c master device creation example]
    // Create a device (using uint8_t as the size for its register addresses)
    auto dev = bus.add_device<uint8_t>(
        espp::I2cMasterDevice<uint8_t>::Config{
            .device_address = CONFIG_EXAMPLE_I2C_DEVICE_ADDR,
            .scl_speed_hz = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
            .log_level = espp::Logger::Verbosity::DEBUG,
        },
        ec);
    if (!dev) {
      logger.error("[NEW API] Failed to add device: {}", ec.message());
      return;
    }
    logger.info("[NEW API] Created device with address {:#02x}", dev->get_device_address());
    //! [i2c master device creation example]
    //! [i2c master menu example]
    using espp::I2cMasterDeviceMenu;
    using espp::I2cMasterMenu;
    // Launch CLI menu for the new driver
    auto root_menu = std::make_unique<cli::Menu>("main", "I2C Main Menu");
    I2cMasterMenu i2c_menu(bus);
    I2cMasterDeviceMenu<uint8_t> i2c_device_menu(dev);
    root_menu->Insert(i2c_menu.get());
    root_menu->Insert(i2c_device_menu.get());
    cli::Cli cli(std::move(root_menu));
    cli::SetColor();
    cli.ExitAction([](auto &out) { out << "Goodbye and thanks for all the fish.\n"; });
    espp::Cli input(cli);
    input.SetInputHistorySize(10);
    input.Start();
    //! [i2c master menu example]

    //! [i2c master bus probe example]
    // NOTE: we turn off logging for this so we don't spam the console
    bus.set_log_level(espp::Logger::Verbosity::ERROR);
    std::vector<uint8_t> found_addresses;
    for (uint8_t address = 1; address < 128; address++) {
      static constexpr int timeout_ms = 50; // timeout for probing each address
      if (bus.probe(address, timeout_ms, ec)) {
        found_addresses.push_back(address);
      }
    }
    bus.set_log_level(espp::Logger::Verbosity::DEBUG);
    logger.info("Found devices at addresses: {::#02x}", found_addresses);
    //! [i2c master bus probe example]

    //! [i2c master device read example]
    static constexpr uint8_t device_address = CONFIG_EXAMPLE_I2C_DEVICE_ADDR;
    static constexpr uint8_t register_address = CONFIG_EXAMPLE_I2C_DEVICE_REG_ADDR;
    bool device_found = bus.probe(device_address, ec);
    if (device_found) {
      logger.info("Found device with address {:#02x}", device_address);
      std::vector<uint8_t> read_data(CONFIG_EXAMPLE_I2C_DEVICE_REG_SIZE, 0);
      bool success = dev->read_register(register_address, read_data, ec);
      if (success) {
        logger.info("read data: {::#02x}", read_data);
      } else {
        logger.error("read failed");
      }
    } else {
      logger.error("Could not find device with address {:#02x}", device_address);
    }
    //! [i2c master device read example]

    // test manually calling the deinit methods
    dev->deinit(ec);
    // you must ensure the bus is not destroyed or deinitialized until after all
    // its devices are!
    bus.deinit(ec);
    // NOTE: the deinit methods will be called again here when the objects go
    // out of scope. Because the bus created the device, the destruction order
    // will be correct automaticaly in this case.
  }
#endif // CONFIG_ESPP_I2C_USE_NEW_API

  logger.info("I2C example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

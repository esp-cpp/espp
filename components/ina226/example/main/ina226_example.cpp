#include <chrono>
#include <sdkconfig.h>

#include "i2c.hpp"
#include "ina226.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

//! [ina226 example]
extern "C" void app_main(void) {
  espp::Logger logger({.tag = "INA226 Example", .level = espp::Logger::Verbosity::INFO});

  espp::I2c i2c({
      .port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
      .clk_speed = CONFIG_EXAMPLE_I2C_CLOCK_SPEED_HZ,
  });

  // since the ina226 can have addresses ranging from 0x40 to 0x4F, we probe for
  // it
#if defined(CONFIG_EXAMPLE_USE_STATIC_ADDRESS)
  uint8_t address = CONFIG_EXAMPLE_I2C_ADDRESS;
#else
  uint8_t address = espp::Ina226::DEFAULT_ADDRESS;
  static constexpr uint8_t max_address = 0x4F;
  while (!i2c.probe_device(address) && address <= max_address) {
    address++;
  }
  if (address > max_address) {
    logger.info("INA226 not found on I2C bus!");
    return;
  }
#endif

  logger.info("Using INA226 address 0x{:02X}", address);

  // Configure INA226 with reasonable defaults and known shunt resistor
  espp::Ina226 ina({
      .device_address = address,
      .averaging = espp::Ina226::Avg::AVG_16,
      .bus_conv_time = espp::Ina226::ConvTime::MS_1_1,
      .shunt_conv_time = espp::Ina226::ConvTime::MS_1_1,
      .mode = espp::Ina226::Mode::SHUNT_BUS_CONT,
      .current_lsb = 0.001f,         // 1mA / LSB
      .shunt_resistance_ohms = 0.1f, // 0.1 ohm
      .probe = std::bind(&espp::I2c::probe_device, &i2c, std::placeholders::_1),
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read_register =
          std::bind(&espp::I2c::read_at_register, &i2c, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .write_then_read =
          std::bind(&espp::I2c::write_read, &i2c, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4, std::placeholders::_5),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  std::error_code ec;
  // Optional explicit initialize
  ina.initialize(ec);
  if (ec) {
    logger.info("INA226 init failed: {}", ec.message());
    return;
  }

  auto task_fn = [&](std::mutex &m, std::condition_variable &cv) {
    auto now = std::chrono::high_resolution_clock::now();

    float vbus = ina.bus_voltage_volts(ec);
    float vshunt = ina.shunt_voltage_volts(ec);
    float current = ina.current_amps(ec);
    float power = ina.power_watts(ec);
    if (ec) {
      logger.info("INA226 read failed: {}", ec.message());
      return true; // stop
    }
    logger.info("Vbus={:.3f}V, Vshunt={:.6f}V, I={:.3f}A, P={:.3f}W", vbus, vshunt, current, power);

    {
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, now + 250ms);
    }
    return false;
  };

  espp::Task task({.callback = task_fn,
                   .task_config = {.name = "INA226 Task", .stack_size_bytes = 4 * 1024},
                   .log_level = espp::Logger::Verbosity::WARN});
  task.start();

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
//! [ina226 example]

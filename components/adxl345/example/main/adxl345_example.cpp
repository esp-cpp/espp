#include <chrono>
#include <sdkconfig.h>
#include <vector>

#include "driver/gpio.h"

#include "adxl345.hpp"

#include "i2c.hpp"
#include "interrupt.hpp"
#include "logger.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

// This example is designed to be run on a board with an ADXL345 connected via I2C.
// Update the I2C pins and address as needed for your hardware.
//
// Example wiring for ESP32:
// ESP32          | ADXL345
// -------------------------
// 3.3V           | VCC
// GND            | GND
// SDA (GPIO21)   | SDA
// SCL (GPIO22)   | SCL
// ALERT (GPIO35) | INT1
//
// You can use the ESP-IDF menuconfig to set the I2C pins and the alert pin.

extern "C" void app_main(void) {
  static espp::Logger logger({.tag = "adxl345 example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting ADXL345 example!");

  //! [adxl345 example]
  // I2C configuration
  espp::I2c i2c({
      .port = I2C_NUM_0,
      .sda_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SDA_GPIO,
      .scl_io_num = (gpio_num_t)CONFIG_EXAMPLE_I2C_SCL_GPIO,
  });

  std::error_code ec;

  //////////////////////
  /// ADXL345 Configuration
  //////////////////////

  // Create the ADXL345 instance
  // NOTE: auto_init defaults to true, so we don't have to call initialize ourselves.
  espp::Adxl345 accel(espp::Adxl345::Config{
      .device_address = espp::Adxl345::DEFAULT_ADDRESS,
      .range = espp::Adxl345::RANGE_2G,
      .data_rate = espp::Adxl345::RATE_100_HZ,
      .write = std::bind(&espp::I2c::write, &i2c, std::placeholders::_1, std::placeholders::_2,
                         std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &i2c, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3),
      .log_level = espp::Logger::Verbosity::WARN,
  });

  // Disable measurement mode initially, so we can configure interrupts and such
  accel.set_measurement_mode(false, ec);

  // Configure the ADXL345
  accel.set_fifo_mode(espp::Adxl345::FifoMode::STREAM, ec); // discard old data when FIFO is full
  accel.set_fifo_num_samples(1, ec); // set FIFO to trigger watermark interrupt on every sample
  accel.set_low_power(false, ec);    // disable low power mode
  accel.set_sleep_mode(false, ec);   // disable sleep mode
  accel.set_auto_sleep(false, ec);   // disable auto sleep mode
  bool active_high = true;
  accel.set_interrupt_polarity(active_high, ec); // set interrupt polarity to active high
  // configure the interrupt to trigger on watermark on INT1 pin
  accel.set_interrupt_mapping(espp::Adxl345::InterruptType::WATERMARK,
                              espp::Adxl345::InterruptPin::INT1, ec);
  // enable the watermark interrupt
  accel.configure_interrupt(espp::Adxl345::InterruptType::WATERMARK, true, ec);

  //////////////////////
  /// Interrupt handling
  //////////////////////

  static auto start = std::chrono::high_resolution_clock::now();
  auto callback = [&](const espp::Interrupt::Event &event) {
    std::error_code ec;
    // clear the interrupt source
    [[maybe_unused]] auto interrupt_status = accel.get_interrupt_source(ec);
    if (ec) {
      logger.error("Error getting interrupt source: {}", ec.message());
    }
    auto now = std::chrono::high_resolution_clock::now();
    // we got a data ready interrupt, read the data and print it
    auto data_vec = accel.read_all(ec);
    if (ec) {
      fmt::print("Error reading ADXL345: {}\n", ec.message());
      return;
    }
    for (int i = 0; i < data_vec.size(); ++i) {
      auto &data = data_vec[i];
      // get the offset in time from now that hte sample was taken
      auto sample_time = now - 10ms * (data_vec.size() - i - 1);
      // convert to seconds
      auto elapsed = std::chrono::duration<float>(sample_time - start).count();
      // Print the data in g's
      fmt::print("{:.3f}, {:.4f}, {:.4f}, {:.4f}\n", elapsed, data.x, data.y, data.z);
    }
  };

  espp::Interrupt::PinConfig accel_int = {
      .gpio_num = (gpio_num_t)CONFIG_EXAMPLE_ALERT_GPIO,
      .callback = callback,
      .active_level = espp::Interrupt::ActiveLevel::HIGH,
      .interrupt_type = espp::Interrupt::Type::RISING_EDGE,
      .pullup_enabled = false,
      .pulldown_enabled = true,
      .filter_type = espp::Interrupt::FilterType::PIN_GLITCH_FILTER,
  };

  espp::Interrupt interrupt({
      .isr_core_id = 1,
      .interrupts = {accel_int},
      .task_config =
          {
              .name = "Interrupt task",
              .stack_size_bytes = 6192,
              .priority = 5,
          },
      .log_level = espp::Logger::Verbosity::WARN,
  });

  //////////////////////
  /// Get the data
  //////////////////////

  // Print CSV header
  fmt::print("%time (s), x (g), y (g), z (g)\n");

  // Enable measurement mode
  accel.set_measurement_mode(true, ec);

  // Keep the main function running to allow the example to run
  while (true) {
    auto start_time = std::chrono::high_resolution_clock::now();
    auto active_states = interrupt.get_active_states();
    logger.info("Instantaneous state of pins: {}", active_states);
    auto interrupt_status = accel.get_interrupt_source(ec);
    if (ec) {
      logger.error("Error getting interrupt source: {}", ec.message());
    } else {
      logger.info("Interrupt source: 0x{:02X}", interrupt_status);
    }
    auto data_vec = accel.read_all(ec);
    if (ec) {
      logger.error("Error reading all data: {}", ec.message());
    } else {
      logger.info("Read {} samples from ADXL345: {}", data_vec.size(), data_vec);
    }
    interrupt_status = accel.get_interrupt_source(ec);
    if (ec) {
      logger.error("Error getting interrupt source: {}", ec.message());
    } else {
      logger.info("Interrupt source: 0x{:02X}", interrupt_status);
    }
    std::this_thread::sleep_until(start_time + 100ms);
  }

  //! [adxl345 example]
}

#include "lilygo-t5-47.hpp"

namespace espp {

bool LilyGoT547::initialize_battery() {
  if (battery_initialized_) {
    logger_.warn("Battery gauge already initialized");
    return true;
  }
  if (!internal_i2c_) {
    logger_.error("initialize_display() must be called before initialize_battery(): it creates the "
                  "shared I2C bus");
    return false;
  }
  logger_.info("Initializing BQ27220 battery fuel gauge on the internal I2C bus");

  std::error_code ec;
  battery_i2c_device_ = internal_i2c_->add_device<std::uint8_t>(
      {
          .device_address = espp::Bq27220::DEFAULT_ADDRESS,
          .scl_speed_hz = 100 * 1000, // match epdiy / the board's 100 kHz bus
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!battery_i2c_device_) {
    logger_.error("Could not add BQ27220 I2C device: {}", ec.message());
    return false;
  }

  battery_ = std::make_unique<espp::Bq27220>(
      espp::Bq27220::Config{.write = espp::make_i2c_addressed_write(battery_i2c_device_),
                            .read = espp::make_i2c_addressed_read(battery_i2c_device_),
                            .log_level = espp::Logger::Verbosity::WARN});

  battery_initialized_ = true;
  return true;
}

} // namespace espp

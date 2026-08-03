#include "lilygo-t5-47.hpp"

namespace espp {

bool LilyGoT547::initialize_rtc() {
  if (rtc_initialized_) {
    logger_.warn("RTC already initialized");
    return true;
  }
  if (!internal_i2c_) {
    logger_.error("initialize_display() must be called before initialize_rtc(): it creates the "
                  "shared I2C bus");
    return false;
  }
  logger_.info("Initializing PCF8563 RTC on the internal I2C bus");

  std::error_code ec;
  rtc_i2c_device_ = internal_i2c_->add_device<std::uint8_t>(
      {
          .device_address = espp::Bm8563::DEFAULT_ADDRESS,
          .scl_speed_hz = 100 * 1000, // match epdiy / the board's 100 kHz bus
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!rtc_i2c_device_) {
    logger_.error("Could not add RTC I2C device: {}", ec.message());
    return false;
  }

  // The PCF8563 shares the BM8563's register map, so the espp::Bm8563 driver
  // drives it directly.
  rtc_ = std::make_unique<espp::Bm8563>(espp::Bm8563::Config{
      .write = espp::make_i2c_addressed_write(rtc_i2c_device_),
      .write_then_read = espp::make_i2c_addressed_write_then_read(rtc_i2c_device_),
      .log_level = espp::Logger::Verbosity::WARN});

  rtc_initialized_ = true;
  return true;
}

} // namespace espp

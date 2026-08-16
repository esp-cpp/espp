#include "lilygo-t5-47.hpp"

namespace espp {

bool LilyGoT547::initialize_io_expander() {
  if (io_expander_initialized_) {
    logger_.warn("I/O expander already initialized");
    return true;
  }
  if (!internal_i2c_) {
    logger_.error("initialize_display() must be called before initialize_io_expander(): it creates "
                  "the shared I2C bus");
    return false;
  }
  logger_.info("Initializing PCA9535 I/O expander on the internal I2C bus (shared with epdiy)");

  std::error_code ec;
  io_expander_i2c_device_ = internal_i2c_->add_device<std::uint8_t>(
      {
          .device_address = espp::Pca9535::DEFAULT_ADDRESS,
          .scl_speed_hz = 100 * 1000, // match epdiy / the board's 100 kHz bus
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!io_expander_i2c_device_) {
    logger_.error("Could not add PCA9535 I2C device: {}", ec.message());
    return false;
  }

  // auto_init=false: epdiy already configured this chip for e-paper power
  // sequencing (it owns port 1's high bits). Do NOT reconfigure it on
  // construction; only touch pins epdiy does not use, via read-modify-write.
  io_expander_ = std::make_unique<espp::Pca9535>(espp::Pca9535::Config{
      .write = espp::make_i2c_addressed_write(io_expander_i2c_device_),
      .read_register = espp::make_i2c_addressed_read_register(io_expander_i2c_device_),
      .auto_init = false,
      .log_level = espp::Logger::Verbosity::WARN});

  io_expander_initialized_ = true;
  return true;
}

bool LilyGoT547::io48_button_pressed() {
  if (!io_expander_) {
    logger_.error("initialize_io_expander() must be called before io48_button_pressed()");
    return false;
  }
  std::error_code ec;
  // Ensure PC12 (port 1, bit 2) is an input, without disturbing the port-1 bits
  // epdiy drives. Read the current direction and only add our bit.
  if (!io48_button_configured_) {
    uint8_t dir = io_expander_->get_direction(espp::Pca9535::Port::PORT1, ec);
    if (ec) {
      logger_.error("Could not read PCA9535 port-1 direction: {}", ec.message());
      return false;
    }
    io_expander_->set_direction(espp::Pca9535::Port::PORT1, dir | io48_button_port1_mask, ec);
    if (ec) {
      logger_.error("Could not set PCA9535 IO48-button pin as input: {}", ec.message());
      return false;
    }
    io48_button_configured_ = true;
  }
  uint8_t value = io_expander_->get_pins(espp::Pca9535::Port::PORT1, ec);
  if (ec) {
    logger_.error("Could not read PCA9535 port 1: {}", ec.message());
    return false;
  }
  // The IO48 button is active-low.
  return !(value & io48_button_port1_mask);
}

} // namespace espp

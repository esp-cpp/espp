#include "lilygo-t5-47.hpp"

namespace espp {

bool LilyGoT547::shutdown() {
  if (!internal_i2c_) {
    logger_.error("initialize_display() must be called before shutdown(): the PMIC is on the "
                  "shared I2C bus");
    return false;
  }

  std::error_code ec;
  if (!pmic_i2c_device_) {
    pmic_i2c_device_ = internal_i2c_->add_device<std::uint8_t>(
        {
            .device_address = pmic_address,
            .scl_speed_hz = 100 * 1000,
            .log_level = espp::Logger::Verbosity::WARN,
        },
        ec);
    if (!pmic_i2c_device_) {
      logger_.error("Could not add BQ25896 PMIC I2C device: {}", ec.message());
      return false;
    }
  }

  // BQ25896 REG09, bit 5 = BATFET_DIS: force the battery FET off (ship mode).
  // Bit 3 (BATFET_DLY) defaults to 0 = turn off immediately. Read-modify-write
  // so the other bits are preserved.
  static constexpr uint8_t REG09 = 0x09;
  static constexpr uint8_t BATFET_DIS = 1 << 5;
  uint8_t reg09 = 0;
  if (!pmic_i2c_device_->read_register(REG09, &reg09, 1, ec)) {
    logger_.error("Could not read BQ25896 REG09: {}", ec.message());
    return false;
  }
  const uint8_t buf[2] = {REG09, static_cast<uint8_t>(reg09 | BATFET_DIS)};
  logger_.info("Entering ship mode (BQ25896 BATFET off) - press PWR to power back on");
  if (!pmic_i2c_device_->write(buf, sizeof(buf), ec)) {
    logger_.error("Could not write BQ25896 REG09: {}", ec.message());
    return false;
  }
  return true;
}

} // namespace espp

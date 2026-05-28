#include "ws-s3-touch.hpp"

using namespace espp;

bool WsS3Touch::initialize_rtc() {
  if (rtc_) {
    logger_.warn("RTC already initialized");
    return true;
  }
  logger_.info("Initializing RTC");
  std::error_code ec;
  auto rtc_device = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = Rtc::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!rtc_device) {
    logger_.error("Could not initialize RTC I2C device: {}", ec.message());
    return false;
  }
  rtc_ = std::make_shared<Rtc>(Rtc::Config{
      .device_address = Rtc::DEFAULT_ADDRESS,
      .write = espp::make_i2c_addressed_write(rtc_device),
      .read = espp::make_i2c_addressed_read(rtc_device),
      .log_level = espp::Logger::Verbosity::WARN,
  });
  return true;
}

std::shared_ptr<WsS3Touch::Rtc> WsS3Touch::rtc() const { return rtc_; }

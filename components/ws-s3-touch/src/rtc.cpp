#include "ws-s3-touch.hpp"

using namespace espp;

bool WsS3Touch::initialize_rtc() {
  if (rtc_) {
    logger_.warn("RTC already initialized");
    return true;
  }
  logger_.info("Initializing RTC");
  rtc_ = std::make_shared<Rtc>(Rtc::Config{
      .device_address = Rtc::DEFAULT_ADDRESS,
      .write = std::bind_front(&espp::I2c::write, &internal_i2c_),
      .read = std::bind_front(&espp::I2c::read, &internal_i2c_),
      .log_level = espp::Logger::Verbosity::WARN,
  });
  return true;
}

std::shared_ptr<WsS3Touch::Rtc> WsS3Touch::rtc() const { return rtc_; }

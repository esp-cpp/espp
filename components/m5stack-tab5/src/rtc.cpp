#include "m5stack-tab5.hpp"

namespace espp {

//////////////////////////////////////////////////////////////////////////
// Real-Time Clock
//////////////////////////////////////////////////////////////////////////

bool M5StackTab5::initialize_rtc() {
  logger_.info("Initializing RX8130CE real-time clock");

  if (rtc_initialized_ || rtc_) {
    logger_.warn("RTC already initialized");
    return true;
  }

  rtc_ = std::make_shared<Rtc>(Rtc::Config{
      .device_address = Rtc::DEFAULT_ADDRESS,
      .write = std::bind_front(&espp::I2c::write, &internal_i2c_),
      .read = std::bind_front(&espp::I2c::read, &internal_i2c_),
      .auto_init = true,
      .log_level = Logger::Verbosity::WARN,
  });

  rtc_initialized_ = true;
  logger_.info("RTC initialization placeholder completed");
  return true;
}

bool M5StackTab5::set_rtc_time(const std::tm &time) {
  if (!rtc_initialized_ || !rtc_) {
    logger_.error("RTC not initialized");
    return false;
  }

  std::error_code ec;
  if (!rtc_->set_time(time, ec)) {
    logger_.error("Failed to set RTC time: {}", ec.message());
    return false;
  }

  logger_.info("RTC time set successfully");
  return true;
}

bool M5StackTab5::set_rtc_time(uint64_t unix_timestamp) {
  // Convert unix timestamp to std::tm
  std::time_t t = static_cast<std::time_t>(unix_timestamp);
  std::tm *tm_info = std::gmtime(&t);
  if (tm_info == nullptr) {
    logger_.error("Failed to convert unix timestamp to tm structure");
    return false;
  }
  return set_rtc_time(*tm_info);
}

bool M5StackTab5::get_rtc_time(std::tm &time) {
  if (!rtc_initialized_ || !rtc_) {
    logger_.error("RTC not initialized");
    return false;
  }

  std::error_code ec;
  time = rtc_->get_time(ec);
  if (ec) {
    logger_.error("Failed to get RTC time: {}", ec.message());
    return false;
  }

  logger_.info("RTC time retrieved successfully");
  return true;
}

uint64_t M5StackTab5::get_unix_time() {
  if (!rtc_initialized_ || !rtc_) {
    logger_.warn("RTC not initialized");
    return 0;
  }

  // Read time from RX8130CE and convert to unix timestamp
  std::error_code ec;
  std::tm tm_info = rtc_->get_time(ec);
  if (ec) {
    logger_.error("Failed to get RTC time: {}", ec.message());
    return 0;
  }
  std::time_t t = std::mktime(&tm_info);
  if (t == -1) {
    logger_.error("Failed to convert tm structure to unix timestamp");
    return 0;
  }

  logger_.info("RTC time retrieved: {}", static_cast<uint64_t>(t));
  return static_cast<uint64_t>(t);
}

bool M5StackTab5::set_rtc_wakeup(uint32_t seconds_from_now) {
  if (!rtc_initialized_) {
    logger_.error("RTC not initialized");
    return false;
  }

  // Configure RX8130CE alarm for wake-up
  std::error_code ec;
  std::tm current_time = rtc_->get_time(ec);
  if (ec) {
    logger_.error("Failed to get current RTC time: {}", ec.message());
    return false;
  }
  // Calculate alarm time
  std::tm alarm_time = current_time;
  alarm_time.tm_sec += static_cast<int>(seconds_from_now);
  std::mktime(&alarm_time); // Normalize the time
  if (!rtc_->set_alarm(alarm_time, false, ec)) {
    logger_.error("Failed to set RTC alarm: {}", ec.message());
    return false;
  }

  logger_.info("RTC wake-up set for {} seconds from now", seconds_from_now);
  return true;
}

} // namespace espp

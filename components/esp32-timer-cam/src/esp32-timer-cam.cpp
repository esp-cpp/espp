#include "esp32-timer-cam.hpp"

using namespace espp;

EspTimerCam::EspTimerCam()
    : BaseComponent("EspTimerCam") {}

////////////////////////
// LED                //
////////////////////////

bool EspTimerCam::initialize_led(float breathing_period) {
  if (led_) {
    logger_.error("LED already initialized");
    return false;
  }
  led_ = std::make_shared<espp::Led>(espp::Led::Config{
      .timer = LEDC_TIMER_2,
      .frequency_hz = 5000,
      .channels = led_channels_,
      .duty_resolution = LEDC_TIMER_10_BIT,
  });
  using namespace std::placeholders;
  led_task_ = espp::Task::make_unique(
      {.callback = std::bind(&EspTimerCam::led_task_callback, this, _1, _2, _3),
       .task_config = {.name = "breathe"}});
  set_led_breathing_period(breathing_period);
  return true;
}

void EspTimerCam::start_led_breathing() {
  breathing_start_ = std::chrono::high_resolution_clock::now();
  led_task_->start();
}

void EspTimerCam::stop_led_breathing() { led_task_->stop(); }

bool EspTimerCam::set_led_brightness(float brightness) {
  if (led_ == nullptr) {
    logger_.error("LED not initialized");
    return false;
  }
  if (led_task_->is_running()) {
    logger_.error("Cannot set brightness while breathing");
    return false;
  }
  brightness = std::clamp(brightness, 0.0f, 1.0f);
  led_->set_duty(led_channels_[0].channel, 100.0f * brightness);
  return true;
}

float EspTimerCam::get_led_brightness() {
  if (led_ == nullptr) {
    logger_.error("LED not initialized");
    return 0.0f;
  }
  auto maybe_duty = led_->get_duty(led_channels_[0].channel);
  if (!maybe_duty.has_value()) {
    logger_.error("Failed to get LED duty");
    return 0.0f;
  }
  return maybe_duty.value() / 100.0f;
}

bool EspTimerCam::set_led_breathing_period(float breathing_period) {
  if (breathing_period <= 0.0f) {
    logger_.error("Invalid breathing period: {}", breathing_period);
    return false;
  }
  breathing_period_ = breathing_period;
  return true;
}

float EspTimerCam::get_led_breathing_period() { return breathing_period_; }

std::shared_ptr<espp::Led> EspTimerCam::led() { return led_; }

espp::Gaussian &EspTimerCam::gaussian() { return gaussian_; }

float EspTimerCam::led_breathe() {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<float>(now - breathing_start_).count();
  float t = std::fmod(elapsed, breathing_period_) / breathing_period_;
  return gaussian_(t);
}

bool EspTimerCam::led_task_callback(std::mutex &m, std::condition_variable &cv,
                                    bool &task_notified) {
  using namespace std::chrono_literals;
  led_->set_duty(led_channels_[0].channel, 100.0f * led_breathe());
  std::unique_lock<std::mutex> lk(m);
  cv.wait_for(lk, 10ms, [&task_notified] { return task_notified; });
  task_notified = false;
  return false;
};

////////////////////////
// Battery            //
////////////////////////

float EspTimerCam::get_battery_voltage() {
  auto maybe_mv = adc_.read_mv(battery_channel_);
  float measurement = 0;
  if (maybe_mv.has_value()) {
    auto mv = maybe_mv.value();
    measurement = mv;
  }
  return measurement * BATTERY_VOLTAGE_SCALE;
}

////////////////////////
// RTC                //
////////////////////////

bool EspTimerCam::initialize_rtc() {
  if (rtc_) {
    logger_.error("RTC already initialized");
    return false;
  }
  std::error_code ec;
  rtc_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = EspTimerCam::Rtc::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!rtc_i2c_device_) {
    logger_.error("Could not initialize RTC I2C device: {}", ec.message());
    return false;
  }
  rtc_ = std::make_shared<EspTimerCam::Rtc>(EspTimerCam::Rtc::Config{
      .write = espp::make_i2c_addressed_write(rtc_i2c_device_),
      .write_then_read = espp::make_i2c_addressed_write_then_read(rtc_i2c_device_),
      .log_level = espp::Logger::Verbosity::WARN});
  return true;
}

std::shared_ptr<EspTimerCam::Rtc> EspTimerCam::rtc() { return rtc_; }

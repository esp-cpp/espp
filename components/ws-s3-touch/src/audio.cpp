#include "ws-s3-touch.hpp"

using namespace espp;

////////////////////////
// Audio Functions   //
////////////////////////

bool WsS3Touch::initialize_buzzer() {
  if (buzzer_) {
    logger_.warn("Buzzer already initialized");
    return true;
  }
  logger_.info("Initializing buzzer...");
  buzzer_ = std::make_shared<espp::Led>(espp::Led::Config{
      .timer = buzzer_ledc_timer,
      .frequency_hz = buzzer_default_frequency_hz,
      .channels = {buzzer_channel_config_},
      .duty_resolution = buzzer_duty_resolution,
  });
  return true;
}

bool WsS3Touch::buzzer(float pwm, size_t frequency_hz) {
  if (!buzzer_) {
    logger_.error("Buzzer not initialized");
    return false;
  }
  return buzzer_->set_pwm(buzzer_ledc_channel, frequency_hz, pwm);
}

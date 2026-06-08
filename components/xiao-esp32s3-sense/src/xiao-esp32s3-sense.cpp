#include "xiao-esp32s3-sense.hpp"

#include <cmath>

using namespace espp;

XiaoEsp32S3Sense::XiaoEsp32S3Sense()
    : BaseComponent("XiaoEsp32S3Sense") {}

XiaoEsp32S3Sense::CameraPins XiaoEsp32S3Sense::camera_pins() const {
  return {
      .pwdn = camera_pwdn_pin(),
      .reset = camera_reset_pin(),
      .xclk = camera_xclk_pin(),
      .sccb_sda = camera_sccb_sda_pin(),
      .sccb_scl = camera_sccb_scl_pin(),
      .d7 = camera_d7_pin(),
      .d6 = camera_d6_pin(),
      .d5 = camera_d5_pin(),
      .d4 = camera_d4_pin(),
      .d3 = camera_d3_pin(),
      .d2 = camera_d2_pin(),
      .d1 = camera_d1_pin(),
      .d0 = camera_d0_pin(),
      .vsync = camera_vsync_pin(),
      .href = camera_href_pin(),
      .pclk = camera_pclk_pin(),
  };
}

XiaoEsp32S3Sense::MicrophonePins XiaoEsp32S3Sense::microphone_pins() const {
  return {
      .clk = microphone_clk_pin(),
      .data = microphone_data_pin(),
  };
}

XiaoEsp32S3Sense::SdCardPins XiaoEsp32S3Sense::sd_card_pins() const {
  return {
      .clk = sd_card_clk_pin(),
      .mosi = sd_card_mosi_pin(),
      .miso = sd_card_miso_pin(),
      .cs = sd_card_cs_pin(),
  };
}

bool XiaoEsp32S3Sense::initialize_led(float breathing_period) {
  if (led_) {
    logger_.error("LED already initialized");
    return false;
  }
  led_ = std::make_shared<espp::Led>(espp::Led::Config{
      .timer = LEDC_TIMER_1,
      .frequency_hz = 5000,
      .channels = led_channels_,
      .duty_resolution = LEDC_TIMER_10_BIT,
  });
  using namespace std::placeholders;
  led_task_ = espp::Task::make_unique(
      {.callback = std::bind(&XiaoEsp32S3Sense::led_task_callback, this, _1, _2, _3),
       .task_config = {.name = "xiao_led_breathe"}});
  set_led_breathing_period(breathing_period);
  return true;
}

void XiaoEsp32S3Sense::start_led_breathing() {
  breathing_start_ = std::chrono::high_resolution_clock::now();
  led_task_->start();
}

void XiaoEsp32S3Sense::stop_led_breathing() { led_task_->stop(); }

bool XiaoEsp32S3Sense::set_led_brightness(float brightness) {
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

float XiaoEsp32S3Sense::get_led_brightness() {
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

bool XiaoEsp32S3Sense::set_led_breathing_period(float breathing_period) {
  if (breathing_period <= 0.0f) {
    logger_.error("Invalid breathing period: {}", breathing_period);
    return false;
  }
  breathing_period_ = breathing_period;
  return true;
}

float XiaoEsp32S3Sense::get_led_breathing_period() { return breathing_period_; }

std::shared_ptr<espp::Led> XiaoEsp32S3Sense::led() { return led_; }

espp::Gaussian &XiaoEsp32S3Sense::gaussian() { return gaussian_; }

float XiaoEsp32S3Sense::led_breathe() {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<float>(now - breathing_start_).count();
  float t = std::fmod(elapsed, breathing_period_) / breathing_period_;
  return gaussian_(t);
}

bool XiaoEsp32S3Sense::led_task_callback(std::mutex &m, std::condition_variable &cv,
                                         bool &task_notified) {
  using namespace std::chrono_literals;
  led_->set_duty(led_channels_[0].channel, 100.0f * led_breathe());
  std::unique_lock<std::mutex> lk(m);
  cv.wait_for(lk, 10ms, [&task_notified] { return task_notified; });
  task_notified = false;
  return false;
}

i2s_pdm_rx_clk_config_t XiaoEsp32S3Sense::microphone_clock_config(uint32_t sample_rate_hz) const {
  i2s_pdm_rx_clk_config_t config = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate_hz);
  return config;
}

i2s_pdm_rx_slot_config_t XiaoEsp32S3Sense::microphone_slot_config(i2s_slot_mode_t slot_mode) const {
  i2s_pdm_rx_slot_config_t config =
      I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode);
  return config;
}

i2s_pdm_rx_gpio_config_t XiaoEsp32S3Sense::microphone_gpio_config(bool clock_inverted) const {
  i2s_pdm_rx_gpio_config_t config = {};
  config.clk = microphone_clk_pin();
  config.din = microphone_data_pin();
  config.invert_flags.clk_inv = clock_inverted;
  return config;
}

i2s_pdm_rx_config_t XiaoEsp32S3Sense::microphone_config(uint32_t sample_rate_hz,
                                                        i2s_slot_mode_t slot_mode,
                                                        bool clock_inverted) const {
  return {
      .clk_cfg = microphone_clock_config(sample_rate_hz),
      .slot_cfg = microphone_slot_config(slot_mode),
      .gpio_cfg = microphone_gpio_config(clock_inverted),
  };
}

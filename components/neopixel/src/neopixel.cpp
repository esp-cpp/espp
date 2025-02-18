#include "neopixel.hpp"

using namespace espp;

Neopixel::Neopixel(const Neopixel::Config& config) : config_(config) {
  // configure the power pin
  configure_power_pin();
  // turn the power on
  set_power(true);
  // make the encoder
  using namespace std::placeholders;
  auto led_encoder = std::make_unique<espp::RmtEncoder>(espp::RmtEncoder::Config{
      // NOTE: since we're using the 10MHz RMT clock, we can use the pre-defined
      //       ws2812_10mhz_bytes_encoder_config
      .bytes_encoder_config = espp::RmtEncoder::ws2812_10mhz_bytes_encoder_config,
        .encode = std::bind(&Neopixel::encode, this, _1, _2, _3, _4, _5, _6),
        .del = [](auto *base_encoder) -> esp_err_t {
          // we don't have any extra resources to free, so just return ESP_OK
          return ESP_OK;
        },
        .reset = [this](auto *base_encoder) -> esp_err_t {
          // all we have is some extra state to reset
          led_encoder_state_ = 0;
          return ESP_OK;
        },
        });

  rmt_ = std::make_shared<espp::Rmt>(espp::Rmt::Config{
      .gpio_num = config_.data_gpio,
      .resolution_hz = WS2812_FREQ_HZ,
      .log_level = espp::Logger::Verbosity::INFO,
    });
  rmt_->set_encoder(std::move(led_encoder));

  // resize the led_data_ array to hold the data for all the LEDs
  led_data_.resize(config_.num_leds * 3);
}

void Neopixel::configure_power_pin() {
  // don't do anything if the power pin is not set
  if (config_.power_gpio == -1) {
    return;
  }
  gpio_config_t power_pin_config = {
    .pin_bit_mask = (1ULL << config_.power_gpio),
    .mode = GPIO_MODE_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&power_pin_config);
}

void Neopixel::set_power(bool on) {
  // don't do anything if the power pin is not set
  if (config_.power_gpio == -1) {
    return;
  }
  gpio_set_level((gpio_num_t)config_.power_gpio, on);
}

std::size_t Neopixel::encode(rmt_channel_handle_t channel, rmt_encoder_t *copy_encoder, rmt_encoder_t *bytes_encoder, const void *data, std::size_t data_size, rmt_encode_state_t *ret_state) {
  // divide by 2 since we have both duration0 and duration1 in the reset code
  static uint16_t reset_ticks =
    WS2812_FREQ_HZ / MICROS_PER_SEC * 50 / 2; // reset code duration defaults to 50us
  static rmt_symbol_word_t led_reset_code = (rmt_symbol_word_t){
    .duration0 = reset_ticks,
    .level0 = 0,
    .duration1 = reset_ticks,
    .level1 = 0,
  };
  rmt_encode_state_t session_state = RMT_ENCODING_RESET;
  int state = RMT_ENCODING_RESET;
  size_t encoded_symbols = 0;
  switch (led_encoder_state_) {
  case 0: // send RGB data
    encoded_symbols +=
      bytes_encoder->encode(bytes_encoder, channel, data, data_size, &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
      led_encoder_state_ = 1; // switch to next state when current encoding session finished
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; // yield if there's no free space for encoding artifacts
    }
    // fall-through
  case 1: // send reset code
    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &led_reset_code,
                                            sizeof(led_reset_code), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE) {
      led_encoder_state_ = RMT_ENCODING_RESET; // back to the initial encoding session
      state |= RMT_ENCODING_COMPLETE;
    }
    if (session_state & RMT_ENCODING_MEM_FULL) {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; // yield if there's no free space for encoding artifacts
    }
  }
 out:
  *ret_state = static_cast<rmt_encode_state_t>(state);
  return encoded_symbols;
}

void Neopixel::set_color(const espp::Rgb &rgb, std::size_t index) {
  // return if the index is out of bounds
  if (index >= config_.num_leds) {
    return;
  }
  // convert the RGB values to 8-bit integers
  uint8_t red = std::clamp(int(rgb.r * 255), 0, 255);
  uint8_t green = std::clamp(int(rgb.g * 255), 0, 255);
  uint8_t blue = std::clamp(int(rgb.b * 255), 0, 255);
  // set the RGB values at the index of the led_data_ array
  led_data_[index * 3] = green;
  led_data_[index * 3 + 1] = red;
  led_data_[index * 3 + 2] = blue;
}

void Neopixel::set_all(const espp::Rgb &rgb) {
  uint8_t red = std::clamp(int(rgb.r * 255), 0, 255);
  uint8_t green = std::clamp(int(rgb.g * 255), 0, 255);
  uint8_t blue = std::clamp(int(rgb.b * 255), 0, 255);
  // fill the led_data_ array with the RGB data
  for (int i=0; i<led_data_.size(); i+=3) {
    led_data_[i] = green;
    led_data_[i+1] = red;
    led_data_[i+2] = blue;
  }
}

void Neopixel::show() {
  // now we can send the data to the LED
  rmt_->transmit(led_data_.data(), led_data_.size());
}

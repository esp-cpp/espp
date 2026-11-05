#include "es8388_codec.hpp"

#include <algorithm>
#include <cmath>

#include <esp_log.h>

using namespace espp;

static const char *TAG = "Es8388Codec";

Es8388Codec *Es8388Codec::owner_ = nullptr;

Es8388Codec::Es8388Codec(const Config &config) {
  set_es8388_write(config.write);
  set_es8388_read(config.read);
  owner_ = this;
}

Es8388Codec::~Es8388Codec() {
  // only if no newer instance has set them since (one still held through an
  // espp::Audio the application kept would otherwise break its successor)
  if (owner_ == this) {
    set_es8388_write(nullptr);
    set_es8388_read(nullptr);
    owner_ = nullptr;
  }
}

bool Es8388Codec::start(Direction direction, uint32_t sample_rate) {
  if (direction != Direction::PLAYBACK) {
    ESP_LOGE(TAG, "Only playback is supported (record through an ES7210)");
    return false;
  }
  audio_hal_codec_config_t cfg{};
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL; // both L and R outputs
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  // as an I2S slave the ES8388 follows the clocks it is given; the driver
  // does not use the rate
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
  ESP_LOGI(TAG, "Initializing the ES8388 DAC at %u Hz", static_cast<unsigned>(sample_rate));
  if (es8388_init(&cfg) != ESP_OK) {
    ESP_LOGE(TAG, "ES8388 init failed");
    return false;
  }
  if (es8388_config_fmt(ES_MODULE_DAC, ES_I2S_NORMAL) != ESP_OK) {
    ESP_LOGE(TAG, "ES8388 format config failed");
    return false;
  }
  if (es8388_set_bits_per_sample(ES_MODULE_DAC, BIT_LENGTH_16BITS) != ESP_OK) {
    ESP_LOGE(TAG, "ES8388 bits per sample config failed");
    return false;
  }
  if (es8388_config_dac_output(DAC_OUTPUT_ALL) != ESP_OK) {
    ESP_LOGE(TAG, "ES8388 DAC output config failed");
    return false;
  }
  if (es8388_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START) != ESP_OK) {
    ESP_LOGE(TAG, "ES8388 start failed");
    return false;
  }
  return true;
}

void Es8388Codec::stop() {
  es8388_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_STOP);
  es8388_deinit();
}

bool Es8388Codec::set_volume(float percent) {
  return es8388_set_voice_volume(static_cast<int>(std::clamp(percent, 0.0f, 100.0f))) == ESP_OK;
}

bool Es8388Codec::set_mute(bool mute) { return es8388_set_voice_mute(mute) == ESP_OK; }

#include "es8311_codec.hpp"

#include <algorithm>
#include <cmath>

#include <esp_log.h>

using namespace espp;

static const char *TAG = "Es8311Codec";

Es8311Codec *Es8311Codec::owner_ = nullptr;

// Map a PCM sample rate onto the driver's sample-rate enum, which selects the
// ES8311 clock coefficients: it must match the rate the I2S clocks generate.
static bool samples_from_rate(uint32_t rate, audio_hal_iface_samples_t &samples) {
  switch (rate) {
  case 8000:
    samples = AUDIO_HAL_08K_SAMPLES;
    return true;
  case 11025:
    samples = AUDIO_HAL_11K_SAMPLES;
    return true;
  case 16000:
    samples = AUDIO_HAL_16K_SAMPLES;
    return true;
  case 22050:
    samples = AUDIO_HAL_22K_SAMPLES;
    return true;
  case 24000:
    samples = AUDIO_HAL_24K_SAMPLES;
    return true;
  case 32000:
    samples = AUDIO_HAL_32K_SAMPLES;
    return true;
  case 44100:
    samples = AUDIO_HAL_44K_SAMPLES;
    return true;
  case 48000:
    samples = AUDIO_HAL_48K_SAMPLES;
    return true;
  default:
    return false;
  }
}

Es8311Codec::Es8311Codec(const Config &config) {
  set_es8311_write(config.write);
  set_es8311_read(config.read);
  owner_ = this;
}

Es8311Codec::~Es8311Codec() {
  // only if no newer instance has set them since (one still held through an
  // espp::Audio the application kept would otherwise break its successor)
  if (owner_ == this) {
    set_es8311_write(nullptr);
    set_es8311_read(nullptr);
    owner_ = nullptr;
  }
}

bool Es8311Codec::start(Direction direction, uint32_t sample_rate) {
  audio_hal_codec_config_t cfg{};
  if (!samples_from_rate(sample_rate, cfg.i2s_iface.samples)) {
    ESP_LOGE(TAG,
             "Unsupported sample rate %u Hz (8000, 11025, 16000, 22050, 24000, 32000, 44100 or "
             "48000)",
             static_cast<unsigned>(sample_rate));
    return false;
  }
  switch (direction) {
  case Direction::PLAYBACK:
    mode_ = AUDIO_HAL_CODEC_MODE_DECODE;
    break;
  case Direction::RECORDING:
    mode_ = AUDIO_HAL_CODEC_MODE_ENCODE;
    break;
  case Direction::BOTH:
    mode_ = AUDIO_HAL_CODEC_MODE_BOTH;
    break;
  }
  cfg.codec_mode = mode_;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  ESP_LOGI(TAG, "Initializing the ES8311 at %u Hz", static_cast<unsigned>(sample_rate));
  // es8311_codec_init() sets the clocks and the master / slave mode, but not
  // the serial data format: 16-bit Philips, in both directions
  if (es8311_codec_init(&cfg) != ESP_OK ||
      es8311_codec_config_i2s(mode_, &cfg.i2s_iface) != ESP_OK) {
    ESP_LOGE(TAG, "ES8311 init failed");
    return false;
  }
  if (es8311_codec_ctrl_state(mode_, AUDIO_HAL_CTRL_START) != ESP_OK) {
    // no cleanup here: per the AudioCodec contract espp::Audio calls stop()
    // after a failed start() (it counts the codec as started first), and
    // stop() undoes this mode_'s init
    ESP_LOGE(TAG, "ES8311 start failed");
    return false;
  }
  return true;
}

void Es8311Codec::stop() {
  es8311_codec_ctrl_state(mode_, AUDIO_HAL_CTRL_STOP);
  es8311_codec_deinit();
}

bool Es8311Codec::set_sample_rate(uint32_t sample_rate) {
  // the ES8311 divides MCLK down with per-rate coefficients: it does not
  // simply follow the I2S clocks
  return es8311_codec_set_sample_rate(static_cast<int>(sample_rate)) == ESP_OK;
}

bool Es8311Codec::set_volume(float percent) {
  volume_ = std::clamp(percent, 0.0f, 100.0f);
  // while muted the DAC's mute bits keep it silent
  return es8311_codec_set_voice_volume(static_cast<int>(volume_)) == ESP_OK;
}

bool Es8311Codec::set_mute(bool mute) {
  if (es8311_set_voice_mute(mute) != ESP_OK) {
    return false;
  }
  // muting also zeroes the DAC volume, which unmuting does not restore
  return mute || es8311_codec_set_voice_volume(static_cast<int>(volume_)) == ESP_OK;
}

bool Es8311Codec::set_gain(float percent) {
  // 0 - 100 % onto the analog gain steps ES8311_MIC_GAIN_0DB .. _42DB (6 dB
  // apart)
  int step = static_cast<int>(std::lround(std::clamp(percent, 0.0f, 100.0f) / 100.0f *
                                          static_cast<float>(ES8311_MIC_GAIN_42DB)));
  step = std::clamp(step, static_cast<int>(ES8311_MIC_GAIN_0DB),
                    static_cast<int>(ES8311_MIC_GAIN_42DB));
  return es8311_set_mic_gain(static_cast<es8311_mic_gain_t>(step)) == ESP_OK;
}

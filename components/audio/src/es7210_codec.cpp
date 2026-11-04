#include "es7210_codec.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

using namespace espp;

// Map a PCM sample rate onto the driver's sample-rate enum, which selects the
// ES7210 coefficient table: it must match the rate the I2S clocks generate.
// An unsupported rate falls back to 48 kHz.
static audio_hal_iface_samples_t samples_from_rate(uint32_t rate) {
  switch (rate) {
  case 8000:
    return AUDIO_HAL_08K_SAMPLES;
  case 11025:
    return AUDIO_HAL_11K_SAMPLES;
  case 16000:
    return AUDIO_HAL_16K_SAMPLES;
  case 22050:
    return AUDIO_HAL_22K_SAMPLES;
  case 24000:
    return AUDIO_HAL_24K_SAMPLES;
  case 32000:
    return AUDIO_HAL_32K_SAMPLES;
  case 44100:
    return AUDIO_HAL_44K_SAMPLES;
  case 48000:
    return AUDIO_HAL_48K_SAMPLES;
  default:
    return AUDIO_HAL_48K_SAMPLES;
  }
}

static audio_hal_codec_i2s_iface_t make_iface(uint32_t sample_rate) {
  audio_hal_codec_i2s_iface_t iface{};
  iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  iface.fmt = AUDIO_HAL_I2S_NORMAL;
  iface.mode = AUDIO_HAL_MODE_SLAVE;
  iface.samples = samples_from_rate(sample_rate);
  return iface;
}

Es7210Codec::Es7210Codec(const Config &config)
    : BaseComponent("Es7210Codec", config.log_level)
    , config_(config) {
  set_es7210_write(config.write);
  set_es7210_read(config.read);
}

bool Es7210Codec::start(Direction direction, uint32_t sample_rate) {
  if (direction != Direction::RECORDING) {
    logger_.error("Only recording is supported");
    return false;
  }
  if (config_.microphones && es7210_mic_select(*config_.microphones) != ESP_OK) {
    logger_.error("ES7210 microphone select failed");
    return false;
  }
  audio_hal_codec_config_t cfg{};
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  cfg.i2s_iface = make_iface(sample_rate);
  logger_.info("Initializing the ES7210 ADC at {} Hz", sample_rate);
  if (es7210_adc_init(&cfg) != ESP_OK) {
    logger_.error("ES7210 init failed");
    return false;
  }
  if (es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &cfg.i2s_iface) != ESP_OK) {
    logger_.error("ES7210 I2S config failed");
  }
  if (config_.high_pass_filter) {
    // The driver leaves the HPF registers at their reset value (off); these
    // are its documented "quick setup" values. A write's result does not
    // mask an earlier failure.
    static constexpr uint8_t ES7210_ADDR = 0x40; // the driver's (AD1/AD0 = 00)
    bool hpf_ok = true;
    for (auto [reg, value] : {std::pair<uint8_t, uint8_t>{0x22, 0x0a}, // ADC1/2 HPF1
                              {0x23, 0x2a},                            // ADC1/2 HPF2
                              {0x20, 0x0a},                            // ADC3/4 HPF2
                              {0x21, 0x2a}}) {                         // ADC3/4 HPF1
      uint8_t data[] = {reg, value};
      hpf_ok &= config_.write(ES7210_ADDR, data, sizeof(data));
    }
    if (!hpf_ok) {
      logger_.warn("Could not enable the ES7210 high-pass filter");
    }
  }
  if (es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START) != ESP_OK) {
    logger_.error("ES7210 start failed");
    return false;
  }
  return true;
}

void Es7210Codec::stop() {
  es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_STOP);
  es7210_adc_deinit();
}

bool Es7210Codec::set_sample_rate(uint32_t sample_rate) {
  auto iface = make_iface(sample_rate);
  return es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &iface) == ESP_OK;
}

bool Es7210Codec::set_gain(float percent) {
  // 0 - 100 % onto the analog gain steps GAIN_0DB .. GAIN_37_5DB
  int step = static_cast<int>(
      std::lround(std::clamp(percent, 0.0f, 100.0f) / 100.0f * static_cast<float>(GAIN_37_5DB)));
  step = std::clamp(step, static_cast<int>(GAIN_0DB), static_cast<int>(GAIN_37_5DB));
  return es7210_adc_set_gain_all(static_cast<es7210_gain_value_t>(step)) == ESP_OK;
}

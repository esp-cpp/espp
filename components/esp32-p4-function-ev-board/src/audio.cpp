#include "esp32-p4-function-ev-board.hpp"

#include <algorithm>
#include <cstring>

#include <driver/gpio.h>

#include "es8311.hpp"

namespace espp {

bool Esp32P4FunctionEvBoard::initialize_audio(uint32_t sample_rate,
                                              const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing audio (ES8311) at {} Hz", sample_rate);

  if (audio_initialized_) {
    logger_.warn("Audio already initialized");
    return true;
  }

  // Configure the speaker-amplifier (NS4150B) enable GPIO
  gpio_config_t pa_cfg{};
  pa_cfg.pin_bit_mask = 1ULL << static_cast<int>(audio_pa_enable_io);
  pa_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&pa_cfg);
  set_speaker_enabled(false);

  std::error_code ec;
  es8311_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = es8311_i2c_address,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!es8311_i2c_device_) {
    logger_.error("Could not initialize ES8311 I2C device: {}", ec.message());
    return false;
  }

  // Wire codec register access over the internal I2C bus
  set_es8311_write(espp::make_i2c_addressed_write(es8311_i2c_device_));
  set_es8311_read(espp::make_i2c_addressed_read_register(es8311_i2c_device_));

  // Create the I2S standard channel for playback (TX). MCLK = 256 * fs (default).
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(audio_i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  if (i2s_new_channel(&chan_cfg, &audio_tx_handle, nullptr) != ESP_OK) {
    logger_.error("Failed to create I2S channel");
    return false;
  }

  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
      .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = audio_mclk_io,
                   .bclk = audio_sclk_io,
                   .ws = audio_lrck_io,
                   .dout = audio_dout_io,
                   .din = audio_din_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  if (i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg) != ESP_OK) {
    logger_.error("Failed to init I2S std mode");
    return false;
  }

  // Initialize the ES8311 codec for playback (codec is an I2S slave)
  audio_hal_codec_config_t es8311_cfg{};
  es8311_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  es8311_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
  es8311_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es8311_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es8311_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  es8311_cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
  if (es8311_codec_init(&es8311_cfg) != ESP_OK) {
    logger_.error("ES8311 init failed");
    return false;
  }
  es8311_codec_set_sample_rate(sample_rate);
  es8311_codec_set_voice_volume(static_cast<int>(volume_));
  es8311_set_voice_mute(false);
  es8311_codec_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

  if (i2s_channel_enable(audio_tx_handle) != ESP_OK) {
    logger_.error("Failed to enable I2S channel");
    return false;
  }

  // The audio task drains this stream buffer to I2S. Size it generously so a
  // whole UI sound clip fits and play_audio() can enqueue it without blocking.
  auto tx_buf_size = calc_audio_buffer_size(sample_rate);
  audio_tx_buffer.resize(tx_buf_size);
  audio_tx_stream = xStreamBufferCreate(std::max<size_t>(tx_buf_size * 4, 64 * 1024), 0);
  xStreamBufferReset(audio_tx_stream);

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique(
      {.callback = std::bind(&Esp32P4FunctionEvBoard::audio_task_callback, this, _1, _2, _3),
       .task_config = task_config});

  set_speaker_enabled(true);
  audio_initialized_ = true;
  return audio_task_->start();
}

void Esp32P4FunctionEvBoard::set_speaker_enabled(bool enable) {
  gpio_set_level(audio_pa_enable_io, enable ? 1 : 0);
}

void Esp32P4FunctionEvBoard::volume(float volume) {
  volume = std::clamp(volume, 0.0f, 100.0f);
  volume_ = volume;
  es8311_codec_set_voice_volume(static_cast<int>(volume_));
}

float Esp32P4FunctionEvBoard::volume() const { return volume_; }

void Esp32P4FunctionEvBoard::mute(bool mute) {
  mute_ = mute;
  es8311_set_voice_mute(mute_);
}

bool Esp32P4FunctionEvBoard::is_muted() const { return mute_; }

void Esp32P4FunctionEvBoard::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!audio_initialized_ || !data || num_bytes == 0) {
    return;
  }
  // Non-blocking: enqueue as much as currently fits in the TX stream buffer for
  // the audio task to drain to I2S. This never blocks the caller, so it is safe
  // to call from a touch callback / any task. If the buffer is full the excess
  // is dropped (the clip is truncated) rather than stalling the caller.
  xStreamBufferSend(audio_tx_stream, data, num_bytes, 0);
}

void Esp32P4FunctionEvBoard::play_audio(std::span<const uint8_t> data) {
  play_audio(data.data(), data.size());
}

bool Esp32P4FunctionEvBoard::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                                 bool &task_notified) {
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *tx_buf = audio_tx_buffer.data();
  memset(tx_buf, 0, buffer_size);
  if (available == 0) {
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
    i2s_channel_write(audio_tx_handle, tx_buf, available, NULL, portMAX_DELAY);
  }
  return false;
}

uint32_t Esp32P4FunctionEvBoard::audio_sample_rate() const {
  return audio_std_cfg.clk_cfg.sample_rate_hz;
}

size_t Esp32P4FunctionEvBoard::audio_buffer_size() const { return audio_tx_buffer.size(); }

void Esp32P4FunctionEvBoard::audio_sample_rate(uint32_t sample_rate) {
  // NOTE: this reconfigures the running I2S channel. It is best called when the
  // audio task is not actively streaming (e.g. right after initialize_audio, or
  // while no audio is playing). To avoid a runtime change entirely, pass the
  // desired sample rate to initialize_audio(). The ES8311 is an I2S slave and
  // follows the I2S clock, so it does not need a separate codec reconfigure.
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  i2s_channel_disable(audio_tx_handle);
  audio_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle, &audio_std_cfg.clk_cfg);
  xStreamBufferReset(audio_tx_stream);
  i2s_channel_enable(audio_tx_handle);
}

} // namespace espp

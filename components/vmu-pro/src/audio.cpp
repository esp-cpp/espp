#include "vmu-pro.hpp"

using namespace espp;

////////////////////////
// Audio Functions    //
////////////////////////

bool VmuPro::initialize_i2s(uint32_t default_audio_rate) {
  logger_.info("initializing i2s driver");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, nullptr));

  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(default_audio_rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = GPIO_NUM_NC,
                   .bclk = i2s_bck_io,
                   .ws = i2s_ws_io,
                   .dout = i2s_do_io,
                   .din = GPIO_NUM_NC,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  audio_tx_buffer.resize(buffer_size);

  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 0);

  xStreamBufferReset(audio_tx_stream);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  return true;
}

bool VmuPro::initialize_sound(uint32_t default_audio_rate,
                              const espp::Task::BaseConfig &task_config) {
  if (sound_initialized_) {
    logger_.warn("Sound already initialized");
    return true;
  }
  if (!initialize_i2s(default_audio_rate)) {
    logger_.error("Could not initialize I2S driver");
    return false;
  }

  // Configure the amplifier enable IO
  gpio_set_direction(sound_enable_pin, GPIO_MODE_OUTPUT);
  enable_sound(true);

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&VmuPro::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  sound_initialized_ = true;

  return audio_task_->start();
}

void VmuPro::enable_sound(bool enable) { gpio_set_level(sound_enable_pin, enable); }

bool VmuPro::audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  // Queue the next I2S out frame to write
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *buffer = &audio_tx_buffer[0];
  memset(buffer, 0, buffer_size);

  if (available > 0) {
    xStreamBufferReceive(audio_tx_stream, buffer, available, 0);
    // The VMU Pro has no hardware volume control, so scale the samples in
    // software according to the current volume / mute state
    float scale = mute_ ? 0.0f : (volume_ / 100.0f);
    auto *samples = reinterpret_cast<int16_t *>(buffer);
    size_t num_samples = available / sizeof(int16_t);
    for (size_t i = 0; i < num_samples; i++) {
      samples[i] = static_cast<int16_t>(samples[i] * scale);
    }
  }
  i2s_channel_write(audio_tx_handle, buffer, buffer_size, NULL, portMAX_DELAY);
  return false; // don't stop the task
}

void VmuPro::mute(bool mute) { mute_ = mute; }

bool VmuPro::is_muted() const { return mute_; }

void VmuPro::volume(float volume) { volume_ = std::clamp(volume, 0.0f, 100.0f); }

float VmuPro::volume() const { return volume_; }

uint32_t VmuPro::audio_sample_rate() const { return audio_std_cfg.clk_cfg.sample_rate_hz; }

size_t VmuPro::audio_buffer_size() const { return audio_tx_buffer.size(); }

void VmuPro::audio_sample_rate(uint32_t sample_rate) {
  if (!sound_initialized_) {
    logger_.error("Sound not initialized, cannot set sample rate");
    return;
  }
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  // stop the channel
  i2s_channel_disable(audio_tx_handle);
  // update the sample rate
  audio_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle, &audio_std_cfg.clk_cfg);
  // clear the buffer
  xStreamBufferReset(audio_tx_stream);
  // restart the channel
  i2s_channel_enable(audio_tx_handle);
}

void VmuPro::play_audio(const std::vector<uint8_t> &data) { play_audio(data.data(), data.size()); }

void VmuPro::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!sound_initialized_) {
    return;
  }
  // don't block here
  xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
}

#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_audio(uint32_t sample_rate,
                                   const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing dual audio system (ES8388 + ES7210) at {} Hz", sample_rate);

  if (audio_initialized_) {
    logger_.warn("Audio already initialized");
    return true;
  }

  // Wire codec register access over internal I2C
  set_es8388_write(std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3));
  set_es8388_read(std::bind(&espp::I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  set_es7210_write(std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3));
  set_es7210_read(std::bind(&espp::I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  // I2S standard channel for TX (playback)
  i2s_chan_config_t chan_cfg_tx = {
      .id = I2S_NUM_0,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = 16,
      .dma_frame_num = 48,
      .auto_clear = true,
      .auto_clear_before_cb = false,
      .allow_pd = false,
      .intr_priority = 0,
  };
  logger_.info("Creating I2S channel for playback (TX)");
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_tx, &audio_tx_handle, nullptr));

  // Optional RX channel for recording (ES7210)
  i2s_chan_config_t chan_cfg_rx = chan_cfg_tx;
  logger_.info("Creating I2S channel for recording (RX)");
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg_rx, nullptr, &audio_rx_handle));

  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
      .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = audio_mclk_io,
                   .bclk = audio_sclk_io,
                   .ws = audio_lrck_io,
                   .dout = audio_asdout_io,
                   .din = audio_dsdin_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  audio_std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
  logger_.info("Configuring I2S standard mode with sample rate {} Hz", sample_rate);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_rx_handle, &audio_std_cfg));

  // ES8388 DAC playback config
  audio_hal_codec_config_t es8388_cfg{};
  es8388_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  es8388_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es8388_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es8388_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  es8388_cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
  logger_.info("Initializing ES8388 codec for playback (DAC) at {} Hz", sample_rate);
  if (es8388_init(&es8388_cfg) != ESP_OK) {
    logger_.error("ES8388 init failed");
    return false;
  }
  if (es8388_config_fmt(ES_MODULE_DAC, ES_I2S_NORMAL) != ESP_OK) {
    logger_.error("ES8388 format config failed");
  }
  if (es8388_set_bits_per_sample(ES_MODULE_DAC, BIT_LENGTH_16BITS) != ESP_OK) {
    logger_.error("ES8388 bps config failed");
  }
  es8388_set_voice_volume(static_cast<int>(volume_));
  es8388_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

  // ES7210 ADC recording config
  audio_hal_codec_config_t es7210_cfg{};
  es7210_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  es7210_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es7210_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es7210_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  es7210_cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;
  logger_.info("Initializing ES7210 codec for recording (ADC) at {} Hz", sample_rate);
  if (es7210_adc_init(&es7210_cfg) != ESP_OK) {
    logger_.error("ES7210 init failed");
    return false;
  }
  if (es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &es7210_cfg.i2s_iface) != ESP_OK) {
    logger_.error("ES7210 I2S cfg failed");
  }
  es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

  // Stream buffers and task
  auto tx_buf_size = calc_audio_buffer_size(sample_rate);
  audio_tx_buffer.resize(tx_buf_size);
  audio_tx_stream = xStreamBufferCreate(tx_buf_size * 4, 0);
  xStreamBufferReset(audio_tx_stream);
  // RX buffer for recording
  audio_rx_buffer.resize(tx_buf_size);
  logger_.info("Enabling I2S channels for playback and recording");
  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  logger_.info("Creating audio task for playback and recording");
  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique(
      {.callback = std::bind(&M5StackTab5::audio_task_callback, this, _1, _2, _3),
       .task_config = task_config});

  enable_audio(true);

  audio_initialized_ = true;

  return audio_task_->start();
}

void M5StackTab5::enable_audio(bool enable) {
  set_speaker_enabled(enable);
  logger_.debug("Audio {}", enable ? "enabled" : "disabled");
}

void M5StackTab5::volume(float volume) {
  volume = std::max(0.0f, std::min(100.0f, volume));
  volume_ = volume;
  es8388_set_voice_volume(static_cast<int>(volume_));
  logger_.debug("Volume set to {:.1f} %", volume);
}

float M5StackTab5::volume() const { return volume_; }

void M5StackTab5::mute(bool mute) {
  mute_ = mute;
  es8388_set_voice_mute(mute_);
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool M5StackTab5::is_muted() const { return mute_; }

void M5StackTab5::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!audio_initialized_ || !data || num_bytes == 0) {
    return;
  }
  xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
}

void M5StackTab5::play_audio(const std::vector<uint8_t> &data) {
  play_audio(data.data(), data.size());
}

bool M5StackTab5::start_audio_recording(
    std::function<void(const uint8_t *data, size_t length)> callback) {
  if (!audio_initialized_) {
    logger_.error("Audio system not initialized");
    return false;
  }
  audio_rx_callback_ = callback;
  recording_ = true;
  logger_.info("Audio recording started");
  return true;
}

void M5StackTab5::stop_audio_recording() {
  recording_ = false;
  logger_.info("Audio recording stopped");
}

bool M5StackTab5::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                      bool &task_notified) {
  // Playback: write next buffer worth of audio from stream buffer
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *tx_buf = audio_tx_buffer.data();
  if (available == 0) {
    memset(tx_buf, 0, buffer_size);
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
    if (available < buffer_size)
      memset(tx_buf + available, 0, buffer_size - available);
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  }

  // Recording: read from RX channel and invoke callback
  if (recording_) {
    size_t bytes_read = 0;
    i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(), &bytes_read,
                     0);
    if (bytes_read > 0 && audio_rx_callback_) {
      audio_rx_callback_(audio_rx_buffer.data(), bytes_read);
    }
  }

  // Sleep ~1 frame at 60 Hz
  {
    using namespace std::chrono_literals;
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, 16ms);
  }
  return false;
}

} // namespace espp

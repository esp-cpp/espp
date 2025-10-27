#include "m5stack-tab5.hpp"

////////////////////////
// Audio Functions   //
////////////////////////

static TaskHandle_t play_audio_task_handle_ = NULL;

static bool IRAM_ATTR audio_tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event,
                                             void *user_ctx) {
  // notify the main task that we're done
  vTaskNotifyGiveFromISR(play_audio_task_handle_, NULL);
  return true;
}

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
  logger_.info("Creating I2S channel for playback (TX)");
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, &audio_rx_handle));

  // Configure I2S for stereo output (needed for proper ES8388 operation)
  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
      .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = audio_mclk_io,
                   .bclk = audio_sclk_io,
                   .ws = audio_lrck_io,
                   .dout = audio_dsdin_io, // ES8388 DSDIN (playback data input)
                   .din = audio_asdout_io, // ES7210 ASDOUT (record data output)
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  logger_.info("Configuring I2S standard mode with sample rate {} Hz", sample_rate);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  // ES8388 DAC playback config
  audio_hal_codec_config_t es8388_cfg{};
  es8388_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  es8388_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL; // Enable both L and R outputs
  // es8388_cfg.adc_input = AUDIO_HAL_ADC_INPUT_LINE1;  // Not used for playback but set anyway
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

  // Configure DAC output routing
  if (es8388_config_dac_output(DAC_OUTPUT_ALL) != ESP_OK) {
    logger_.error("ES8388 DAC output config failed");
  }

  // Set initial volume and unmute
  es8388_set_voice_volume(static_cast<int>(volume_));
  es8388_set_voice_mute(false); // Make sure it's not muted
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

  // Optional RX channel for recording (ES7210)
  logger_.info("Creating I2S channel for recording (RX)");
  i2s_tdm_config_t tdm_cfg = {
      .clk_cfg =
          {
              .sample_rate_hz = (uint32_t)48000,
              .clk_src = I2S_CLK_SRC_DEFAULT,
              .ext_clk_freq_hz = 0,
              .mclk_multiple = I2S_MCLK_MULTIPLE_256,
              .bclk_div = 8,
          },
      .slot_cfg = {.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                   .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
                   .slot_mode = I2S_SLOT_MODE_STEREO,
                   .slot_mask = (i2s_tdm_slot_mask_t)(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 |
                                                      I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
                   .ws_width = I2S_TDM_AUTO_WS_WIDTH,
                   .ws_pol = false,
                   .bit_shift = true,
                   .left_align = false,
                   .big_endian = false,
                   .bit_order_lsb = false,
                   .skip_mask = false,
                   .total_slot = I2S_TDM_AUTO_SLOT_NUM},
      .gpio_cfg = {.mclk = audio_mclk_io,
                   .bclk = audio_sclk_io,
                   .ws = audio_lrck_io,
                   .dout = audio_dsdin_io, // ES8388 DSDIN (playback data input)
                   .din = audio_asdout_io, // ES7210 ASDOUT (record data output)
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(audio_rx_handle, &tdm_cfg));

  // Register TX done callback
  memset(&audio_tx_callbacks_, 0, sizeof(audio_tx_callbacks_));
  audio_tx_callbacks_.on_sent = audio_tx_sent_callback;
  i2s_channel_register_event_callback(audio_tx_handle, &audio_tx_callbacks_, NULL);

  // now enable both channels
  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  play_audio_task_handle_ = xTaskGetCurrentTaskHandle();

  // Stream buffers and task
  auto tx_buf_size = calc_audio_buffer_size(sample_rate);
  audio_tx_buffer.resize(tx_buf_size);
  audio_tx_stream = xStreamBufferCreate(tx_buf_size * 4, 0);
  xStreamBufferReset(audio_tx_stream);
  // RX buffer for recording
  audio_rx_buffer.resize(tx_buf_size);

  logger_.info("Creating audio task for playback and recording");
  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique(
      {.callback = std::bind(&M5StackTab5::audio_task_callback, this, _1, _2, _3),
       .task_config = task_config});

  // Enable speaker output
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
  if (has_sound) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
  has_sound = true;
}

void M5StackTab5::play_audio(std::span<const uint8_t> data) {
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
  memset(tx_buf, 0, buffer_size);
  if (available == 0) {
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
    i2s_channel_write(audio_tx_handle, tx_buf, available, NULL, portMAX_DELAY);
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
  return false;
}

uint32_t M5StackTab5::audio_sample_rate() const { return audio_std_cfg.clk_cfg.sample_rate_hz; }

size_t M5StackTab5::audio_buffer_size() const { return audio_tx_buffer.size(); }

void M5StackTab5::audio_sample_rate(uint32_t sample_rate) {
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

} // namespace espp

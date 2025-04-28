#include "t-deck.hpp"

using namespace espp;

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

bool TDeck::initialize_i2s(uint32_t default_audio_rate) {
  logger_.info("initializing i2s driver");
  logger_.debug("Using newer I2S standard");
  i2s_chan_config_t chan_cfg = {
      .id = i2s_port,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = 16,  // TODO: calculate form audio rate
      .dma_frame_num = 48, // TODO: calculate from audio rate
      .auto_clear = true,
      .auto_clear_before_cb = false,
      .allow_pd = false,
      .intr_priority = 0,
  };

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, nullptr));

  audio_std_cfg = {
      .clk_cfg =
          {
              .sample_rate_hz = default_audio_rate,
              .clk_src = I2S_CLK_SRC_DEFAULT,
              .ext_clk_freq_hz = 0,
              .mclk_multiple = I2S_MCLK_MULTIPLE_256,
          },
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = GPIO_NUM_NC, // i2s_mck_io,
              .bclk = i2s_bck_io,
              .ws = i2s_ws_io,
              .dout = i2s_do_io,
              .din = GPIO_NUM_NC, // i2s_di_io,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  audio_std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  audio_tx_buffer.resize(buffer_size);

  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 0);

  play_audio_task_handle_ = xTaskGetCurrentTaskHandle();

  memset(&audio_tx_callbacks_, 0, sizeof(audio_tx_callbacks_));
  audio_tx_callbacks_.on_sent = audio_tx_sent_callback;
  i2s_channel_register_event_callback(audio_tx_handle, &audio_tx_callbacks_, NULL);

  xStreamBufferReset(audio_tx_stream);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  return true;
}

bool TDeck::initialize_sound(uint32_t default_audio_rate,
                             const espp::Task::BaseConfig &task_config) {
  if (sound_initialized_) {
    logger_.warn("Sound already initialized");
    return true;
  }

  logger_.info("Initializing sound");

  if (!initialize_i2s(default_audio_rate)) {
    logger_.error("Could not initialize I2S driver");
    return false;
  }

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&TDeck::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  sound_initialized_ = true;

  return audio_task_->start();
}

bool TDeck::audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  // Queue the next I2S out frame to write
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *buffer = &audio_tx_buffer[0];
  memset(buffer, 0, buffer_size);

  if (available == 0) {
    i2s_channel_write(audio_tx_handle, buffer, buffer_size, NULL, portMAX_DELAY);
  } else {
    // get the data from the buffer
    xStreamBufferReceive(audio_tx_stream, buffer, available, 0);

    // apply volume and mute
    if (mute_) {
      memset(buffer, 0, available);
    } else {
      // NOTE: we are actually using int16_t samples, so we need to adjust the
      // pointers to the buffer to the correct positions.
      int16_t *ptr = reinterpret_cast<int16_t *>(buffer);
      for (int i = 0; i < (available / 2); ++i) {
        int16_t sample = ptr[i];
        ptr[i] = static_cast<int16_t>((sample * volume_) / 100.0f);
      }
    }

    // write the buffer to the I2S channel
    i2s_channel_write(audio_tx_handle, buffer, available, NULL, portMAX_DELAY);
  }
  return false; // don't stop the task
}

void TDeck::mute(bool mute) { mute_ = mute; }

bool TDeck::is_muted() const { return mute_; }

void TDeck::volume(float volume) { volume_ = std::clamp(volume, 0.0f, 100.0f); }

float TDeck::volume() const { return volume_; }

uint32_t TDeck::audio_sample_rate() const { return audio_std_cfg.clk_cfg.sample_rate_hz; }

size_t TDeck::audio_buffer_size() const { return audio_tx_buffer.size(); }

void TDeck::audio_sample_rate(uint32_t sample_rate) {
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

void TDeck::play_audio(const std::vector<uint8_t> &data) { play_audio(data.data(), data.size()); }

void TDeck::play_audio(const uint8_t *data, uint32_t num_bytes) {
  play_audio_task_handle_ = xTaskGetCurrentTaskHandle();
  if (has_sound) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  // don't block here
  xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
  has_sound = true;
}

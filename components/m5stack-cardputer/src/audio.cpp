#include "m5stack-cardputer.hpp"

#include <cstring>

using namespace espp;

////////////////////////
// Audio Functions    //
////////////////////////

bool M5StackCardputer::initialize_i2s(uint32_t default_audio_rate) {
  // original Cardputer only: transmit-only standard I2S into the NS4168
  // amplifier (the ADV uses the full-duplex path in ensure_adv_i2s())
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

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  return true;
}

bool M5StackCardputer::ensure_adv_i2s(uint32_t sample_rate) {
  // ADV only: the ES8311 codec is a full-duplex device on a single I2S bus
  // (shared bit/word clocks; DAC data on dout, ADC data on din), so both the
  // transmit and receive channels are allocated together on one port. The
  // channels are enabled separately by initialize_sound() /
  // initialize_microphone().
  if (audio_tx_handle != nullptr) {
    // already created by whichever of sound / microphone was initialized
    // first; the clock is shared, so a different requested rate cannot be
    // honored
    if (sample_rate != audio_std_cfg.clk_cfg.sample_rate_hz) {
      logger_.warn("I2S already configured at {} Hz; ignoring requested {} Hz (the ADV speaker "
                   "and microphone share the I2S clock)",
                   audio_std_cfg.clk_cfg.sample_rate_hz, sample_rate);
    }
    return true;
  }

  logger_.info("initializing full-duplex i2s driver at {} Hz", sample_rate);

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, &audio_rx_handle));

  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
      .gpio_cfg = {.mclk = GPIO_NUM_NC,
                   .bclk = i2s_bck_io,
                   .ws = i2s_ws_io,
                   .dout = i2s_do_io,
                   .din = mic_data_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  // Receive in stereo (both 16-bit slots of every frame) even though the
  // codec's ADC is mono: a mono RX slot configuration in this full-duplex
  // setup does not deliver one sample per frame (recordings came out with
  // every sample doubled, i.e. at half speed and an octave low). Capturing
  // both slots gives a deterministic L,R word layout; the microphone task
  // keeps only the left slot, where the ES8311 drives its ADC data. The
  // transmit side is unaffected.
  i2s_std_config_t rx_cfg = audio_std_cfg;
  rx_cfg.slot_cfg =
      I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_rx_handle, &rx_cfg));

  return true;
}

bool M5StackCardputer::initialize_sound(uint32_t default_audio_rate,
                                        const espp::Task::BaseConfig &task_config) {
  if (sound_initialized_) {
    logger_.warn("Sound already initialized");
    return true;
  }

  bool is_adv = variant() == Variant::ADV;

  if (is_adv) {
    // full duplex with the microphone; create (or reuse) the shared channels
    if (!ensure_adv_i2s(default_audio_rate)) {
      logger_.error("Could not initialize I2S driver");
      return false;
    }
  } else {
    // the original cannot run the speaker and PDM microphone at the same
    // time: GPIO 43 doubles as the speaker WS and the microphone clock
    if (microphone_initialized_) {
      logger_.error("Cannot initialize sound: the microphone is active and shares GPIO {} with "
                    "the speaker (WS / PDM clock)",
                    static_cast<int>(i2s_ws_io));
      return false;
    }
    if (!initialize_i2s(default_audio_rate)) {
      logger_.error("Could not initialize I2S driver");
      return false;
    }
  }

  // size the transmit buffering from the actual (possibly shared) rate
  auto buffer_size = calc_audio_buffer_size(audio_sample_rate());
  audio_tx_buffer.resize(buffer_size);
  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 0);
  xStreamBufferReset(audio_tx_stream);

  if (is_adv) {
    ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));
    // On the ADV the I2S signals go through the ES8311 codec (then an
    // NS4150B amplifier), so the codec's DAC path must be configured. Do
    // this after the I2S channel is enabled since the codec derives its
    // clock from BCLK.
    if (!initialize_es8311_speaker()) {
      logger_.error("Could not initialize the ES8311 codec");
      i2s_channel_disable(audio_tx_handle);
      if (!microphone_initialized_) {
        // the channels are shared with the microphone; only tear them down
        // if it is not using them
        i2s_del_channel(audio_tx_handle);
        i2s_del_channel(audio_rx_handle);
        audio_tx_handle = nullptr;
        audio_rx_handle = nullptr;
      }
      vStreamBufferDelete(audio_tx_stream);
      audio_tx_stream = nullptr;
      audio_tx_buffer.clear();
      return false;
    }
  }

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&M5StackCardputer::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  sound_initialized_ = true;

  return audio_task_->start();
}

bool M5StackCardputer::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                           bool &task_notified) {
  // Queue the next I2S out frame to write
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *buffer = &audio_tx_buffer[0];
  memset(buffer, 0, buffer_size);

  if (available > 0) {
    xStreamBufferReceive(audio_tx_stream, buffer, available, 0);
    // The amplifier has no volume control, so scale the samples in software
    // according to the current volume / mute state
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

void M5StackCardputer::mute(bool mute) { mute_ = mute; }

bool M5StackCardputer::is_muted() const { return mute_; }

void M5StackCardputer::volume(float volume) { volume_ = std::clamp(volume, 0.0f, 100.0f); }

float M5StackCardputer::volume() const { return volume_; }

uint32_t M5StackCardputer::audio_sample_rate() const {
  return audio_std_cfg.clk_cfg.sample_rate_hz;
}

size_t M5StackCardputer::audio_buffer_size() const { return audio_tx_buffer.size(); }

void M5StackCardputer::audio_sample_rate(uint32_t sample_rate) {
  if (!sound_initialized_) {
    logger_.error("Sound not initialized, cannot set sample rate");
    return;
  }
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  bool reconfigure_rx = variant() == Variant::ADV && microphone_initialized_;
  // stop the channel(s); on the ADV the microphone shares the clock, so its
  // channel must be reconfigured as well
  i2s_channel_disable(audio_tx_handle);
  if (reconfigure_rx) {
    i2s_channel_disable(audio_rx_handle);
  }
  // update the sample rate
  audio_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle, &audio_std_cfg.clk_cfg);
  if (reconfigure_rx) {
    i2s_channel_reconfig_std_clock(audio_rx_handle, &audio_std_cfg.clk_cfg);
    mic_sample_rate_ = sample_rate;
  }
  // clear the buffer
  xStreamBufferReset(audio_tx_stream);
  // restart the channel(s)
  i2s_channel_enable(audio_tx_handle);
  if (reconfigure_rx) {
    i2s_channel_enable(audio_rx_handle);
  }
}

size_t M5StackCardputer::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

size_t M5StackCardputer::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!sound_initialized_) {
    return 0;
  }
  // don't block here; report how much was actually queued so callers can
  // stream data larger than the buffer
  return xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
}

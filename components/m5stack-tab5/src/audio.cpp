#include <algorithm>
#include <cmath>

#include "m5stack-tab5.hpp"

////////////////////////
// Audio Functions   //
////////////////////////

namespace espp {

// Map a 0-100% microphone volume onto the ES7210's analog gain steps
// (GAIN_0DB .. GAIN_37_5DB)
static es7210_gain_value_t microphone_gain_from_volume(float volume) {
  int step = static_cast<int>(std::lround(volume / 100.0f * static_cast<float>(GAIN_37_5DB)));
  step = std::clamp(step, static_cast<int>(GAIN_0DB), static_cast<int>(GAIN_37_5DB));
  return static_cast<es7210_gain_value_t>(step);
}

// Map a PCM sample rate onto the codec's audio_hal sample-rate enum. The ES7210
// coefficient table is selected by this enum, so it must match the rate the I2S
// clocks actually generate; an unsupported rate falls back to 48 kHz.
static audio_hal_iface_samples_t audio_hal_samples_from_rate(uint32_t rate) {
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

bool M5StackTab5::initialize_audio(uint32_t sample_rate,
                                   const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing dual audio system (ES8388 + ES7210) at {} Hz", sample_rate);

  if (audio_initialized_) {
    logger_.warn("Audio already initialized");
    return true;
  }

  // I2S standard channel for TX (playback)
  logger_.info("Creating I2S channel for playback (TX)");
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, &audio_rx_handle));

  // Configure I2S for stereo output (needed for proper ES8388 operation). The
  // playback data is interleaved 16-bit stereo, so the slot mode MUST be stereo
  // — a mono slot plays interleaved stereo at the wrong rate and garbles L/R.
  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
      .slot_cfg =
          I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = audio_mclk_io,
                   .bclk = audio_sclk_io,
                   .ws = audio_lrck_io,
                   .dout = audio_dsdin_io, // ES8388 DSDIN (playback data input)
                   .din = audio_asdout_io, // ES7210 ASDOUT (record data output)
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  logger_.info("Configuring I2S standard mode with sample rate {} Hz", sample_rate);
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  // RX channel for recording (ES7210). The ES7210 is configured below for
  // standard I2S output (16-bit stereo: microphone 1 on the left slot,
  // microphone 2 on the right), and in full-duplex mode the RX module shares
  // the TX BCLK/WS, so the RX channel must use the same standard-mode
  // configuration - a different frame geometry (e.g. multi-slot TDM) would
  // not match the shared clock.
  logger_.info("Configuring I2S channel for recording (RX)");
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_rx_handle, &audio_std_cfg));

  // Stream buffers and task
  auto tx_buf_size = calc_audio_buffer_size(sample_rate);
  audio_tx_buffer.resize(tx_buf_size);
  audio_tx_stream = xStreamBufferCreate(tx_buf_size * 4, 0);
  xStreamBufferReset(audio_tx_stream);
  // RX buffer for recording
  audio_rx_buffer.resize(tx_buf_size);

  // now enable both channels
  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));
  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  std::error_code ec;
  es8388_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = 0x10,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!es8388_i2c_device_) {
    logger_.error("Could not initialize ES8388 I2C device: {}", ec.message());
    return false;
  }
  es7210_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = 0x40,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!es7210_i2c_device_) {
    logger_.error("Could not initialize ES7210 I2C device: {}", ec.message());
    return false;
  }

  // Wire codec register access over internal I2C
  set_es8388_write(espp::make_i2c_addressed_write(es8388_i2c_device_));
  set_es8388_read(espp::make_i2c_addressed_read_register(es8388_i2c_device_));
  set_es7210_write(espp::make_i2c_addressed_write(es7210_i2c_device_));
  set_es7210_read(espp::make_i2c_addressed_read_register(es7210_i2c_device_));

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
  // configure the ES7210 for the rate the I2S clocks are actually running at
  // (rather than assuming 48 kHz) so its coefficient table matches
  es7210_cfg.i2s_iface.samples = audio_hal_samples_from_rate(sample_rate);
  logger_.info("Initializing ES7210 codec for recording (ADC) at {} Hz", sample_rate);
  if (es7210_adc_init(&es7210_cfg) != ESP_OK) {
    logger_.error("ES7210 init failed");
    return false;
  }
  if (es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &es7210_cfg.i2s_iface) != ESP_OK) {
    logger_.error("ES7210 I2S cfg failed");
  }
  // Enable the ES7210 digital high-pass filter on the ADC channels to strip
  // the microphone DC offset. The espp es7210 driver leaves these registers
  // at their reset value (HPF off), so the DC offset is amplified by the
  // analog gain and rails the ADC to full scale - recordings come out as a
  // near-constant DC level that the AC-coupled speaker plays as a click then
  // silence. These are the driver's documented "quick setup" HPF values.
  std::error_code hpf_ec;
  es7210_i2c_device_->write_register(0x22, std::vector<uint8_t>{0x0a}, hpf_ec); // ADC1/2 HPF1
  es7210_i2c_device_->write_register(0x23, std::vector<uint8_t>{0x2a}, hpf_ec); // ADC1/2 HPF2
  es7210_i2c_device_->write_register(0x20, std::vector<uint8_t>{0x0a}, hpf_ec); // ADC3/4 HPF2
  es7210_i2c_device_->write_register(0x21, std::vector<uint8_t>{0x2a}, hpf_ec); // ADC3/4 HPF1
  if (hpf_ec) {
    logger_.warn("Could not enable the ES7210 high-pass filter: {}", hpf_ec.message());
  }
  // apply the stored microphone volume (the driver's init leaves the analog
  // gain at its 0 dB minimum, which records very quietly)
  es7210_adc_set_gain_all(microphone_gain_from_volume(mic_volume_));
  es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

  // Create two independent tasks, matching the (well-behaved) esp-box audio
  // path: one drains the playback stream buffer to the TX channel, and a
  // separate task blocks on the RX channel to read the microphone. Keeping the
  // record read off the playback task means the RX DMA ring is drained at
  // exactly the rate samples arrive (rather than only between blocking TX
  // writes), which the shared-task approach did irregularly - on this
  // full-duplex controller that jitter degraded the recording and let the ring
  // back up.
  logger_.info("Creating audio playback and microphone tasks");
  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique(
      {.callback = std::bind(&M5StackTab5::audio_task_callback, this, _1, _2, _3),
       .task_config = task_config});
  // give the microphone task a distinct name from the playback task (they share
  // the rest of the caller's task config) so the two are easy to tell apart in
  // logs and while debugging
  auto mic_task_config = task_config;
  mic_task_config.name =
      task_config.name.empty() ? std::string("microphone") : task_config.name + " microphone";
  microphone_task_ = espp::Task::make_unique(
      {.callback = std::bind(&M5StackTab5::microphone_task_callback, this, _1, _2, _3),
       .task_config = mic_task_config});

  // Enable speaker output
  enable_audio(true);

  // Start the tasks stepwise so a failure does not leave a half-initialized
  // state: if the microphone task fails to start, stop the already-running
  // playback task and report failure rather than leaving it running with
  // audio_initialized_ set.
  if (!audio_task_->start()) {
    logger_.error("Failed to start the audio playback task");
    return false;
  }
  if (!microphone_task_->start()) {
    logger_.error("Failed to start the microphone task");
    audio_task_->stop();
    return false;
  }

  audio_initialized_ = true;
  return true;
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

size_t M5StackTab5::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!audio_initialized_ || !data || num_bytes == 0) {
    return 0;
  }
  // only enqueue whole 16-bit stereo frames (4 bytes) so a partial sample
  // cannot shift the L/R framing or strand 1-3 bytes in the stream buffer
  num_bytes -= num_bytes % 4;
  if (num_bytes == 0) {
    return 0;
  }
  // Don't block here: append what fits into the stream buffer and report how
  // much was actually queued so callers can stream data larger than the buffer.
  // The audio task drains it to the I2S peripheral. This runs in task context,
  // so use the non-ISR send with a 0 timeout (never blocks) rather than the
  // FromISR variant.
  return xStreamBufferSend(audio_tx_stream, data, num_bytes, 0);
}

size_t M5StackTab5::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

void M5StackTab5::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (audio_initialized_) {
    es7210_adc_set_gain_all(microphone_gain_from_volume(mic_volume_));
  }
}

float M5StackTab5::microphone_volume() const { return mic_volume_; }

bool M5StackTab5::start_audio_recording(
    std::function<void(const uint8_t *data, size_t length)> callback) {
  if (!audio_initialized_) {
    logger_.error("Audio system not initialized");
    return false;
  }
  // Reject a start while a recording is already in progress: the microphone
  // task reads audio_rx_callback_ whenever recording_ is true, so reassigning
  // the callback here (without stopping first) would race that task. The caller
  // must stop_audio_recording() before starting a new one.
  if (recording_) {
    logger_.warn("Recording already in progress; stop it before starting a new one");
    return false;
  }
  // The microphone task drains the RX ring continuously, so there is no stale
  // backlog to flush here (and flushing would race that task on the same RX
  // handle). Install the callback before arming recording_: the mic task only
  // reads the callback once it observes recording_ == true, and the release/
  // acquire on that atomic publishes the callback write to it.
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
  // Queue the next I2S out frame to write (matches the esp-box / t-deck path):
  // always write a full, frame-aligned buffer with the queued samples zero-
  // padded up to buffer_size so the I2S DMA is fed at a constant cadence.
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *tx_buf = audio_tx_buffer.data();
  memset(tx_buf, 0, buffer_size);
  if (available == 0) {
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
    i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, NULL, portMAX_DELAY);
  }

  return false;
}

bool M5StackTab5::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                           bool &task_notified) {
  // Block until the RX channel has a buffer of microphone samples. This runs on
  // its own task so the ES7210's RX DMA ring is always drained at the rate
  // samples arrive - in full-duplex RX and TX share one I2S controller, so
  // letting the ring back up perturbs the TX (pops the playback). Only forward
  // the samples to the callback while a recording is in progress; otherwise the
  // read still drains the ring and the data is discarded.
  size_t bytes_read = 0;
  // Use a finite read timeout (not portMAX_DELAY) so this task returns
  // periodically and can observe a stop request; an infinite read would block
  // Task::stop() from joining during teardown.
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, pdMS_TO_TICKS(100));
  if (err == ESP_OK && bytes_read > 0 && recording_ && audio_rx_callback_) {
    audio_rx_callback_(audio_rx_buffer.data(), bytes_read);
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
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

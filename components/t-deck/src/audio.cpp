#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <mutex>

#include <driver/i2s_tdm.h>

#include "t-deck.hpp"

using namespace espp;

////////////////////////
// Audio Functions   //
////////////////////////

// Map a 0-100% microphone volume onto the ES7210's analog gain steps
// (GAIN_0DB .. GAIN_37_5DB)
static es7210_gain_value_t microphone_gain_from_volume(float volume) {
  int step = static_cast<int>(std::lround(volume / 100.0f * static_cast<float>(GAIN_37_5DB)));
  step = std::clamp(step, static_cast<int>(GAIN_0DB), static_cast<int>(GAIN_37_5DB));
  return static_cast<es7210_gain_value_t>(step);
}

// Map a PCM sample rate onto the codec's audio_hal sample-rate enum. The ES7210
// coefficient table is selected by this enum, so it must match the rate the I2S
// clocks actually generate; an unsupported rate falls back to 16 kHz (the rate
// this board records at).
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
    return AUDIO_HAL_16K_SAMPLES;
  }
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
              .bclk_div = 8,
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
        int32_t sample = (ptr[i] * volume_) / 100.0f;
        ptr[i] = static_cast<int16_t>(std::clamp<int32_t>(sample, INT16_MIN, INT16_MAX));
      }
    }

    // write the buffer to the I2S channel
    i2s_channel_write(audio_tx_handle, buffer, buffer_size, NULL, portMAX_DELAY);
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

size_t TDeck::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

size_t TDeck::play_audio(const uint8_t *data, uint32_t num_bytes) {
  // guard against being called before initialize_sound() (audio_tx_stream is
  // not valid until then) and against empty input
  if (!sound_initialized_ || !data || num_bytes == 0) {
    return 0;
  }
  // Enqueue only whole 16-bit stereo frames (4 bytes) that actually fit:
  // xStreamBufferSend can accept fewer bytes than requested when the buffer is
  // nearly full, and a partial (non-frame-aligned) send would strand 1-3 bytes
  // and corrupt the L/R framing of subsequent appends. Cap the request to the
  // free space rounded down to a whole frame so the send is always
  // frame-aligned. This runs in task context, so the non-ISR send with a 0
  // timeout never blocks. The number of bytes actually queued is returned so
  // callers can stream data larger than the buffer.
  size_t sendable = std::min<size_t>(num_bytes, xStreamBufferSpacesAvailable(audio_tx_stream));
  sendable -= sendable % 4;
  if (sendable == 0) {
    return 0;
  }
  return xStreamBufferSend(audio_tx_stream, data, sendable, 0);
}

//////////////////////////
// Microphone Functions //
//////////////////////////

bool TDeck::initialize_microphone(const microphone_callback_t &callback, uint32_t sample_rate,
                                  const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (microphone_initialized_) {
    logger_.warn("Microphone already initialized, not initializing again!");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  microphone_callback_ = callback;

  // The ES7210 has its own I2S bus (separate from the speaker amplifier), so
  // make a receive-only master channel with its own clocks; the codec is a
  // slave clocked from the MCLK / SCLK / LRCK we generate.
  //
  // The ES7210 outputs its four ADC channels as a 4-slot TDM frame (MIC1 ->
  // slot 0, MIC2 -> slot 1, MIC3 -> slot 2, MIC4 -> slot 3), so read all four
  // slots. The microphone task extracts the populated microphones (the T-Deck
  // wires MIC1 and MIC3, i.e. slots 0 and 2 - one from each ADC pair) and
  // presents them as 16-bit stereo. A 2-slot standard-I2S read would only see
  // the first ADC pair (MIC1 + the unpopulated MIC2).
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(mic_i2s_port, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &audio_rx_handle));
  i2s_tdm_config_t mic_tdm_cfg = {
      .clk_cfg = {.sample_rate_hz = sample_rate,
                  .clk_src = I2S_CLK_SRC_DEFAULT,
                  .ext_clk_freq_hz = 0,
                  .mclk_multiple = I2S_MCLK_MULTIPLE_256,
                  // Let the driver derive BCLK from the slot count/width. The
                  // ES7210 coefficient table is programmed for MCLK = 256 x
                  // sample_rate (see MCLK_DIV_FRE in es7210.cpp), which
                  // mclk_multiple pins here. Forcing bclk_div (e.g. 8) makes
                  // the driver raise MCLK to satisfy the divider, so the codec
                  // receives an MCLK its coefficients were not computed for;
                  // its ADC decimation then drifts against LRCK and slips a
                  // frame every so often, which reads as sparse full-scale
                  // (0x8000) spikes on top of otherwise-clean audio.
                  .bclk_div = 0},
      .slot_cfg = {.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
                   .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
                   .slot_mode = I2S_SLOT_MODE_STEREO,
                   .slot_mask = static_cast<i2s_tdm_slot_mask_t>(I2S_TDM_SLOT0 | I2S_TDM_SLOT1 |
                                                                 I2S_TDM_SLOT2 | I2S_TDM_SLOT3),
                   .ws_width = I2S_TDM_AUTO_WS_WIDTH,
                   .ws_pol = false,
                   .bit_shift = true,
                   .left_align = false,
                   .big_endian = false,
                   .bit_order_lsb = false,
                   .skip_mask = false,
                   .total_slot = I2S_TDM_AUTO_SLOT_NUM},
      .gpio_cfg = {.mclk = es7210_mclk_io,
                   .bclk = es7210_sclk_io,
                   .ws = es7210_lrck_io,
                   .dout = GPIO_NUM_NC,
                   .din = es7210_sdout_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(audio_rx_handle, &mic_tdm_cfg));
  mic_sample_rate_ = sample_rate;
  // the read buffer holds the raw 4-slot TDM frames (twice the size of the
  // 2-channel stereo the callback receives)
  audio_rx_buffer.resize(2 * calc_audio_buffer_size(sample_rate));
  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  // Configure the ES7210 ADC now that its clocks are running. The ES7210 always
  // emits a fixed 4-slot TDM frame; this board only populates MIC1 (slot 0) and
  // MIC3 (slot 2), which the callback compacts to the left / right channels of
  // the delivered 16-bit stereo.
  std::error_code ec;
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
    i2s_channel_disable(audio_rx_handle);
    i2s_del_channel(audio_rx_handle);
    audio_rx_handle = nullptr;
    return false;
  }
  set_es7210_write(espp::make_i2c_addressed_write(es7210_i2c_device_));
  set_es7210_read(espp::make_i2c_addressed_read_register(es7210_i2c_device_));

  // Enable all four ADC channels so the ES7210 always emits a fixed 4-slot
  // TDM frame ([MIC1, MIC2, MIC3, MIC4]). Enabling only a subset can make the
  // codec pack fewer slots, which shifts the frame boundary against the fixed
  // 4-slot read below and periodically lands the extraction on a floating,
  // railing slot (heard as a robotic / static buzz on top of the audio). The
  // T-Deck only populates MIC1 and MIC3, so the task keeps slots 0 and 2 and
  // discards the unpopulated MIC2 / MIC4 slots.
  es7210_mic_select(static_cast<es7210_input_mics_t>(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 |
                                                     ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4));

  audio_hal_codec_config_t es7210_cfg{};
  es7210_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  es7210_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es7210_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es7210_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  // configure the ES7210 for the requested capture rate (rather than assuming
  // 16 kHz) so its coefficient table matches the I2S clocks
  es7210_cfg.i2s_iface.samples = audio_hal_samples_from_rate(sample_rate);
  if (es7210_adc_init(&es7210_cfg) != ESP_OK) {
    logger_.error("Could not initialize the ES7210 codec");
    i2s_channel_disable(audio_rx_handle);
    i2s_del_channel(audio_rx_handle);
    audio_rx_handle = nullptr;
    return false;
  }
  if (es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &es7210_cfg.i2s_iface) != ESP_OK) {
    logger_.error("ES7210 I2S config failed");
  }
  // NOTE: do not write register 0x08 (master/slave & channels) here. The
  // es7210 driver leaves it at its reset default in slave mode, which is
  // correct for this board. Forcing 0x08 = 0x20 (as some MicroPython T-Deck
  // ports do for their async PWM MCLK) reconfigures the channel/decimation
  // path on our hardware-MCLK setup into a half-rate, every-other-frame-zero
  // capture (MIC1 samples drop out and the glitch/de-spike rate rises), so it
  // is deliberately left alone.
  // Enable the ES7210 digital high-pass filter on the ADC channels to strip
  // the microphone DC offset. The espp es7210 driver leaves these registers at
  // their reset value (HPF off), so the (large, positive) DC offset from the
  // T-Deck's electret mics is amplified by the analog gain and rails the ADC:
  // captures come out all-positive with a big constant bias (e.g. MIC1 min=0
  // max=24672) that reads as a quiet-but-distorted / robotic recording, and at
  // higher gain saturates to full scale. Removing the DC here lets the signal
  // swing symmetrically around zero so the analog gain can be raised safely.
  // These are the driver's documented "quick setup" HPF values, and are what
  // the (working) esp-box microphone path uses. The T-Deck populates MIC1 and
  // MIC3, one from each ADC pair, so both pairs' HPFs are enabled.
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

  using namespace std::placeholders;
  microphone_task_ = espp::Task::make_unique({
      .callback = std::bind(&TDeck::microphone_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  microphone_initialized_ = true;

  return microphone_task_->start();
}

uint32_t TDeck::microphone_sample_rate() const { return mic_sample_rate_; }

bool TDeck::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                     bool &task_notified) {
  size_t bytes_read = 0;
  // Use a finite read timeout (not portMAX_DELAY) so this task returns
  // periodically and can observe a stop request; an infinite read would block
  // Task::stop() from joining during teardown.
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, pdMS_TO_TICKS(100));
  if (err == ESP_OK && bytes_read > 0 && microphone_callback_) {
    // audio_rx_buffer holds 4-slot TDM frames: [MIC1, MIC2, MIC3, MIC4].
    // Compact the populated microphones (MIC1 -> slot 0, MIC3 -> slot 2) down
    // to 16-bit stereo in place (the output region is the front half, so this
    // is safe).
    auto *samples = reinterpret_cast<int16_t *>(audio_rx_buffer.data());
    size_t num_frames = bytes_read / (4 * sizeof(int16_t));
    for (size_t i = 0; i < num_frames; i++) {
      samples[2 * i] = samples[4 * i];         // MIC1 -> left
      samples[2 * i + 1] = samples[4 * i + 2]; // MIC3 -> right
    }
    microphone_callback_(audio_rx_buffer.data(), num_frames * 2 * sizeof(int16_t));
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

void TDeck::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (microphone_initialized_) {
    es7210_adc_set_gain_all(microphone_gain_from_volume(mic_volume_));
  }
}

float TDeck::microphone_volume() const { return mic_volume_; }

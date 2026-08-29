#include "esp32-p4-wifi6-dev-kit.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <driver/gpio.h>

#include "es8311.hpp"

namespace espp {

// Map a requested sample rate onto the ES8311 HAL's supported-rates enum.
// Returns false for rates the codec driver does not support.
static bool es8311_samples_from_rate(uint32_t sample_rate, audio_hal_iface_samples_t &out_samples) {
  switch (sample_rate) {
  case 8000:
    out_samples = AUDIO_HAL_08K_SAMPLES;
    return true;
  case 11025:
    out_samples = AUDIO_HAL_11K_SAMPLES;
    return true;
  case 16000:
    out_samples = AUDIO_HAL_16K_SAMPLES;
    return true;
  case 22050:
    out_samples = AUDIO_HAL_22K_SAMPLES;
    return true;
  case 24000:
    out_samples = AUDIO_HAL_24K_SAMPLES;
    return true;
  case 32000:
    out_samples = AUDIO_HAL_32K_SAMPLES;
    return true;
  case 44100:
    out_samples = AUDIO_HAL_44K_SAMPLES;
    return true;
  case 48000:
    out_samples = AUDIO_HAL_48K_SAMPLES;
    return true;
  default:
    return false;
  }
}

bool Esp32P4Wifi6DevKit::initialize_audio(uint32_t sample_rate,
                                          const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing audio (ES8311) at {} Hz", sample_rate);

  if (audio_initialized_) {
    logger_.warn("Audio already initialized");
    return true;
  }

  // Validate the requested rate up front so the codec's initial clock config
  // (below) matches the I2S clock instead of silently assuming 48 kHz.
  audio_hal_iface_samples_t es8311_samples;
  if (!es8311_samples_from_rate(sample_rate, es8311_samples)) {
    logger_.error("Unsupported audio sample rate {} Hz; supported rates: 8000, 11025, 16000, "
                  "22050, 24000, 32000, 44100, 48000",
                  sample_rate);
    return false;
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

  // Create the I2S standard channels: TX for playback, RX for the ES8311's
  // ADC (initialized on demand by initialize_microphone(); the codec is full
  // duplex on this single bus, sharing the clock). MCLK = 256 * fs (default).
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(audio_i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true;
  if (i2s_new_channel(&chan_cfg, &audio_tx_handle, &audio_rx_handle) != ESP_OK) {
    logger_.error("Failed to create I2S channel");
    return false;
  }

  // Consolidated teardown for the failure paths below: i2s_new_channel() creates
  // both channels together, so on any later failure delete both (and the stream
  // buffer if created) and reset state rather than leaking channels/clocks.
  auto fail_audio_init = [&](const char *msg) -> bool {
    logger_.error("{}", msg);
    if (audio_tx_handle) {
      i2s_channel_disable(audio_tx_handle);
      i2s_del_channel(audio_tx_handle);
      audio_tx_handle = nullptr;
    }
    if (audio_rx_handle) {
      i2s_channel_disable(audio_rx_handle);
      i2s_del_channel(audio_rx_handle);
      audio_rx_handle = nullptr;
    }
    if (audio_tx_stream) {
      vStreamBufferDelete(audio_tx_stream);
      audio_tx_stream = nullptr;
    }
    return false;
  };

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
    return fail_audio_init("Failed to init I2S std mode");
  }

  // Initialize the ES8311 codec for playback (codec is an I2S slave)
  audio_hal_codec_config_t es8311_cfg{};
  es8311_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  es8311_cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL;
  es8311_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es8311_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es8311_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  es8311_cfg.i2s_iface.samples = es8311_samples;
  if (es8311_codec_init(&es8311_cfg) != ESP_OK) {
    return fail_audio_init("ES8311 init failed");
  }
  // The rate was validated above, but propagate a codec failure anyway: if the
  // codec clock config is not applied, playback would be unusable even though
  // initialization "succeeded".
  if (es8311_codec_set_sample_rate(sample_rate) != ESP_OK) {
    return fail_audio_init("ES8311 sample-rate configuration failed");
  }
  // Apply the cached volume/mute state so values set via volume()/mute()
  // before initialization (which only cache) take effect now.
  es8311_codec_set_voice_volume(static_cast<int>(volume_));
  es8311_set_voice_mute(mute_);
  es8311_codec_ctrl_state(AUDIO_HAL_CODEC_MODE_DECODE, AUDIO_HAL_CTRL_START);

  if (i2s_channel_enable(audio_tx_handle) != ESP_OK) {
    return fail_audio_init("Failed to enable I2S channel");
  }

  // The audio task drains this stream buffer to I2S. Size it generously so a
  // whole UI sound clip fits and play_audio() can enqueue it without blocking.
  auto tx_buf_size = calc_audio_buffer_size(sample_rate, TX_NUM_CHANNELS);
  audio_tx_buffer.resize(tx_buf_size);
  audio_tx_stream = xStreamBufferCreate(std::max<size_t>(tx_buf_size * 4, 64 * 1024), 0);
  if (audio_tx_stream == nullptr) {
    return fail_audio_init("Failed to allocate the audio TX stream buffer");
  }
  xStreamBufferReset(audio_tx_stream);

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique(
      {.callback = std::bind(&Esp32P4Wifi6DevKit::audio_task_callback, this, _1, _2, _3),
       .task_config = task_config});

  if (!audio_task_->start()) {
    audio_task_.reset();
    return fail_audio_init("Failed to start the audio task");
  }
  set_speaker_enabled(true);
  audio_initialized_ = true;
  return true;
}

void Esp32P4Wifi6DevKit::set_speaker_enabled(bool enable) {
  gpio_set_level(audio_pa_enable_io, enable ? 1 : 0);
}

void Esp32P4Wifi6DevKit::volume(float volume) {
  volume = std::clamp(volume, 0.0f, 100.0f);
  volume_ = volume;
  // Before initialize_audio() the ES8311 driver has no I2C read/write
  // callbacks installed, so touching the codec would invoke null
  // std::functions and terminate. Just cache the value; initialize_audio()
  // applies it to the codec.
  if (!audio_initialized_) {
    return;
  }
  es8311_codec_set_voice_volume(static_cast<int>(volume_));
}

float Esp32P4Wifi6DevKit::volume() const { return volume_; }

void Esp32P4Wifi6DevKit::mute(bool mute) {
  mute_ = mute;
  // Same pre-initialization guard as volume(): cache only, applied on init.
  if (!audio_initialized_) {
    return;
  }
  es8311_set_voice_mute(mute_);
}

bool Esp32P4Wifi6DevKit::is_muted() const { return mute_; }

size_t Esp32P4Wifi6DevKit::play_audio(const uint8_t *data, uint32_t num_bytes) {
  if (!audio_initialized_ || !data || num_bytes == 0) {
    return 0;
  }
  // Enqueue only whole 16-bit samples (2 bytes; the TX slot is mono) that
  // actually fit: xStreamBufferSend can accept fewer bytes than requested when
  // the buffer is nearly full, and a partial (odd) send would strand a byte and
  // shift framing on subsequent appends. Cap the request to the free space
  // rounded down to a whole sample. This runs in task context, so the non-ISR
  // send with a 0 timeout never blocks; the number of bytes actually queued is
  // returned so callers can stream data larger than the buffer.
  size_t sendable = std::min<size_t>(num_bytes, xStreamBufferSpacesAvailable(audio_tx_stream));
  sendable -= sendable % 2;
  if (sendable == 0) {
    return 0;
  }
  return xStreamBufferSend(audio_tx_stream, data, sendable, 0);
}

void Esp32P4Wifi6DevKit::clear_audio() {
  if (!audio_initialized_ || audio_tx_stream == nullptr) {
    return;
  }
  // Drop everything queued but not yet handed to the I2S DMA. The drain task's
  // current (at most one) frame still finishes, so this cuts over on the next
  // ~16 ms frame boundary.
  xStreamBufferReset(audio_tx_stream);
}

size_t Esp32P4Wifi6DevKit::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

//////////////////////////
// Microphone Functions //
//////////////////////////

// Map a 0-100% microphone volume onto the ES8311's analog microphone gain
// steps (ES8311_MIC_GAIN_0DB .. ES8311_MIC_GAIN_42DB, 6 dB apart)
static es8311_mic_gain_t microphone_gain_from_volume(float volume) {
  int step = static_cast<int>(std::lround(volume / 100.0f * 7.0f));
  step = std::clamp(step, static_cast<int>(ES8311_MIC_GAIN_0DB),
                    static_cast<int>(ES8311_MIC_GAIN_42DB));
  return static_cast<es8311_mic_gain_t>(step);
}

bool Esp32P4Wifi6DevKit::initialize_microphone(const microphone_callback_t &callback,
                                               const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (microphone_initialized_) {
    // Idempotent, matching the other initialize_* methods: calling again is
    // harmless, so warn and report success.
    logger_.warn("Microphone already initialized, not initializing again!");
    return true;
  }
  if (!audio_initialized_) {
    logger_.error("The audio subsystem must be initialized first: the ES8311 is a full-duplex "
                  "codec on a single I2S bus");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  microphone_callback_ = callback;

  // The RX channel shares the TX BCLK/WS in full-duplex mode. Receive in
  // stereo (both 16-bit slots of every frame) even though the codec's ADC is
  // mono: a mono RX slot configuration in this full-duplex setup does not
  // deliver one sample per frame (each sample comes through twice, i.e. at
  // half speed); capturing both slots gives a deterministic L,R word layout
  // and the microphone task keeps only the left slot, where the ES8311
  // drives its ADC data.
  i2s_std_config_t rx_cfg = audio_std_cfg;
  rx_cfg.slot_cfg =
      I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
  // i2s_channel_init_std_mode() may only be called once per channel (it fails
  // on a READY channel), so track whether the RX channel has already been
  // configured by an earlier attempt that failed later (enable / task start).
  // On such a retry, skip the init and just refresh the clock config in case
  // the sample rate changed via audio_sample_rate() in between.
  if (!audio_rx_std_configured_) {
    if (i2s_channel_init_std_mode(audio_rx_handle, &rx_cfg) != ESP_OK) {
      logger_.error("Failed to init I2S RX std mode");
      return false;
    }
    audio_rx_std_configured_ = true;
  } else if (i2s_channel_reconfig_std_clock(audio_rx_handle, &rx_cfg.clk_cfg) != ESP_OK) {
    logger_.error("Failed to reconfigure the I2S RX clock");
    return false;
  }
  // one update period's worth of stereo (L,R) frames
  audio_rx_buffer.resize(calc_audio_buffer_size(audio_sample_rate(), RX_NUM_CHANNELS));
  if (i2s_channel_enable(audio_rx_handle) != ESP_OK) {
    logger_.error("Failed to enable I2S RX channel");
    return false;
  }

  // Enable the codec's ADC path alongside the running DAC and apply the
  // stored microphone gain
  es8311_codec_ctrl_state(AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);
  es8311_set_mic_gain(microphone_gain_from_volume(mic_volume_));

  using namespace std::placeholders;
  microphone_task_ = espp::Task::make_unique({
      .callback = std::bind(&Esp32P4Wifi6DevKit::microphone_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  if (!microphone_task_->start()) {
    // Leave the RX channel configured (audio_rx_std_configured_ stays true):
    // disabling it returns it to READY so a later initialize_microphone()
    // retry can enable it again.
    logger_.error("Could not start the microphone task");
    i2s_channel_disable(audio_rx_handle);
    microphone_task_.reset();
    return false;
  }

  microphone_initialized_ = true;
  return true;
}

bool Esp32P4Wifi6DevKit::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                                  bool &task_notified) {
  (void)cv; // unused: this task paces itself on the finite-timeout I2S read below
  size_t bytes_read = 0;
  // Use a finite read timeout (not portMAX_DELAY) so this task returns
  // periodically and can observe a stop request; an infinite read would block
  // Task::stop() from joining during teardown.
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, pdMS_TO_TICKS(100));
  if (err == ESP_OK && bytes_read > 0 && microphone_callback_) {
    // compact the L,R word pairs down to mono in place, keeping the left
    // slot (the ES8311's ADC data)
    auto *samples = reinterpret_cast<int16_t *>(audio_rx_buffer.data());
    size_t num_frames = bytes_read / (2 * sizeof(int16_t));
    for (size_t i = 0; i < num_frames; i++) {
      samples[i] = samples[2 * i];
    }
    microphone_callback_(audio_rx_buffer.data(), num_frames * sizeof(int16_t));
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

void Esp32P4Wifi6DevKit::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (microphone_initialized_) {
    es8311_set_mic_gain(microphone_gain_from_volume(mic_volume_));
  }
}

float Esp32P4Wifi6DevKit::microphone_volume() const { return mic_volume_; }

bool Esp32P4Wifi6DevKit::audio_task_callback(std::mutex &m, std::condition_variable &cv,
                                             bool &task_notified) {
  (void)cv; // unused: this task paces itself on the finite-timeout I2S write below
  size_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  size_t buffer_size = audio_tx_buffer.size();
  available = std::min(available, buffer_size);
  // only ever hand whole 16-bit samples to I2S; a partial sample would shift
  // the framing of everything after it
  available &= ~static_cast<size_t>(1);
  uint8_t *tx_buf = audio_tx_buffer.data();
  memset(tx_buf, 0, buffer_size);
  if (available > 0) {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
  }
  // Always write a full, frame-aligned buffer (queued samples zero-padded to
  // buffer_size) so the I2S DMA is fed at a constant cadence - matching the
  // esp-box / t-deck / m5stack-tab5 playback path. Writing only `available`
  // bytes makes the drain cadence variable and interleaves whole frames of
  // silence into bursty streams, which sounds choppy/glitchy.
  //
  // Use a finite write timeout (not portMAX_DELAY) so this task returns
  // periodically and can observe a stop request; an infinite write would block
  // Task::stop() from joining during teardown if the I2S sink ever stalls.
  size_t bytes_written = 0;
  esp_err_t err =
      i2s_channel_write(audio_tx_handle, tx_buf, buffer_size, &bytes_written, pdMS_TO_TICKS(100));
  if (err != ESP_OK || bytes_written != buffer_size) {
    // Rate-limit: a stalled I2S sink would otherwise emit this every ~100 ms.
    static uint32_t write_warn_count = 0;
    if ((write_warn_count++ % 100) == 0) {
      logger_.warn("i2s_channel_write: {} ({} of {} bytes written, occurrence {})",
                   esp_err_to_name(err), bytes_written, buffer_size, write_warn_count);
    }
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

uint32_t Esp32P4Wifi6DevKit::audio_sample_rate() const {
  return audio_std_cfg.clk_cfg.sample_rate_hz;
}

size_t Esp32P4Wifi6DevKit::audio_buffer_size() const { return audio_tx_buffer.size(); }

void Esp32P4Wifi6DevKit::audio_sample_rate(uint32_t sample_rate) {
  if (!audio_initialized_) {
    logger_.warn("audio_sample_rate() called before initialize_audio(); ignoring");
    return;
  }
  if (microphone_initialized_) {
    logger_.warn("Refusing to change the sample rate while the microphone is running: TX and RX "
                 "share the full-duplex I2S clock. Stop the microphone first, or pass the desired "
                 "rate to initialize_audio().");
    return;
  }
  // NOTE: this reconfigures the running I2S channel. It is best called when the
  // audio task is not actively streaming (e.g. right after initialize_audio, or
  // while no audio is playing). To avoid a runtime change entirely, pass the
  // desired sample rate to initialize_audio(). The ES8311 is an I2S slave and
  // follows the I2S clock, so it does not need a separate codec reconfigure.
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  esp_err_t err = i2s_channel_disable(audio_tx_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to disable I2S channel for reconfig: {}", esp_err_to_name(err));
    return;
  }
  // Reconfigure from a local copy and commit it to audio_std_cfg only on
  // success: audio_sample_rate() (the getter) reports the cached value, which
  // must not claim a rate the hardware rejected.
  i2s_std_clk_config_t clk_cfg = audio_std_cfg.clk_cfg;
  clk_cfg.sample_rate_hz = sample_rate;
  err = i2s_channel_reconfig_std_clock(audio_tx_handle, &clk_cfg);
  if (err != ESP_OK) {
    logger_.error("Failed to reconfigure I2S clock: {}", esp_err_to_name(err));
    // Leave the cached config and the queued stream untouched and re-enable
    // the channel so playback continues at the old (still-configured) rate.
    err = i2s_channel_enable(audio_tx_handle);
    if (err != ESP_OK) {
      logger_.error("Failed to re-enable I2S channel after failed reconfig: {}",
                    esp_err_to_name(err));
    }
    return;
  }
  audio_std_cfg.clk_cfg = clk_cfg;
  xStreamBufferReset(audio_tx_stream);
  err = i2s_channel_enable(audio_tx_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to re-enable I2S channel after reconfig: {}", esp_err_to_name(err));
  }
}

} // namespace espp

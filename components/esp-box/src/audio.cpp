#include <algorithm>
#include <cmath>
#include <cstring>

#include "esp-box.hpp"

using namespace espp;

// How long the audio task gives a producer that is mid-stream to complete a
// block before the block is written padded with silence. Longer than one block
// at every rate the board supports (a block is 1/60 s), so a producer whose
// next frame simply has not arrived yet is not padded; and shorter than the
// DMA ring minus one block, so the ring never drains while waiting (see
// AUDIO_TX_DMA_DESC_NUM).
static constexpr uint32_t AUDIO_TX_WHOLE_BLOCK_WAIT_MS = 24;

// Depth of the TX channel's DMA ring, in descriptors of AUDIO_TX_DMA_FRAME_NUM
// frames (the I2S driver's default frame count). The audio task keeps this
// ring full: the I2S driver fills descriptors progressively and the DMA sends
// them in ring order, so a descriptor the task has only partly filled is
// safe only while the DMA is a full ring behind it. The ring therefore has to
// outlast the longest pause between writes (AUDIO_TX_WHOLE_BLOCK_WAIT_MS) plus
// the block written after it: 12 x 240 frames is 60 ms at 48 kHz / 65 ms at
// 44.1 kHz; the driver's default of 6 (about 30 ms) leaves no margin.
static constexpr uint32_t AUDIO_TX_DMA_FRAME_NUM = 240;
static constexpr uint32_t AUDIO_TX_DMA_DESC_NUM = 12;

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

bool EspBox::initialize_codec() {
  logger_.info("initializing codec");

  std::error_code ec;
  codec_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = 0x18,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!codec_i2c_device_) {
    logger_.error("Could not initialize codec I2C device: {}", ec.message());
    return false;
  }

  set_es8311_write(espp::make_i2c_addressed_write(codec_i2c_device_));
  set_es8311_read(espp::make_i2c_addressed_read_register(codec_i2c_device_));

  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_ALL; // Enable both L and R outputs
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_48K_SAMPLES;

  ret_val |= es8311_codec_init(&cfg);
  ret_val |= es8311_config_fmt(ES_I2S_NORMAL);
  ret_val |= es8311_set_bits_per_sample(AUDIO_HAL_BIT_LENGTH_16BITS);
  ret_val |= es8311_codec_set_voice_volume(volume_);
  ret_val |= es8311_codec_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger_.error("Codec initialization failed");
    return false;
  } else {
    logger_.info("Codec initialized");
    return true;
  }
}

bool EspBox::initialize_i2s(uint32_t default_audio_rate) {
  logger_.info("initializing i2s driver");
  logger_.debug("Using newer I2S standard");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  // The ring must outlast the audio task's whole-block wait plus one block at
  // the highest rate the board supports (see AUDIO_TX_DMA_DESC_NUM).
  static_assert(AUDIO_TX_DMA_DESC_NUM * AUDIO_TX_DMA_FRAME_NUM * 1000 / 48000 >
                    AUDIO_TX_WHOLE_BLOCK_WAIT_MS + 1000 / UPDATE_FREQUENCY,
                "the I2S TX DMA ring must outlast the whole-block wait plus one block");
  chan_cfg.dma_desc_num = AUDIO_TX_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = AUDIO_TX_DMA_FRAME_NUM;
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, &audio_rx_handle));

  audio_std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(default_audio_rate),
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = i2s_mck_io,
                   .bclk = i2s_bck_io,
                   .ws = i2s_ws_io,
                   .dout = i2s_do_io,
                   .din = i2s_di_io,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  // audio_std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  // the block (and so the stream's trigger level) must be a whole number of
  // frames: the sample-rate formula can land between frames, and a block that
  // split one would desynchronize the framing of everything received after it
  buffer_size -= buffer_size % (NUM_CHANNELS * NUM_BYTES_PER_CHANNEL);
  audio_tx_buffer.resize(buffer_size);

  // The audio task never blocks on the stream (see audio_task_callback), so
  // the trigger level is irrelevant; the minimum keeps a producer's send from
  // ever holding a receiver.
  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 1);

  xStreamBufferReset(audio_tx_stream);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  return true;
}

bool EspBox::initialize_sound(uint32_t default_audio_rate,
                              const espp::Task::BaseConfig &task_config) {
  if (sound_initialized_) {
    logger_.warn("Sound already initialized");
    return true;
  }
  if (!initialize_i2s(default_audio_rate)) {
    logger_.error("Could not initialize I2S driver");
    return false;
  }
  if (!initialize_codec()) {
    logger_.error("Could not initialize codec");
    return false;
  }

  // Config power control IO
  gpio_set_direction(sound_power_pin, GPIO_MODE_OUTPUT);
  enable_sound(true);

  using namespace std::placeholders;
  audio_task_ = espp::Task::make_unique({
      .callback = std::bind(&EspBox::audio_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  sound_initialized_ = true;

  return audio_task_->start();
}

void EspBox::enable_sound(bool enable) { gpio_set_level(sound_power_pin, enable); }

bool EspBox::audio_task_callback(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  // Playback: write the next block to the I2S channel, ALWAYS a whole block.
  //
  // The I2S driver fills DMA descriptors progressively and the DMA transmits
  // them in ring order, so the only safe steady state is a full ring with the
  // write position a whole ring ahead of the DMA. Writing a whole block every
  // iteration - audio when there is some, silence when idle - keeps it there:
  // the write blocks on the ring and paces this loop. Letting the task sit on
  // the stream instead (waiting for data, writing whatever arrived) drains the
  // ring while idle; the next sound then starts in descriptors the DMA is
  // about to send, half old and half new - a pop at the start of every sound,
  // and more whenever a partial write is left behind.
  //
  // Producers push audio in bursts (a video frame at a time, with jitter), so
  // when a stream that is flowing momentarily holds less than a block, give
  // it a bounded chance to complete the block rather than padding the gap
  // with silence (audible as stutter). The wait is event-driven: play_audio()
  // notifies this task when the stream reaches a block. It is shorter than
  // the ring minus one block, so the ring never drains while waiting; a
  // stream that was already idle gets silence at once.
  if (audio_tx_task_handle_.load(std::memory_order_relaxed) == nullptr) {
    audio_tx_task_handle_.store(xTaskGetCurrentTaskHandle(), std::memory_order_release);
  }
  const size_t block = audio_tx_buffer.size();
  uint8_t *tx_buf = audio_tx_buffer.data();
  constexpr size_t frame_bytes = NUM_CHANNELS * NUM_BYTES_PER_CHANNEL;
  size_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  if (available % frame_bytes != 0) {
    // Producers queue whole frames only (play_audio), so this is a bug
    // somewhere. A stuck partial frame would misalign every frame queued after
    // it for good, and merely skipping it here would leave it stuck: say so and
    // start the stream over.
    logger_.warn_rate_limited("audio stream misaligned ({} bytes queued); resetting it", available);
    xStreamBufferReset(audio_tx_stream);
    available = 0;
  }
  if (available < block && (available > 0 || audio_tx_was_flowing_)) {
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(AUDIO_TX_WHOLE_BLOCK_WAIT_MS);
    while (available < block) {
      const TickType_t remaining = deadline - xTaskGetTickCount();
      if (static_cast<int32_t>(remaining) <= 0) {
        break;
      }
      // a notification left over from an earlier send wakes this at once; the
      // stream is re-read and the wait resumes, still bounded by the deadline
      ulTaskNotifyTake(pdTRUE, remaining);
      available = xStreamBufferBytesAvailable(audio_tx_stream);
    }
  }
  available = std::min(available, block);
  memset(tx_buf, 0, block);
  if (available > 0) {
    xStreamBufferReceive(audio_tx_stream, tx_buf, available, 0);
  }
  audio_tx_was_flowing_ = available > 0;
  i2s_channel_write(audio_tx_handle, tx_buf, block, NULL, portMAX_DELAY);
  return false; // don't stop the task
}

void EspBox::update_volume_output() {
  if (!sound_initialized_) {
    return;
  }
  if (mute_) {
    es8311_codec_set_voice_volume(0);
  } else {
    es8311_codec_set_voice_volume(volume_);
  }
}

void EspBox::mute(bool mute) {
  mute_ = mute;
  update_volume_output();
}

bool EspBox::is_muted() const { return mute_; }

void EspBox::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  update_volume_output();
}

float EspBox::volume() const { return volume_; }

uint32_t EspBox::audio_sample_rate() const { return audio_std_cfg.clk_cfg.sample_rate_hz; }

size_t EspBox::audio_buffer_size() const { return audio_tx_buffer.size(); }

void EspBox::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  // stop the channel
  i2s_channel_disable(audio_tx_handle);
  // update the sample rate
  auto err = es8311_codec_set_sample_rate(sample_rate);
  if (err != ESP_OK) {
    logger_.error("Could not set codec sample rate: {}", err);
  }
  audio_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle, &audio_std_cfg.clk_cfg);
  // clear the buffer
  xStreamBufferReset(audio_tx_stream);
  // restart the channel
  i2s_channel_enable(audio_tx_handle);
}

size_t EspBox::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

size_t EspBox::play_audio(const uint8_t *data, uint32_t num_bytes) {
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
  const size_t sent = xStreamBufferSend(audio_tx_stream, data, sendable, 0);
  // Wake the playback task if it is waiting for the stream to hold a whole
  // block (see audio_task_callback); a notification nobody waits for is
  // harmless (the task re-reads the stream after every wake).
  if (sent > 0 && xStreamBufferBytesAvailable(audio_tx_stream) >= audio_tx_buffer.size()) {
    if (TaskHandle_t task = audio_tx_task_handle_.load(std::memory_order_acquire)) {
      xTaskNotifyGive(task);
    }
  }
  return sent;
}

//////////////////////////
// Microphone Functions //
//////////////////////////

bool EspBox::initialize_microphone(const microphone_callback_t &callback,
                                   const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (microphone_initialized_) {
    logger_.warn("Microphone already initialized, not initializing again!");
    return false;
  }
  if (!sound_initialized_) {
    logger_.error("The sound subsystem must be initialized first: the ES7210 shares the I2S bus "
                  "with the ES8311 in full duplex");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  microphone_callback_ = callback;

  // The RX channel was allocated alongside the TX channel in
  // initialize_i2s(); in full-duplex mode it shares the TX BCLK/WS, so
  // initialize it with the same standard-mode (16-bit stereo) config
  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_rx_handle, &audio_std_cfg));
  audio_rx_buffer.resize(calc_audio_buffer_size(audio_sample_rate()));
  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  // Configure the ES7210 ADC (microphone 1 -> left slot, microphone 2 ->
  // right slot) now that the I2S clocks are running. Probe for the chip
  // first: the address depends on how its AD pins are strapped, so accept
  // either 0x40 or 0x41 and report clearly if neither responds.
  uint8_t es7210_address = 0;
  for (uint8_t address : {0x40, 0x41}) {
    if (internal_i2c_.probe_device(address)) {
      es7210_address = address;
      break;
    }
  }
  if (es7210_address == 0) {
    logger_.error("No ES7210 found on the internal I2C bus (probed 0x40 and 0x41); "
                  "cannot record from the microphones");
    i2s_channel_disable(audio_rx_handle);
    return false;
  }
  logger_.info("Found ES7210 at {:#04x}", es7210_address);
  std::error_code ec;
  es7210_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = es7210_address,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!es7210_i2c_device_) {
    logger_.error("Could not initialize ES7210 I2C device: {}", ec.message());
    i2s_channel_disable(audio_rx_handle);
    return false;
  }
  set_es7210_write(espp::make_i2c_addressed_write(es7210_i2c_device_));
  set_es7210_read(espp::make_i2c_addressed_read_register(es7210_i2c_device_));

  // only the two microphones are wired on the box (the other ES7210 inputs
  // are unused), and they map onto the two I2S slots
  es7210_mic_select(static_cast<es7210_input_mics_t>(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2));

  audio_hal_codec_config_t es7210_cfg{};
  es7210_cfg.codec_mode = AUDIO_HAL_CODEC_MODE_ENCODE;
  es7210_cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  es7210_cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  es7210_cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  // configure the ES7210 for the rate the I2S clocks are actually running at
  // (rather than assuming 48 kHz) so its coefficient table matches
  es7210_cfg.i2s_iface.samples = audio_hal_samples_from_rate(audio_sample_rate());
  if (es7210_adc_init(&es7210_cfg) != ESP_OK) {
    logger_.error("Could not initialize the ES7210 codec");
    i2s_channel_disable(audio_rx_handle);
    return false;
  }
  if (es7210_adc_config_i2s(AUDIO_HAL_CODEC_MODE_ENCODE, &es7210_cfg.i2s_iface) != ESP_OK) {
    logger_.error("ES7210 I2S config failed");
  }
  // Enable the ES7210 digital high-pass filter on the ADC channels to strip
  // the microphone DC offset. The espp es7210 driver leaves these registers
  // at their reset value (HPF off), so the DC offset is amplified by the
  // analog gain and rails the ADC to full scale - recordings come out as a
  // near-constant DC level that the AC-coupled speaker plays as a click then
  // silence. These are the driver's documented "quick setup" HPF values.
  // write_register clears the error code on success, so a later successful
  // write would mask an earlier failure; remember the first failure explicitly.
  std::error_code hpf_ec, first_hpf_ec;
  auto write_hpf = [&](uint8_t reg, uint8_t val) {
    es7210_i2c_device_->write_register(reg, std::vector<uint8_t>{val}, hpf_ec);
    if (hpf_ec && !first_hpf_ec) {
      first_hpf_ec = hpf_ec;
    }
  };
  write_hpf(0x22, 0x0a); // ADC1/2 HPF1
  write_hpf(0x23, 0x2a); // ADC1/2 HPF2
  write_hpf(0x20, 0x0a); // ADC3/4 HPF2
  write_hpf(0x21, 0x2a); // ADC3/4 HPF1
  if (first_hpf_ec) {
    logger_.warn("Could not enable the ES7210 high-pass filter: {}", first_hpf_ec.message());
  }
  // apply the stored microphone volume (the driver's init leaves the analog
  // gain at its 0 dB minimum, which records very quietly)
  es7210_adc_set_gain_all(microphone_gain_from_volume(mic_volume_));
  es7210_adc_ctrl_state(AUDIO_HAL_CODEC_MODE_ENCODE, AUDIO_HAL_CTRL_START);

  using namespace std::placeholders;
  microphone_task_ = espp::Task::make_unique({
      .callback = std::bind(&EspBox::microphone_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  if (!microphone_task_->start()) {
    i2s_channel_disable(audio_rx_handle);
    microphone_task_.reset();
    return false;
  }

  microphone_initialized_ = true;
  return true;
}

bool EspBox::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                      bool &task_notified) {
  size_t bytes_read = 0;
  // Use a finite read timeout (not portMAX_DELAY) so this task returns
  // periodically and can observe a stop request; an infinite read would block
  // Task::stop() from joining during teardown.
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, pdMS_TO_TICKS(100));
  if (err == ESP_OK && bytes_read > 0 && microphone_callback_) {
    microphone_callback_(audio_rx_buffer.data(), bytes_read);
  }
  // honor a stop request per the Task contract: check/clear notified under m
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true; // stop the task
  }
  return false; // keep running
}

void EspBox::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (microphone_initialized_) {
    es7210_adc_set_gain_all(microphone_gain_from_volume(mic_volume_));
  }
}

float EspBox::microphone_volume() const { return mic_volume_; }

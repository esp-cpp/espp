#include "m5stack-cardputer.hpp"

#include <algorithm>
#include <cstring>

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
// Audio Functions    //
////////////////////////

bool M5StackCardputer::initialize_i2s(uint32_t default_audio_rate) {
  // original Cardputer only: transmit-only standard I2S into the NS4168
  // amplifier (the ADV uses the full-duplex path in ensure_adv_i2s())
  logger_.info("initializing i2s driver");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(i2s_port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // Auto clear the legacy data in the DMA buffer
  // The ring must outlast the audio task's whole-block wait plus one block at
  // the highest rate the board supports (see AUDIO_TX_DMA_DESC_NUM).
  static_assert(AUDIO_TX_DMA_DESC_NUM * AUDIO_TX_DMA_FRAME_NUM * 1000 / 48000 >
                    AUDIO_TX_WHOLE_BLOCK_WAIT_MS + 1000 / UPDATE_FREQUENCY,
                "the I2S TX DMA ring must outlast the whole-block wait plus one block");
  chan_cfg.dma_desc_num = AUDIO_TX_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = AUDIO_TX_DMA_FRAME_NUM;
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
  // The ring must outlast the audio task's whole-block wait plus one block at
  // the highest rate the board supports (see AUDIO_TX_DMA_DESC_NUM).
  static_assert(AUDIO_TX_DMA_DESC_NUM * AUDIO_TX_DMA_FRAME_NUM * 1000 / 48000 >
                    AUDIO_TX_WHOLE_BLOCK_WAIT_MS + 1000 / UPDATE_FREQUENCY,
                "the I2S TX DMA ring must outlast the whole-block wait plus one block");
  chan_cfg.dma_desc_num = AUDIO_TX_DMA_DESC_NUM;
  chan_cfg.dma_frame_num = AUDIO_TX_DMA_FRAME_NUM;
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
  size_t buffer_size = calc_audio_buffer_size(audio_sample_rate());
  // the block (and so the stream's trigger level) must be a whole number of
  // frames: the sample-rate formula can land between frames, and a block that
  // split one would desynchronize the framing of everything received after it
  buffer_size -= buffer_size % (NUM_CHANNELS * NUM_BYTES_PER_CHANNEL);
  audio_tx_buffer.resize(buffer_size);
  // The audio task never blocks on the stream (see audio_task_callback), so
  // the trigger level is irrelevant; the minimum keeps a producer's send from
  // ever holding a receiver.
  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 1);
  if (audio_tx_stream == nullptr) {
    logger_.error("Could not allocate the audio stream buffer");
    audio_tx_buffer.clear();
    if (is_adv) {
      if (!microphone_initialized_) {
        // the channels are shared with the microphone; only tear them down
        // if it is not using them
        i2s_del_channel(audio_tx_handle);
        i2s_del_channel(audio_rx_handle);
        audio_tx_handle = nullptr;
        audio_rx_handle = nullptr;
      }
    } else {
      i2s_channel_disable(audio_tx_handle);
      i2s_del_channel(audio_tx_handle);
      audio_tx_handle = nullptr;
    }
    return false;
  }
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
  // The amplifier has no volume control, so scale the samples in software
  // according to the current volume / mute state (silence stays silence).
  if (available > 0) {
    float scale = mute_ ? 0.0f : (volume_ / 100.0f);
    auto *samples = reinterpret_cast<int16_t *>(tx_buf);
    size_t num_samples = block / sizeof(int16_t);
    for (size_t i = 0; i < num_samples; i++) {
      samples[i] = static_cast<int16_t>(samples[i] * scale);
    }
  }
  i2s_channel_write(audio_tx_handle, tx_buf, block, NULL, portMAX_DELAY);
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
  // guard against being called before initialize_sound() (audio_tx_stream is
  // not valid until then) and against empty input
  if (!sound_initialized_ || !data || num_bytes == 0) {
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

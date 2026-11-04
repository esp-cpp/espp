#include "audio.hpp"

#include <algorithm>
#include <cmath>

#include <esp_err.h>

using namespace espp;

// The ring must outlast the playback task's whole-block wait plus one block at
// the highest rate the I2S standard-mode boards use (the policy and its
// rationale: codec/i2s_tx_feed.hpp).
static_assert(espp::i2s_tx_feed::ring_outlasts_wait(espp::i2s_tx_feed::DMA_DESC_NUM,
                                                    espp::i2s_tx_feed::DMA_FRAME_NUM, 48000,
                                                    Audio::UPDATE_FREQUENCY),
              "the I2S TX DMA ring must outlast the whole-block wait plus one block");

// 16-bit PCM
static constexpr size_t BYTES_PER_SAMPLE = 2;

// One block: 1 / UPDATE_FREQUENCY seconds of audio, whole frames only (the
// stream's framing depends on it: a block that split a frame would misalign
// everything after it).
static size_t block_size(uint32_t sample_rate, uint8_t channels) {
  const size_t frame_bytes = channels * BYTES_PER_SAMPLE;
  return (sample_rate / Audio::UPDATE_FREQUENCY) * frame_bytes;
}

static i2s_std_config_t make_std_config(const Audio::Config &config, uint8_t channels) {
  i2s_std_config_t cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(config.sample_rate),
      .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
          I2S_DATA_BIT_WIDTH_16BIT, channels == 1 ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO),
      .gpio_cfg = {.mclk = config.pins.mclk,
                   .bclk = config.pins.bclk,
                   .ws = config.pins.ws,
                   .dout = config.pins.dout,
                   .din = config.pins.din,
                   .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
  };
  cfg.clk_cfg.mclk_multiple = config.mclk_multiple;
  return cfg;
}

Audio::Audio(const Config &config)
    : BaseComponent("Audio", config.log_level)
    , config_(config)
    , sample_rate_(config.sample_rate)
    , volume_(std::clamp(config.volume, 0.0f, 100.0f))
    , muted_(config.muted)
    , microphone_volume_(std::clamp(config.microphone_volume, 0.0f, 100.0f)) {}

Audio::~Audio() { deinitialize(); }

bool Audio::same_codec() const {
  return config_.output_codec && config_.output_codec == config_.input_codec;
}

bool Audio::software_volume() const {
  return !config_.output_codec || !config_.output_codec->has_volume();
}

bool Audio::initialize() {
  std::error_code ec;
  if (!initialize(ec)) {
    logger_.error("Could not initialize audio: {}", ec.message());
    return false;
  }
  return true;
}

bool Audio::initialize(std::error_code &ec) {
  std::unique_lock<std::mutex> lock(mutex_);
  ec.clear();
  if (initialized_) {
    return true;
  }
  if (tearing_down_) {
    ec = std::make_error_code(std::errc::device_or_resource_busy);
    return false;
  }
  const bool playback = config_.playback_enabled;
  const bool recording = config_.recording_enabled;
  if (!playback && !recording) {
    logger_.error("Neither playback nor recording is enabled");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  auto valid_channels = [](uint8_t channels) { return channels == 1 || channels == 2; };
  if ((playback && !valid_channels(config_.playback_channels)) ||
      (recording && !valid_channels(config_.recording_channels))) {
    logger_.error("Channels must be 1 (mono) or 2 (stereo)");
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }
  const uint32_t rate = config_.sample_rate;
  logger_.info("Initializing I2S{} audio ({}{}{}) at {} Hz", config_.port,
               playback ? "playback" : "", playback && recording ? " + " : "",
               recording ? "recording" : "", rate);
  sample_rate_ = rate;

  // every failure below unwinds what was set up before it
  auto fail = [&](std::errc err) {
    ec = std::make_error_code(err);
    teardown(lock);
    return false;
  };

  // One call creates both channels so they run full duplex (RX shares the TX
  // BCLK/WS), each with a DMA ring of this size; a direction that is not
  // enabled is not created, which saves its ring's internal DMA memory. Alone,
  // either channel drives the clocks itself (simplex master).
  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(config_.port, I2S_ROLE_MASTER);
  chan_cfg.auto_clear = true; // clear the legacy data in the DMA buffer
  chan_cfg.dma_desc_num = espp::i2s_tx_feed::DMA_DESC_NUM;
  chan_cfg.dma_frame_num = espp::i2s_tx_feed::DMA_FRAME_NUM;
  esp_err_t err = i2s_new_channel(&chan_cfg, playback ? &tx_handle_ : nullptr,
                                  recording ? &rx_handle_ : nullptr);
  if (err != ESP_OK) {
    logger_.error("Could not create the I2S channel(s): {}", esp_err_to_name(err));
    // a handle that is set was registered before the failure (e.g. TX when RX
    // could not be): teardown() deletes it
    return fail(std::errc::io_error);
  }

  if (playback) {
    const i2s_std_config_t tx_cfg = make_std_config(config_, config_.playback_channels);
    clk_cfg_ = tx_cfg.clk_cfg;
    err = i2s_channel_init_std_mode(tx_handle_, &tx_cfg);
    if (err != ESP_OK) {
      logger_.error("Could not configure the I2S TX channel: {}", esp_err_to_name(err));
      return fail(std::errc::io_error);
    }
  }
  if (recording) {
    // in full duplex the RX channel runs on the TX clocks, so it shares the
    // standard-mode clock / slot geometry (only the slot mode may differ)
    const i2s_std_config_t rx_cfg = make_std_config(config_, config_.recording_channels);
    if (!playback) {
      clk_cfg_ = rx_cfg.clk_cfg;
    }
    err = i2s_channel_init_std_mode(rx_handle_, &rx_cfg);
    if (err != ESP_OK) {
      logger_.error("Could not configure the I2S RX channel: {}", esp_err_to_name(err));
      return fail(std::errc::io_error);
    }
  }

  if (playback) {
    block_bytes_ = block_size(rate, config_.playback_channels);
    tx_buffer_.resize(block_bytes_);
    // set before the stream exists: play() uses it once tx_stream_ is set
    // (both under mutex_, which initialize() holds anyway)
    tx_feed_.frame_bytes = config_.playback_channels * BYTES_PER_SAMPLE;
    // The playback task never blocks on the stream (see
    // playback_task_callback), so the trigger level is irrelevant; the
    // minimum keeps a producer's send from ever holding a receiver.
    tx_stream_ = xStreamBufferCreate(block_bytes_ * 4, 1);
    if (!tx_stream_) {
      logger_.error("Could not create the playback stream buffer");
      return fail(std::errc::not_enough_memory);
    }
    tx_feed_.stream = tx_stream_;
  }
  if (recording) {
    rx_buffer_.resize(block_size(rate, config_.recording_channels));
  }

  // the codecs need the clocks (MCLK) running before they are configured
  if (playback) {
    err = i2s_channel_enable(tx_handle_);
    if (err != ESP_OK) {
      logger_.error("Could not enable the I2S TX channel: {}", esp_err_to_name(err));
      return fail(std::errc::io_error);
    }
    tx_enabled_ = true;
  }
  if (recording) {
    err = i2s_channel_enable(rx_handle_);
    if (err != ESP_OK) {
      logger_.error("Could not enable the I2S RX channel: {}", esp_err_to_name(err));
      return fail(std::errc::io_error);
    }
    rx_enabled_ = true;
  }

  // a codec counts as started (and is stopped on failure) once its start
  // has begun: an init may leave the chip partly powered up when it fails
  auto start_codec = [&](AudioCodec &codec, AudioCodec::Direction direction) {
    logger_.info("Starting the {} codec", codec.name());
    if (!codec.start(direction, rate)) {
      logger_.error("Could not start the {} codec", codec.name());
      return false;
    }
    return true;
  };
  if (same_codec() && playback && recording) {
    output_started_ = input_started_ = true;
    if (!start_codec(*config_.output_codec, AudioCodec::Direction::BOTH)) {
      return fail(std::errc::io_error);
    }
  } else {
    if (playback && config_.output_codec) {
      output_started_ = true;
      if (!start_codec(*config_.output_codec, AudioCodec::Direction::PLAYBACK)) {
        return fail(std::errc::io_error);
      }
    }
    if (recording && config_.input_codec) {
      input_started_ = true;
      if (!start_codec(*config_.input_codec, AudioCodec::Direction::RECORDING)) {
        return fail(std::errc::io_error);
      }
    }
  }
  if (output_started_ && config_.output_codec->has_volume()) {
    config_.output_codec->set_volume(volume_);
    config_.output_codec->set_mute(muted_);
  }
  if (input_started_ && config_.input_codec->has_gain()) {
    config_.input_codec->set_gain(microphone_volume_);
  }

  // Two independent tasks: one keeps the TX ring full from the stream, the
  // other blocks on the RX channel. Keeping the record read off the playback
  // task drains the RX ring at exactly the rate samples arrive (rather than
  // only between blocking TX writes); in full duplex a ring that backs up
  // perturbs the TX too.
  using namespace std::placeholders;
  if (playback) {
    playback_task_ = espp::Task::make_unique(
        {.callback = std::bind(&Audio::playback_task_callback, this, _1, _2, _3),
         .task_config = config_.playback_task_config});
    if (!playback_task_->start()) {
      logger_.error("Could not start the playback task");
      return fail(std::errc::resource_unavailable_try_again);
    }
  }
  if (recording) {
    recording_task_ = espp::Task::make_unique(
        {.callback = std::bind(&Audio::recording_task_callback, this, _1, _2, _3),
         .task_config = config_.recording_task_config});
    if (!recording_task_->start()) {
      logger_.error("Could not start the recording task");
      return fail(std::errc::resource_unavailable_try_again);
    }
  }

  initialized_ = true;
  return true;
}

void Audio::deinitialize() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (!initialized_ || tearing_down_) {
    return;
  }
  logger_.info("Deinitializing audio");
  teardown(lock);
}

void Audio::teardown(std::unique_lock<std::mutex> &lock) {
  // called with mutex_ held
  initialized_ = false;
  recording_ = false;
  // The tasks first: they use the channels, the stream and the buffers. Join
  // them with mutex_ released, so a callback that calls a method (which takes
  // mutex_) cannot deadlock against the join; tearing_down_ keeps
  // initialize() / deinitialize() out meanwhile.
  auto playback_task = std::move(playback_task_);
  auto recording_task = std::move(recording_task_);
  tearing_down_ = true;
  lock.unlock();
  if (playback_task) {
    playback_task->stop();
    playback_task.reset();
  }
  if (recording_task) {
    recording_task->stop();
    recording_task.reset();
  }
  lock.lock();
  tearing_down_ = false;
  // the playback task registers its handle for play()'s notifications on
  // every iteration: forget it only once the task can no longer run, or it
  // could register it again and a later initialize() would keep the stale
  // handle
  tx_feed_.detach_task();
  recording_task_handle_ = nullptr;
  recording_callback_ = nullptr;

  if (input_started_ && !same_codec()) {
    config_.input_codec->stop();
  }
  if (output_started_) {
    config_.output_codec->stop();
  }
  input_started_ = output_started_ = false;
  if (tx_enabled_) {
    i2s_channel_disable(tx_handle_);
    tx_enabled_ = false;
  }
  if (rx_enabled_) {
    i2s_channel_disable(rx_handle_);
    rx_enabled_ = false;
  }
  if (tx_handle_) {
    i2s_del_channel(tx_handle_);
    tx_handle_ = nullptr;
  }
  if (rx_handle_) {
    i2s_del_channel(rx_handle_);
    rx_handle_ = nullptr;
  }
  tx_feed_.stream = nullptr;
  if (tx_stream_) {
    vStreamBufferDelete(tx_stream_);
    tx_stream_ = nullptr;
  }
  std::vector<uint8_t>().swap(tx_buffer_);
  std::vector<uint8_t>().swap(rx_buffer_);
  block_bytes_ = 0;
}

size_t Audio::play(std::span<const uint8_t> data) { return play(data.data(), data.size()); }

size_t Audio::play(const uint8_t *data, size_t num_bytes) {
  if (!data || num_bytes == 0) {
    return 0;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!tx_stream_) {
    return 0;
  }
  // Queue only whole frames that actually fit: xStreamBufferSend can accept
  // fewer bytes than requested when the buffer is nearly full, and a partial
  // frame would corrupt the framing of everything queued after it. With a 0
  // timeout the send never blocks.
  size_t sendable = std::min<size_t>(num_bytes, xStreamBufferSpacesAvailable(tx_stream_));
  sendable -= sendable % tx_feed_.frame_bytes;
  if (sendable == 0) {
    return 0;
  }
  const size_t sent = xStreamBufferSend(tx_stream_, data, sendable, 0);
  // wake the playback task if it is waiting for the stream to hold a whole
  // block (send -> notify -> return; see i2s_tx_feed.hpp)
  if (sent > 0) {
    tx_feed_.notify_if_block_ready(block_bytes_);
  }
  return sent;
}

void Audio::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (tx_stream_) {
    xStreamBufferReset(tx_stream_);
  }
}

void Audio::volume(float volume) {
  std::lock_guard<std::mutex> lock(mutex_);
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (output_started_ && config_.output_codec->has_volume()) {
    config_.output_codec->set_volume(volume_);
  }
}

void Audio::mute(bool mute) {
  std::lock_guard<std::mutex> lock(mutex_);
  muted_ = mute;
  if (output_started_ && config_.output_codec->has_volume()) {
    config_.output_codec->set_mute(mute);
  }
}

void Audio::microphone_volume(float volume) {
  std::lock_guard<std::mutex> lock(mutex_);
  microphone_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (input_started_ && config_.input_codec->has_gain()) {
    config_.input_codec->set_gain(microphone_volume_);
  }
}

bool Audio::sample_rate(uint32_t sample_rate) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_) {
    logger_.warn("Not initialized: set Config::sample_rate instead");
    return false;
  }
  logger_.info("Setting the sample rate to {} Hz", sample_rate);
  // The channel that drives the clocks: TX (whose BCLK/WS RX shares in full
  // duplex), or RX when only recording is set up. Both channels stop while the
  // clock changes, rather than RX sampling through the switch (the recording
  // task's read just times out meanwhile).
  i2s_chan_handle_t clock_handle = tx_handle_ ? tx_handle_ : rx_handle_;
  if (tx_enabled_) {
    i2s_channel_disable(tx_handle_);
  }
  if (rx_enabled_) {
    i2s_channel_disable(rx_handle_);
  }
  i2s_std_clk_config_t clk_cfg = clk_cfg_;
  clk_cfg.sample_rate_hz = sample_rate;
  const esp_err_t err = i2s_channel_reconfig_std_clock(clock_handle, &clk_cfg);
  if (err == ESP_OK) {
    clk_cfg_ = clk_cfg;
    sample_rate_ = sample_rate;
  } else {
    logger_.error("Could not set the I2S clock to {} Hz: {}", sample_rate, esp_err_to_name(err));
  }
  // the queued audio was for the old rate
  if (tx_stream_) {
    xStreamBufferReset(tx_stream_);
  }
  if (tx_enabled_) {
    i2s_channel_enable(tx_handle_);
  }
  if (rx_enabled_) {
    i2s_channel_enable(rx_handle_);
  }
  if (err != ESP_OK) {
    return false;
  }
  bool ok = true;
  if (output_started_) {
    ok &= config_.output_codec->set_sample_rate(sample_rate);
  }
  if (input_started_ && !same_codec()) {
    ok &= config_.input_codec->set_sample_rate(sample_rate);
  }
  return ok;
}

bool Audio::start_recording(const recording_callback_t &callback) {
  if (!callback) {
    logger_.error("start_recording() needs a callback");
    return false;
  }
  // a call of the previous callback may still be finishing (after a
  // stop_recording() from that callback): replace it only once it returned.
  // Wait without mutex_, which that callback may be waiting for.
  while (in_callback_) {
    vTaskDelay(1);
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!initialized_ || !config_.recording_enabled) {
    logger_.error("Recording is not initialized");
    return false;
  }
  if (recording_) {
    logger_.warn("A recording is already in progress; stop it first");
    return false;
  }
  recording_callback_ = callback;
  recording_ = true;
  return true;
}

void Audio::stop_recording() {
  recording_ = false;
  // wait for a callback in flight to return, so that none runs after this
  // returns - unless this is that callback. (recording_ and in_callback_ are
  // sequentially consistent: either the recording task sees recording_ false
  // and skips the call, or this sees in_callback_ set and waits.)
  if (xTaskGetCurrentTaskHandle() == recording_task_handle_) {
    return;
  }
  while (in_callback_) {
    vTaskDelay(1);
  }
}

void Audio::apply_software_volume(uint8_t *data, size_t length) const {
  const float gain = muted_ ? 0.0f : volume_ / 100.0f;
  if (gain >= 1.0f) {
    return;
  }
  auto *samples = reinterpret_cast<int16_t *>(data);
  const size_t count = length / BYTES_PER_SAMPLE;
  for (size_t i = 0; i < count; i++) {
    samples[i] = static_cast<int16_t>(samples[i] * gain);
  }
}

bool Audio::playback_task_callback(std::mutex &m, std::condition_variable &cv,
                                   bool &task_notified) {
  // Write the next block to the I2S channel, ALWAYS a whole block, so the TX
  // DMA ring stays full; a stream that is flowing but momentarily holds less
  // than a block is given a bounded, notification-driven chance to complete
  // it. The policy and its rationale live in codec/i2s_tx_feed.hpp.
  tx_feed_.attach_current_task();
  uint8_t *buffer = tx_buffer_.data();
  bool misaligned = false;
  const size_t audio_bytes = tx_feed_.fill_block(buffer, block_bytes_, misaligned);
  if (misaligned) {
    // play() queues whole frames only, so this is a bug somewhere; the feeder
    // reset the stream rather than leave a partial frame misaligning
    // everything queued after it.
    logger_.warn_rate_limited(
        "playback stream misaligned ({} bytes queued, {} bytes into a {}-byte frame); reset it",
        tx_feed_.misaligned_bytes, tx_feed_.misaligned_bytes % tx_feed_.frame_bytes,
        tx_feed_.frame_bytes);
  }
  if (audio_bytes > 0 && software_volume()) {
    apply_software_volume(buffer, audio_bytes);
  }
  if (config_.on_playback_block) {
    config_.on_playback_block(audio_bytes > 0);
  }
  // blocks on the DMA ring, which paces this loop
  i2s_channel_write(tx_handle_, buffer, block_bytes_, nullptr, portMAX_DELAY);
  return false;
}

bool Audio::recording_task_callback(std::mutex &m, std::condition_variable &cv,
                                    bool &task_notified) {
  recording_task_handle_ = xTaskGetCurrentTaskHandle();
  // Always read, so the RX DMA ring is drained at the rate samples arrive (a
  // ring that backs up perturbs the TX in full duplex); the samples only go to
  // the callback while a recording is in progress. The finite timeout lets the
  // task observe a stop request.
  size_t bytes_read = 0;
  const esp_err_t err = i2s_channel_read(rx_handle_, rx_buffer_.data(), rx_buffer_.size(),
                                         &bytes_read, pdMS_TO_TICKS(100));
  if (err == ESP_OK && bytes_read > 0 && recording_) {
    // in_callback_ before re-checking recording_ (see stop_recording())
    in_callback_ = true;
    if (recording_) {
      recording_callback_(rx_buffer_.data(), bytes_read);
    }
    in_callback_ = false;
  }
  std::unique_lock<std::mutex> lock(m);
  if (task_notified) {
    task_notified = false;
    return true;
  }
  return false;
}

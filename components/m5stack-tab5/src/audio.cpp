#include <algorithm>

#include "m5stack-tab5.hpp"

////////////////////////
// Audio Functions   //
////////////////////////

// The I2S channels, the playback and microphone tasks, the stream buffer and
// the codec setup live in the espp::Audio component (with its ES8388 and
// ES7210 adapters); this file keeps only what is specific to the Tab5: the
// pins and codecs, the speaker amplifier, and pausing the charger during
// playback.

namespace espp {

M5StackTab5::AudioConfig M5StackTab5::default_audio_config() {
  return {.sample_rate = 48000,
          .playback_enabled = true,
          .recording_enabled = true,
          .playback_task_config = {.name = "tab5_audio out",
                                   .stack_size_bytes = CONFIG_M5STACK_TAB5_AUDIO_TASK_STACK_SIZE,
                                   .priority = 20,
                                   .core_id = 1},
          .recording_task_config = {.name = "tab5_audio in",
                                    .stack_size_bytes = CONFIG_M5STACK_TAB5_AUDIO_TASK_STACK_SIZE,
                                    .priority = 20,
                                    .core_id = 1}};
}

bool M5StackTab5::initialize_audio() { return initialize_audio(default_audio_config()); }

bool M5StackTab5::initialize_audio(uint32_t sample_rate,
                                   const espp::Task::BaseConfig &task_config) {
  auto config = default_audio_config();
  config.sample_rate = sample_rate;
  // the old task config was shared by both tasks, named "<name> out" / "<name> in"
  const std::string name = task_config.name.empty() ? std::string("audio") : task_config.name;
  config.playback_task_config = task_config;
  config.playback_task_config.name = name + " out";
  config.recording_task_config = task_config;
  config.recording_task_config.name = name + " in";
  return initialize_audio(config);
}

bool M5StackTab5::initialize_audio(const AudioConfig &config) {
  const bool playback = config.playback_enabled;
  const bool recording = config.recording_enabled;
  logger_.info("Initializing audio system ({}{}{}) at {} Hz", playback ? "ES8388 playback" : "",
               playback && recording ? " + " : "", recording ? "ES7210 recording" : "",
               config.sample_rate);
  if (is_audio_initialized()) {
    logger_.warn("Audio already initialized");
    return true;
  }
  if (!playback && !recording) {
    logger_.error("initialize_audio(): neither playback nor recording is enabled");
    return false;
  }

  // the codecs are on the internal I2C bus
  auto add_codec_device = [this](uint8_t address, const char *name) {
    std::error_code ec;
    auto device = internal_i2c_.add_device<uint8_t>(
        {
            .device_address = address,
            .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
            .scl_speed_hz = internal_i2c_.config().clk_speed,
            .log_level = espp::Logger::Verbosity::WARN,
        },
        ec);
    if (!device) {
      logger_.error("Could not initialize {} I2C device: {}", name, ec.message());
    }
    return device;
  };
  if (playback) {
    es8388_i2c_device_ = add_codec_device(0x10, "ES8388");
    if (!es8388_i2c_device_) {
      return false;
    }
    es8388_ = std::make_shared<espp::Es8388Codec>(espp::Es8388Codec::Config{
        .write = espp::make_i2c_addressed_write(es8388_i2c_device_),
        .read = espp::make_i2c_addressed_read_register(es8388_i2c_device_),
    });
  }
  if (recording) {
    es7210_i2c_device_ = add_codec_device(0x40, "ES7210");
    if (!es7210_i2c_device_) {
      deinitialize_audio();
      return false;
    }
    // standard I2S output, 16-bit stereo: microphone 1 on the left slot,
    // microphone 2 on the right
    es7210_ = std::make_shared<espp::Es7210Codec>(espp::Es7210Codec::Config{
        .write = espp::make_i2c_addressed_write(es7210_i2c_device_),
        .read = espp::make_i2c_addressed_read_register(es7210_i2c_device_),
    });
  }

  // Both directions on I2S0, full duplex: the ES8388 (DSDIN) and the ES7210
  // (ASDOUT) share MCLK / SCLK / LRCK. The playback data is interleaved
  // 16-bit stereo, so the slots MUST be stereo - a mono slot plays
  // interleaved stereo at the wrong rate and garbles L/R.
  auto audio = std::make_shared<espp::Audio>(espp::Audio::Config{
      .app = config,
      .hardware = {.port = I2S_NUM_0,
                   .pins = {.mclk = audio_mclk_io,
                            .bclk = audio_sclk_io,
                            .ws = audio_lrck_io,
                            .dout = audio_dsdin_io,
                            .din = audio_asdout_io},
                   .playback_channels = 2,
                   .recording_channels = 2,
                   .output_codec = es8388_,
                   .input_codec = es7210_},
      .volume = volume_,
      .muted = mute_,
      .microphone_volume = mic_volume_,
      // the charger pops the speaker while it is enabled: hold it off during
      // playback (see set_charging_pause_on_playback_enabled)
      .on_playback_block = [this](bool active) { update_charging_for_audio(active); },
  });
  if (!audio->initialize()) {
    audio.reset();
    deinitialize_audio();
    return false;
  }
  // published once it is initialized; the forwarders below each take one
  // snapshot of it
  audio_.store(audio);
  // A setter that ran meanwhile only updated its cached value: forward the
  // latest ones, again while they change, so one racing this is not undone by
  // a stale value.
  float volume;
  do {
    volume = volume_;
    audio->volume(volume);
  } while (volume != volume_);
  bool muted;
  do {
    muted = mute_;
    audio->mute(muted);
  } while (muted != mute_);
  float mic_volume;
  do {
    mic_volume = mic_volume_;
    audio->microphone_volume(mic_volume);
  } while (mic_volume != mic_volume_);

  if (playback) {
    enable_audio(true);
  }
  return true;
}

void M5StackTab5::deinitialize_audio() {
  // unpublished first: a forwarder called meanwhile (e.g. from a recording
  // callback still finishing) then sees no audio rather than one being torn
  // down
  auto audio = audio_.exchange(nullptr);
  if (audio && audio->is_initialized() && audio->is_playback_enabled()) {
    enable_audio(false);
  }
  // release the I2S channels, tasks and codecs now, even if the application
  // still holds the object from audio()
  if (audio) {
    audio->deinitialize();
  }
  audio.reset();
  es8388_.reset();
  es7210_.reset();
  es8388_i2c_device_.reset();
  es7210_i2c_device_.reset();
  // the playback task may have paused charging
  resume_charging_after_audio();
}

void M5StackTab5::enable_audio(bool enable) {
  set_speaker_enabled(enable);
  logger_.debug("Audio {}", enable ? "enabled" : "disabled");
}

void M5StackTab5::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
  logger_.debug("Volume set to {:.1f} %", volume_.load());
}

float M5StackTab5::volume() const { return volume_; }

void M5StackTab5::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool M5StackTab5::is_muted() const { return mute_; }

size_t M5StackTab5::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t M5StackTab5::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

void M5StackTab5::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

void M5StackTab5::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->microphone_volume(mic_volume_);
  }
}

float M5StackTab5::microphone_volume() const { return mic_volume_; }

bool M5StackTab5::start_audio_recording(
    std::function<void(const uint8_t *data, size_t length)> callback) {
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.error("Audio system not initialized");
    return false;
  }
  if (!audio->is_recording_enabled()) {
    logger_.error("Audio recording was not enabled in initialize_audio()");
    return false;
  }
  if (!audio->start_recording(callback)) {
    return false;
  }
  logger_.info("Audio recording started");
  return true;
}

void M5StackTab5::stop_audio_recording() {
  if (auto audio = audio_.load()) {
    audio->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

uint32_t M5StackTab5::audio_sample_rate() const {
  auto audio = audio_.load();
  return audio ? audio->sample_rate() : 0;
}

size_t M5StackTab5::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

bool M5StackTab5::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio->sample_rate(sample_rate);
}

} // namespace espp

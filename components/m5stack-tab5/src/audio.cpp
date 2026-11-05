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

bool M5StackTab5::initialize_audio() { return initialize_audio(AudioConfig{}); }

bool M5StackTab5::initialize_audio(uint32_t sample_rate,
                                   const espp::Task::BaseConfig &task_config) {
  return initialize_audio(AudioConfig{.sample_rate = sample_rate, .task_config = task_config});
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

  // the playback and microphone tasks share the caller's task config, named
  // "<name> out" / "<name> in" so the two are easy to tell apart in logs and
  // while debugging (short enough for FreeRTOS's 15-character task names)
  const std::string base_name =
      config.task_config.name.empty() ? std::string("audio") : config.task_config.name;
  auto playback_task_config = config.task_config;
  playback_task_config.name = base_name + " out";
  auto recording_task_config = config.task_config;
  recording_task_config.name = base_name + " in";
  // Both directions on I2S0, full duplex: the ES8388 (DSDIN) and the ES7210
  // (ASDOUT) share MCLK / SCLK / LRCK. The playback data is interleaved
  // 16-bit stereo, so the slots MUST be stereo - a mono slot plays
  // interleaved stereo at the wrong rate and garbles L/R.
  audio_ = std::make_shared<espp::Audio>(espp::Audio::Config{
      .port = I2S_NUM_0,
      .pins = {.mclk = audio_mclk_io,
               .bclk = audio_sclk_io,
               .ws = audio_lrck_io,
               .dout = audio_dsdin_io,
               .din = audio_asdout_io},
      .sample_rate = config.sample_rate,
      .playback_enabled = playback,
      .recording_enabled = recording,
      .playback_channels = 2,
      .recording_channels = 2,
      .output_codec = es8388_,
      .input_codec = es7210_,
      .volume = volume_,
      .muted = mute_,
      .microphone_volume = mic_volume_,
      // the charger pops the speaker while it is enabled: hold it off during
      // playback (see set_charging_pause_on_playback_enabled)
      .on_playback_block = [this](bool active) { update_charging_for_audio(active); },
      .playback_task_config = playback_task_config,
      .recording_task_config = recording_task_config,
  });
  if (!audio_->initialize()) {
    deinitialize_audio();
    return false;
  }

  if (playback) {
    enable_audio(true);
  }
  return true;
}

void M5StackTab5::deinitialize_audio() {
  if (is_audio_initialized() && audio_->is_playback_enabled()) {
    enable_audio(false);
  }
  // release the I2S channels, tasks and codecs now, even if the application
  // still holds the object from audio()
  if (audio_) {
    audio_->deinitialize();
  }
  audio_.reset();
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
  if (audio_) {
    audio_->volume(volume_);
  }
  logger_.debug("Volume set to {:.1f} %", volume_.load());
}

float M5StackTab5::volume() const { return volume_; }

void M5StackTab5::mute(bool mute) {
  mute_ = mute;
  if (audio_) {
    audio_->mute(mute);
  }
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool M5StackTab5::is_muted() const { return mute_; }

size_t M5StackTab5::play_audio(const uint8_t *data, size_t num_bytes) {
  return audio_ ? audio_->play(data, num_bytes) : 0;
}

size_t M5StackTab5::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

void M5StackTab5::clear_audio() {
  if (audio_) {
    audio_->clear();
  }
}

void M5StackTab5::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (audio_) {
    audio_->microphone_volume(mic_volume_);
  }
}

float M5StackTab5::microphone_volume() const { return mic_volume_; }

bool M5StackTab5::start_audio_recording(
    std::function<void(const uint8_t *data, size_t length)> callback) {
  if (!is_audio_initialized()) {
    logger_.error("Audio system not initialized");
    return false;
  }
  if (!audio_->is_recording_enabled()) {
    logger_.error("Audio recording was not enabled in initialize_audio()");
    return false;
  }
  if (!audio_->start_recording(callback)) {
    return false;
  }
  logger_.info("Audio recording started");
  return true;
}

void M5StackTab5::stop_audio_recording() {
  if (audio_) {
    audio_->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

uint32_t M5StackTab5::audio_sample_rate() const { return audio_ ? audio_->sample_rate() : 0; }

size_t M5StackTab5::audio_buffer_size() const { return audio_ ? audio_->buffer_size() : 0; }

bool M5StackTab5::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  if (!is_audio_initialized()) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio_->sample_rate(sample_rate);
}

} // namespace espp

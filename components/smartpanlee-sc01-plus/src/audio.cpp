#include "smartpanlee-sc01-plus.hpp"

#include <algorithm>

////////////////////////
// Audio Functions   //
////////////////////////

// The I2S channel, the playback task and the stream buffer live in the
// espp::Audio component; this file keeps only what is specific to the board:
// the pins. There is no codec, so espp::Audio applies volume and mute in
// software.

namespace espp {

bool SmartPanleeSc01Plus::initialize_audio() { return initialize_audio(AudioConfig{}); }

bool SmartPanleeSc01Plus::initialize_audio(uint32_t sample_rate) {
  return initialize_audio(AudioConfig{.sample_rate = sample_rate});
}

bool SmartPanleeSc01Plus::initialize_audio(uint32_t sample_rate,
                                           const espp::Task::BaseConfig &task_config) {
  return initialize_audio(AudioConfig{.sample_rate = sample_rate, .task_config = task_config});
}

bool SmartPanleeSc01Plus::initialize_audio(const AudioConfig &config) {
  logger_.info("Initializing audio playback at {} Hz", config.sample_rate);
  if (is_audio_initialized()) {
    logger_.warn("Audio already initialized");
    return true;
  }

  // the playback task is named "<name> out", like the other BSPs' (short
  // enough for FreeRTOS's 15-character task names)
  auto playback_task_config = config.task_config;
  playback_task_config.name =
      (config.task_config.name.empty() ? std::string("audio") : config.task_config.name) + " out";
  const auto pins = i2s_pins();
  auto audio = std::make_shared<espp::Audio>(espp::Audio::Config{
      .port = i2s_port,
      .pins = {.mclk = pins.mclk, .bclk = pins.bclk, .ws = pins.ws, .dout = pins.dout},
      .sample_rate = config.sample_rate,
      .playback_enabled = true,
      .recording_enabled = false,
      .playback_channels = 2,
      .playback_buffer_size = config.playback_buffer_size,
      .playback_buffer_caps = config.playback_buffer_caps,
      .volume = volume_,
      .muted = mute_,
      .playback_task_config = playback_task_config,
  });
  if (!audio->initialize()) {
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
  return true;
}

void SmartPanleeSc01Plus::deinitialize_audio() {
  // unpublished first: a forwarder called meanwhile then sees no audio rather
  // than one being torn down
  auto audio = audio_.exchange(nullptr);
  // release the I2S channel and task now, even if the application still holds
  // the object from audio()
  if (audio) {
    audio->deinitialize();
  }
}

void SmartPanleeSc01Plus::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
}

float SmartPanleeSc01Plus::volume() const { return volume_; }

void SmartPanleeSc01Plus::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
}

bool SmartPanleeSc01Plus::is_muted() const { return mute_; }

size_t SmartPanleeSc01Plus::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t SmartPanleeSc01Plus::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

void SmartPanleeSc01Plus::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

uint32_t SmartPanleeSc01Plus::audio_sample_rate() const {
  auto audio = audio_.load();
  return audio ? audio->sample_rate() : 0;
}

size_t SmartPanleeSc01Plus::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

bool SmartPanleeSc01Plus::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio->sample_rate(sample_rate);
}

} // namespace espp

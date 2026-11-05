#include "esp32-p4-wifi6-dev-kit.hpp"

#include <algorithm>

#include <driver/gpio.h>

////////////////////////
// Audio Functions   //
////////////////////////

// The I2S channels, the playback and microphone tasks, the stream buffer and
// the codec setup live in the espp::Audio component (with its ES8311
// adapter); this file keeps only what is specific to the board: the pins, the
// codec's I2C device and the speaker amplifier.

namespace espp {

bool Esp32P4Wifi6DevKit::initialize_audio() { return initialize_audio(AudioConfig{}); }

bool Esp32P4Wifi6DevKit::initialize_audio(uint32_t sample_rate,
                                          const espp::Task::BaseConfig &task_config) {
  // the old API set up playback only (initialize_microphone() added recording)
  return initialize_audio(AudioConfig{
      .sample_rate = sample_rate, .recording_enabled = false, .task_config = task_config});
}

bool Esp32P4Wifi6DevKit::initialize_audio(const AudioConfig &config) {
  const bool playback = config.playback_enabled;
  const bool recording = config.recording_enabled;
  logger_.info("Initializing audio (ES8311{}{}{}) at {} Hz", playback ? " playback" : "",
               playback && recording ? " +" : "", recording ? " recording" : "",
               config.sample_rate);
  if (is_audio_initialized()) {
    logger_.warn("Audio already initialized");
    return true;
  }
  if (!playback && !recording) {
    logger_.error("initialize_audio(): neither playback nor recording is enabled");
    return false;
  }

  // the speaker amplifier (NS4150B) enable GPIO, off until playback is up
  gpio_config_t pa_cfg{};
  pa_cfg.pin_bit_mask = 1ULL << static_cast<int>(audio_pa_enable_io);
  pa_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&pa_cfg);
  enable_audio(false);

  // the codec is on the internal I2C bus
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
  es8311_ = std::make_shared<espp::Es8311Codec>(espp::Es8311Codec::Config{
      .write = espp::make_i2c_addressed_write(es8311_i2c_device_),
      .read = espp::make_i2c_addressed_read_register(es8311_i2c_device_),
  });

  // the playback and microphone tasks share the caller's task config, named
  // "<name> out" / "<name> in" so the two are easy to tell apart in logs and
  // while debugging (short enough for FreeRTOS's 15-character task names)
  const std::string base_name =
      config.task_config.name.empty() ? std::string("audio") : config.task_config.name;
  auto playback_task_config = config.task_config;
  playback_task_config.name = base_name + " out";
  auto recording_task_config = config.task_config;
  recording_task_config.name = base_name + " in";
  // One I2S bus, full duplex: the ES8311's DAC (DSDIN) and ADC (ASDOUT) share
  // MCLK / SCLK / LRCK. The speaker is mono; the ADC drives the left slot,
  // which espp::Audio receives as stereo and hands over as mono.
  auto audio = std::make_shared<espp::Audio>(espp::Audio::Config{
      .port = audio_i2s_port,
      .pins = {.mclk = audio_mclk_io,
               .bclk = audio_sclk_io,
               .ws = audio_lrck_io,
               .dout = audio_dout_io,
               .din = audio_din_io},
      .sample_rate = config.sample_rate,
      .playback_enabled = playback,
      .recording_enabled = recording,
      .playback_channels = 1,
      .recording_channels = 1,
      .output_codec = es8311_,
      .input_codec = es8311_,
      .playback_buffer_size = config.playback_buffer_size,
      .playback_buffer_caps = config.playback_buffer_caps,
      .volume = volume_,
      .muted = mute_,
      .microphone_volume = mic_volume_,
      .playback_task_config = playback_task_config,
      .recording_task_config = recording_task_config,
  });
  if (!audio->initialize()) {
    audio.reset();
    deinitialize_audio();
    return false;
  }
  audio_config_ = config;
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

void Esp32P4Wifi6DevKit::deinitialize_audio() {
  // unpublished first: a forwarder called meanwhile (e.g. from a recording
  // callback still finishing) then sees no audio rather than one being torn
  // down
  auto audio = audio_.exchange(nullptr);
  if (audio && audio->is_initialized() && audio->is_playback_enabled()) {
    enable_audio(false);
  }
  // release the I2S channels, tasks and codec now, even if the application
  // still holds the object from audio()
  if (audio) {
    audio->deinitialize();
  }
  audio.reset();
  es8311_.reset();
  es8311_i2c_device_.reset();
}

void Esp32P4Wifi6DevKit::enable_audio(bool enable) {
  gpio_set_level(audio_pa_enable_io, enable ? 1 : 0);
  logger_.debug("Audio {}", enable ? "enabled" : "disabled");
}

void Esp32P4Wifi6DevKit::set_speaker_enabled(bool enable) { enable_audio(enable); }

void Esp32P4Wifi6DevKit::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
  logger_.debug("Volume set to {:.1f} %", volume_.load());
}

float Esp32P4Wifi6DevKit::volume() const { return volume_; }

void Esp32P4Wifi6DevKit::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool Esp32P4Wifi6DevKit::is_muted() const { return mute_; }

size_t Esp32P4Wifi6DevKit::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t Esp32P4Wifi6DevKit::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

void Esp32P4Wifi6DevKit::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

void Esp32P4Wifi6DevKit::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->microphone_volume(mic_volume_);
  }
}

float Esp32P4Wifi6DevKit::microphone_volume() const { return mic_volume_; }

bool Esp32P4Wifi6DevKit::start_audio_recording(
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

void Esp32P4Wifi6DevKit::stop_audio_recording() {
  if (auto audio = audio_.load()) {
    audio->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

bool Esp32P4Wifi6DevKit::initialize_microphone(const microphone_callback_t &callback,
                                               const espp::Task::BaseConfig &) {
  logger_.info("Initializing microphone");
  if (!is_audio_initialized()) {
    logger_.error("The audio subsystem must be initialized first: the ES8311 is a full-duplex "
                  "codec on a single I2S bus");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  // one snapshot: a concurrent deinitialize_audio() may unpublish it
  auto audio = audio_.load();
  if (!audio) {
    logger_.error("Audio system not initialized");
    return false;
  }
  if (!audio->is_recording_enabled()) {
    // the old API set recording up separately: set the audio system up again
    // with it
    AudioConfig config = audio_config_;
    config.recording_enabled = true;
    // at the current rate, which audio_sample_rate() may have changed since
    config.sample_rate = audio->sample_rate();
    audio.reset();
    deinitialize_audio();
    if (!initialize_audio(config)) {
      return false;
    }
  }
  return start_audio_recording(callback);
}

uint32_t Esp32P4Wifi6DevKit::audio_sample_rate() const {
  auto audio = audio_.load();
  return audio ? audio->sample_rate() : 0;
}

size_t Esp32P4Wifi6DevKit::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

bool Esp32P4Wifi6DevKit::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio->sample_rate(sample_rate);
}

} // namespace espp

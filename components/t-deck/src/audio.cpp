#include "t-deck.hpp"

#include <algorithm>

using namespace espp;

////////////////////////
// Audio Functions   //
////////////////////////

// The I2S channels, the playback and microphone tasks, the stream buffer and
// the codec setup live in the espp::Audio component (with its ES7210
// adapter); this file keeps only what is specific to the board. The speaker
// amplifier (MAX98357A, no codec: volume and mute in software) and the ES7210
// microphone ADC each have their own I2S bus, so each has its own
// espp::Audio.

TDeck::AudioConfig TDeck::default_audio_config() {
  return {.sample_rate = 48000,
          .recording_sample_rate = 16000,
          .playback_enabled = true,
          .recording_enabled = true,
          .playback_task_config = {.name = "tdeck audio out",
                                   .stack_size_bytes = 4096,
                                   .priority = 19,
                                   .core_id = 1},
          .recording_task_config = {
              .name = "tdeck audio in", .stack_size_bytes = 4096, .priority = 19, .core_id = 1}};
}

bool TDeck::initialize_audio() { return initialize_audio(default_audio_config()); }

bool TDeck::initialize_sound(uint32_t default_audio_rate,
                             const espp::Task::BaseConfig &task_config) {
  // the old API set up playback only (initialize_microphone() added recording)
  auto config = default_audio_config();
  config.sample_rate = default_audio_rate;
  config.recording_enabled = false;
  config.playback_task_config = task_config;
  return initialize_audio(config);
}

bool TDeck::initialize_audio(const AudioConfig &config) {
  const bool playback = config.playback_enabled;
  const bool recording = config.recording_enabled;
  logger_.info("Initializing audio ({}{}{})", playback ? "playback" : "",
               playback && recording ? " + " : "", recording ? "recording" : "");
  if (is_audio_initialized()) {
    logger_.warn("Audio already initialized");
    return true;
  }
  if (!playback && !recording) {
    logger_.error("initialize_audio(): neither playback nor recording is enabled");
    return false;
  }
  if (playback && !initialize_playback(config)) {
    deinitialize_audio();
    return false;
  }
  if (recording && !initialize_recording(config)) {
    deinitialize_audio();
    return false;
  }
  return true;
}

bool TDeck::initialize_playback(const AudioConfig &config) {
  // the speaker's own I2S bus: this instance only plays
  auto app = config;
  app.playback_enabled = true;
  app.recording_enabled = false;
  auto audio = std::make_shared<espp::Audio>(espp::Audio::Config{
      .app = app,
      .hardware = {.port = i2s_port,
                   .pins = {.bclk = i2s_bck_io, .ws = i2s_ws_io, .dout = i2s_do_io},
                   .playback_channels = 2},
      .volume = volume_,
      .muted = mute_,
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

bool TDeck::initialize_recording(const AudioConfig &config) {
  // the ES7210 is on the internal I2C bus
  std::error_code ec;
  es7210_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = es7210_i2c_address,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!es7210_i2c_device_) {
    logger_.error("Could not initialize ES7210 I2C device: {}", ec.message());
    return false;
  }
  // Enable all four ADC channels so the ES7210 always emits a fixed 4-slot
  // TDM frame ([MIC1, MIC2, MIC3, MIC4]). Enabling only a subset can make the
  // codec pack fewer slots, which shifts the frame boundary against the
  // 4-slot read and periodically lands the extraction on a floating, railing
  // slot (heard as a robotic / static buzz on top of the audio). The T-Deck
  // populates only MIC1 and MIC3 (one from each ADC pair), so the recording
  // keeps slots 0 and 2. The high-pass filter strips the electret mics' DC
  // offset, which the analog gain would otherwise amplify into the rails.
  es7210_ = std::make_shared<espp::Es7210Codec>(espp::Es7210Codec::Config{
      .write = espp::make_i2c_addressed_write(es7210_i2c_device_),
      .read = espp::make_i2c_addressed_read_register(es7210_i2c_device_),
      .address = es7210_i2c_address,
      .microphones = static_cast<es7210_input_mics_t>(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2 |
                                                      ES7210_INPUT_MIC3 | ES7210_INPUT_MIC4),
      .high_pass_filter = true,
  });
  // NOTE: register 0x08 (master/slave & channels) is left at its reset
  // default, which is correct in slave mode. Forcing 0x08 = 0x20 (as some
  // MicroPython T-Deck ports do for their async PWM MCLK) turns the capture on
  // this hardware-MCLK setup into a half-rate, every-other-frame-zero one.
  // the microphones' own I2S bus: this instance only records, at
  // recording_sample_rate (sample_rate if 0)
  auto app = config;
  app.playback_enabled = false;
  app.recording_enabled = true;
  auto microphone = std::make_shared<espp::Audio>(espp::Audio::Config{
      .app = app,
      .hardware = {.port = mic_i2s_port,
                   .pins = {.mclk = es7210_mclk_io,
                            .bclk = es7210_sclk_io,
                            .ws = es7210_lrck_io,
                            .din = es7210_sdout_io},
                   .recording_channels = 2,
                   .recording_format = espp::Audio::RecordingFormat::TDM,
                   .recording_tdm_slots = 4,
                   .recording_slots = {0, 2}, // MIC1 -> left, MIC3 -> right
                   .input_codec = es7210_},
      .microphone_volume = mic_volume_,
  });
  if (!microphone->initialize()) {
    microphone.reset();
    deinitialize_recording();
    return false;
  }
  microphone_audio_.store(microphone);
  // as for the playback settings (see initialize_playback())
  float mic_volume;
  do {
    mic_volume = mic_volume_;
    microphone->microphone_volume(mic_volume);
  } while (mic_volume != mic_volume_);
  return true;
}

void TDeck::deinitialize_recording() {
  // unpublished first: a forwarder called meanwhile (e.g. from a recording
  // callback still finishing) then sees no audio rather than one being torn
  // down
  auto microphone = microphone_audio_.exchange(nullptr);
  // release the I2S channel, task and codec now, even if the application
  // still holds the object from microphone_audio()
  if (microphone) {
    microphone->deinitialize();
  }
  microphone.reset();
  es7210_.reset();
  es7210_i2c_device_.reset();
}

void TDeck::deinitialize_audio() {
  // as for the microphone (see deinitialize_recording())
  auto audio = audio_.exchange(nullptr);
  if (audio) {
    audio->deinitialize();
  }
  audio.reset();
  deinitialize_recording();
}

void TDeck::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
}

bool TDeck::is_muted() const { return mute_; }

void TDeck::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
}

float TDeck::volume() const { return volume_; }

uint32_t TDeck::audio_sample_rate() const {
  auto audio = audio_.load();
  return audio ? audio->sample_rate() : 0;
}

size_t TDeck::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

bool TDeck::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.warn("Playback not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio->sample_rate(sample_rate);
}

size_t TDeck::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t TDeck::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

size_t TDeck::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

void TDeck::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

//////////////////////////
// Microphone Functions //
//////////////////////////

bool TDeck::start_audio_recording(const microphone_callback_t &callback) {
  auto microphone = microphone_audio_.load();
  if (!microphone || !microphone->is_initialized()) {
    logger_.error("Audio recording was not enabled in initialize_audio()");
    return false;
  }
  if (!microphone->start_recording(callback)) {
    return false;
  }
  logger_.info("Audio recording started");
  return true;
}

void TDeck::stop_audio_recording() {
  if (auto microphone = microphone_audio_.load()) {
    microphone->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

bool TDeck::initialize_microphone(const microphone_callback_t &callback, uint32_t sample_rate,
                                  const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  // the microphones have their own I2S bus: set (only) them up again if they
  // are not running at this rate
  if (microphone_sample_rate() != sample_rate) {
    deinitialize_recording();
    auto config = default_audio_config();
    config.recording_sample_rate = sample_rate;
    config.recording_task_config = task_config;
    if (!initialize_recording(config)) {
      return false;
    }
  }
  return start_audio_recording(callback);
}

uint32_t TDeck::microphone_sample_rate() const {
  auto microphone = microphone_audio_.load();
  return microphone ? microphone->sample_rate() : 0;
}

void TDeck::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto microphone = microphone_audio_.load()) {
    microphone->microphone_volume(mic_volume_);
  }
}

float TDeck::microphone_volume() const { return mic_volume_; }

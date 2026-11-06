#include "m5stack-cardputer.hpp"

#include <algorithm>
#include <cstring>

#include "cardputer_adv_codec.hpp"

using namespace espp;

////////////////////////
// Audio Functions    //
////////////////////////

// The I2S channels, the playback and microphone tasks and the stream buffer
// live in the espp::Audio component; this file keeps only what is specific to
// the board: which instance(s) each variant uses, their pins, the ADV's
// BCLK-clocked ES8311 (CardputerAdvCodec) and the original's software
// microphone gain.

M5StackCardputer::AudioConfig M5StackCardputer::default_audio_config() {
  return {
      .sample_rate = 44100,
      .playback_enabled = true,
      .recording_enabled = false,
      .playback_task_config = {.name = "audio out",
                               .stack_size_bytes = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_STACK_SIZE,
                               .priority = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_PRIORITY,
                               .core_id = CONFIG_M5STACK_CARDPUTER_AUDIO_TASK_CORE_ID},
      .recording_task_config = {.name = "audio in",
                                .stack_size_bytes =
                                    CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_STACK_SIZE,
                                .priority = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_PRIORITY,
                                .core_id = CONFIG_M5STACK_CARDPUTER_MICROPHONE_TASK_CORE_ID}};
}

bool M5StackCardputer::initialize_audio() { return initialize_audio(default_audio_config()); }

bool M5StackCardputer::initialize_audio(const AudioConfig &config) {
  const bool playback = config.playback_enabled;
  const bool recording = config.recording_enabled;
  const bool is_adv = variant() == Variant::ADV;
  logger_.info("Initializing audio ({}{}{}{}) at {} Hz", is_adv ? "ES8311 " : "",
               playback ? "playback" : "", playback && recording ? " + " : "",
               recording ? "recording" : "", config.sample_rate);
  if (is_audio_initialized()) {
    logger_.warn("Audio already initialized");
    return true;
  }
  if (!playback && !recording) {
    logger_.error("initialize_audio(): neither playback nor recording is enabled");
    return false;
  }
  if (!is_adv && playback && recording) {
    logger_.error("initialize_audio(): the original Cardputer cannot play and record at the same "
                  "time: GPIO {} is both the speaker's word select and the PDM microphone's clock",
                  static_cast<int>(i2s_ws_io));
    return false;
  }

  auto base_config = [&]() {
    espp::Audio::Config c{};
    c.app = config;
    c.hardware.playback_channels = 1;
    c.hardware.recording_channels = 1;
    c.volume = volume_;
    c.muted = mute_;
    c.microphone_volume = mic_volume_;
    return c;
  };

  std::shared_ptr<espp::Audio> playback_audio;
  std::shared_ptr<espp::Audio> recording_audio;
  if (is_adv) {
    // One I2S bus, full duplex: the ES8311's DAC (DSDIN) and ADC (ASDOUT)
    // share BCLK / LRCK (there is no MCLK: the codec is clocked from BCLK).
    // The speaker is mono; the ADC drives the left slot, which espp::Audio
    // receives as stereo and hands over as mono.
    adv_codec_ = std::make_shared<espp::CardputerAdvCodec>(
        [this](uint8_t reg, uint8_t value) { return es8311_write(reg, value); });
    auto c = base_config();
    c.hardware.port = i2s_port;
    c.hardware.pins = {.bclk = i2s_bck_io,
                       .ws = i2s_ws_io,
                       .dout = playback ? i2s_do_io : GPIO_NUM_NC,
                       .din = recording ? mic_data_io : GPIO_NUM_NC};
    c.app.playback_enabled = playback;
    c.app.recording_enabled = recording;
    c.hardware.output_codec = adv_codec_;
    c.hardware.input_codec = adv_codec_;
    auto audio = std::make_shared<espp::Audio>(c);
    if (!audio->initialize()) {
      audio.reset();
      deinitialize_audio();
      return false;
    }
    if (playback) {
      playback_audio = audio;
    }
    if (recording) {
      recording_audio = audio;
    }
  } else if (playback) {
    // the NS4168 amplifier: no codec, so espp::Audio applies the volume in
    // software
    auto c = base_config();
    c.hardware.port = i2s_port;
    c.hardware.pins = {.bclk = i2s_bck_io, .ws = i2s_ws_io, .dout = i2s_do_io};
    c.app.playback_enabled = true;
    c.app.recording_enabled = false;
    playback_audio = std::make_shared<espp::Audio>(c);
    if (!playback_audio->initialize()) {
      playback_audio.reset();
      deinitialize_audio();
      return false;
    }
  } else {
    // the SPM1423 PDM microphone, on its own I2S port: its clock on GPIO 43
    // (the speaker's word select), its data on GPIO 46
    auto c = base_config();
    c.hardware.port = mic_i2s_port;
    c.hardware.pins = {.ws = mic_clk_io, .din = mic_data_io};
    c.app.playback_enabled = false;
    c.app.recording_enabled = true;
    c.hardware.recording_format = espp::Audio::RecordingFormat::PDM;
    recording_audio = std::make_shared<espp::Audio>(c);
    if (!recording_audio->initialize()) {
      recording_audio.reset();
      deinitialize_audio();
      return false;
    }
    // the software gain's buffer, sized for a block (1/60 s) so the
    // microphone task does not allocate
    mic_scaled_.resize(config.sample_rate / espp::Audio::UPDATE_FREQUENCY);
  }
  audio_config_ = config;
  // published once they are initialized; the forwarders below each take one
  // snapshot
  audio_.store(playback_audio);
  microphone_audio_.store(recording_audio);
  // A setter that ran meanwhile only updated its cached value: forward the
  // latest ones, again while they change, so one racing this is not undone by
  // a stale value.
  if (playback_audio) {
    float volume;
    do {
      volume = volume_;
      playback_audio->volume(volume);
    } while (volume != volume_);
    bool muted;
    do {
      muted = mute_;
      playback_audio->mute(muted);
    } while (muted != mute_);
  }
  if (recording_audio) {
    float mic_volume;
    do {
      mic_volume = mic_volume_;
      recording_audio->microphone_volume(mic_volume);
    } while (mic_volume != mic_volume_);
  }
  return true;
}

void M5StackCardputer::deinitialize_audio() {
  // unpublished first: a forwarder called meanwhile (e.g. from a recording
  // callback still finishing) then sees no audio rather than one being torn
  // down
  auto playback_audio = audio_.exchange(nullptr);
  auto recording_audio = microphone_audio_.exchange(nullptr);
  // release the I2S channels, tasks and codec now, even if the application
  // still holds an object from audio() / microphone_audio()
  if (playback_audio) {
    playback_audio->deinitialize();
  }
  if (recording_audio) {
    recording_audio->deinitialize();
  }
  playback_audio.reset();
  recording_audio.reset();
  adv_codec_.reset();
  std::vector<int16_t>().swap(mic_scaled_);
}

void M5StackCardputer::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
  logger_.debug("Volume set to {:.1f} %", volume_.load());
}

float M5StackCardputer::volume() const { return volume_; }

void M5StackCardputer::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool M5StackCardputer::is_muted() const { return mute_; }

uint32_t M5StackCardputer::audio_sample_rate() const {
  if (auto audio = audio_.load()) {
    return audio->sample_rate();
  }
  auto microphone = microphone_audio_.load();
  return microphone ? microphone->sample_rate() : 0;
}

bool M5StackCardputer::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto playback_audio = audio_.load();
  auto recording_audio = microphone_audio_.load();
  if (!playback_audio && !recording_audio) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  bool ok = true;
  if (playback_audio) {
    ok = playback_audio->sample_rate(sample_rate);
  }
  if (recording_audio && recording_audio != playback_audio) {
    ok = recording_audio->sample_rate(sample_rate) && ok;
  }
  return ok;
}

size_t M5StackCardputer::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

size_t M5StackCardputer::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

size_t M5StackCardputer::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t M5StackCardputer::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

void M5StackCardputer::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

bool M5StackCardputer::start_audio_recording(
    std::function<void(const uint8_t *data, size_t length)> callback) {
  auto microphone = microphone_audio_.load();
  if (!microphone || !microphone->is_initialized()) {
    logger_.error("Audio recording was not enabled in initialize_audio()");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  recording_callback_ = callback;
  espp::Audio::recording_callback_t recording_callback = callback;
  if (variant() == Variant::ORIGINAL) {
    // the PDM microphone has no hardware gain, so apply the microphone volume
    // in software (75 % = unity; the ADV scales in the codec instead)
    recording_callback = [this, callback](const uint8_t *data, size_t length) {
      const float scale = mic_volume_ / 75.0f;
      const size_t count = length / sizeof(int16_t);
      if (scale == 1.0f) {
        callback(data, length);
        return;
      }
      if (mic_scaled_.size() < count) {
        mic_scaled_.resize(count);
      }
      std::memcpy(mic_scaled_.data(), data, count * sizeof(int16_t));
      for (size_t i = 0; i < count; i++) {
        auto v = static_cast<int32_t>(mic_scaled_[i] * scale);
        mic_scaled_[i] = static_cast<int16_t>(std::clamp<int32_t>(v, INT16_MIN, INT16_MAX));
      }
      callback(reinterpret_cast<const uint8_t *>(mic_scaled_.data()), count * sizeof(int16_t));
    };
  }
  if (!microphone->start_recording(recording_callback)) {
    return false;
  }
  logger_.info("Audio recording started");
  return true;
}

void M5StackCardputer::stop_audio_recording() {
  if (auto microphone = microphone_audio_.load()) {
    microphone->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

void M5StackCardputer::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  // the ADV's codec gain; the original's software gain reads mic_volume_
  if (auto microphone = microphone_audio_.load()) {
    microphone->microphone_volume(mic_volume_);
  }
}

float M5StackCardputer::microphone_volume() const { return mic_volume_; }

uint32_t M5StackCardputer::microphone_sample_rate() const {
  auto microphone = microphone_audio_.load();
  return microphone ? microphone->sample_rate() : 0;
}

bool M5StackCardputer::initialize_sound(uint32_t default_audio_rate,
                                        const espp::Task::BaseConfig &task_config) {
  if (audio_.load()) {
    logger_.warn("Sound already initialized");
    return true;
  }
  auto microphone = microphone_audio_.load();
  if (!microphone) {
    auto config = default_audio_config();
    config.sample_rate = default_audio_rate;
    config.playback_enabled = true;
    config.recording_enabled = false;
    config.playback_task_config = task_config;
    return initialize_audio(config);
  }
  if (variant() != Variant::ADV) {
    logger_.error("Cannot initialize sound: the microphone is active and shares GPIO {} with "
                  "the speaker (WS / PDM clock)",
                  static_cast<int>(i2s_ws_io));
    return false;
  }
  // the ADV's full-duplex bus: set it up again with playback added, at the
  // microphone's rate, resuming a recording in progress
  AudioConfig config = audio_config_;
  config.playback_enabled = true;
  config.playback_task_config = task_config;
  config.sample_rate = microphone->sample_rate();
  if (default_audio_rate != config.sample_rate) {
    logger_.warn("I2S already configured at {} Hz; ignoring requested {} Hz (the ADV speaker and "
                 "microphone share the I2S clock)",
                 config.sample_rate, default_audio_rate);
  }
  const bool was_recording = microphone->is_recording();
  microphone.reset();
  deinitialize_audio();
  if (!initialize_audio(config)) {
    return false;
  }
  return !was_recording || start_audio_recording(recording_callback_);
}

bool M5StackCardputer::initialize_microphone(const microphone_callback_t &callback,
                                             uint32_t sample_rate,
                                             const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }
  if (!microphone_audio_.load()) {
    // one snapshot: a concurrent deinitialize_audio() may unpublish it
    auto playback_audio = audio_.load();
    if (!playback_audio) {
      auto config = default_audio_config();
      config.sample_rate = sample_rate;
      config.playback_enabled = false;
      config.recording_enabled = true;
      config.recording_task_config = task_config;
      if (!initialize_audio(config)) {
        return false;
      }
    } else if (variant() != Variant::ADV) {
      logger_.error("Cannot initialize microphone: the sound subsystem is active and shares GPIO "
                    "{} with the microphone (WS / PDM clock)",
                    static_cast<int>(mic_clk_io));
      return false;
    } else {
      // the ADV's full-duplex bus: set it up again with recording added, at
      // the speaker's rate
      AudioConfig config = audio_config_;
      config.recording_enabled = true;
      config.recording_task_config = task_config;
      config.sample_rate = playback_audio->sample_rate();
      if (sample_rate != config.sample_rate) {
        logger_.warn("I2S already configured at {} Hz; ignoring requested {} Hz (the ADV speaker "
                     "and microphone share the I2S clock)",
                     config.sample_rate, sample_rate);
      }
      playback_audio.reset();
      deinitialize_audio();
      if (!initialize_audio(config)) {
        return false;
      }
    }
  }
  return start_audio_recording(callback);
}

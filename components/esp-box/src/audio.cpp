#include "esp-box.hpp"

#include <algorithm>

#include <driver/gpio.h>

////////////////////////
// Audio Functions   //
////////////////////////

// The I2S channels, the playback and microphone tasks, the stream buffer and
// the codec setup live in the espp::Audio component (with its ES8311 and
// ES7210 adapters); this file keeps only what is specific to the board: the
// pins, the codecs' I2C devices and the speaker power pin.

namespace espp {

EspBox::AudioConfig EspBox::default_audio_config() {
  return {.sample_rate = 48000,
          .playback_enabled = true,
          .recording_enabled = true,
          .playback_task_config = {.name = "box audio out",
                                   .stack_size_bytes = 4096,
                                   .priority = 19,
                                   .core_id = 1},
          .recording_task_config = {
              .name = "box audio in", .stack_size_bytes = 4096, .priority = 19, .core_id = 1}};
}

bool EspBox::initialize_audio() { return initialize_audio(default_audio_config()); }

bool EspBox::initialize_sound(uint32_t default_audio_rate,
                              const espp::Task::BaseConfig &task_config) {
  // the old API set up playback only (initialize_microphone() added recording)
  auto config = default_audio_config();
  config.sample_rate = default_audio_rate;
  config.recording_enabled = false;
  config.playback_task_config = task_config;
  return initialize_audio(config);
}

bool EspBox::initialize_audio(const AudioConfig &config) {
  const bool playback = config.playback_enabled;
  const bool recording = config.recording_enabled;
  logger_.info("Initializing audio ({}{}{}) at {} Hz", playback ? "ES8311 playback" : "",
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

  // the speaker power pin, off until playback is up
  gpio_config_t power_cfg{};
  power_cfg.pin_bit_mask = 1ULL << static_cast<int>(sound_power_pin);
  power_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&power_cfg);
  enable_audio(false);

  // both codecs are on the internal I2C bus
  std::error_code ec;
  if (playback) {
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
      deinitialize_audio();
      return false;
    }
    es8311_ = std::make_shared<espp::Es8311Codec>(espp::Es8311Codec::Config{
        .write = espp::make_i2c_addressed_write(es8311_i2c_device_),
        .read = espp::make_i2c_addressed_read_register(es8311_i2c_device_),
    });
  }
  if (recording) {
    // the ES7210's address depends on how its AD pins are strapped: accept
    // either 0x40 or 0x41 and report clearly if neither responds
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
      deinitialize_audio();
      return false;
    }
    logger_.info("Found ES7210 at {:#04x}", es7210_address);
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
      deinitialize_audio();
      return false;
    }
    es7210_ = std::make_shared<espp::Es7210Codec>(espp::Es7210Codec::Config{
        .write = espp::make_i2c_addressed_write(es7210_i2c_device_),
        .read = espp::make_i2c_addressed_read_register(es7210_i2c_device_),
        .address = es7210_address,
        // only the two microphones are wired on the box (the other ES7210
        // inputs are unused), and they map onto the two I2S slots
        .microphones = static_cast<es7210_input_mics_t>(ES7210_INPUT_MIC1 | ES7210_INPUT_MIC2),
        .high_pass_filter = true,
    });
  }

  // One I2S bus, full duplex: the ES8311's DAC (DSDIN) and the ES7210's ADC
  // (SDOUT) share MCLK / SCLK / LRCK. Both are stereo: the speaker plays the
  // interleaved frames, and the ES7210 puts microphone 1 on the left slot and
  // microphone 2 on the right.
  auto audio = std::make_shared<espp::Audio>(espp::Audio::Config{
      .app = config,
      .hardware = {.port = i2s_port,
                   .pins = {.mclk = i2s_mck_io,
                            .bclk = i2s_bck_io,
                            .ws = i2s_ws_io,
                            .dout = i2s_do_io,
                            .din = i2s_di_io},
                   .playback_channels = 2,
                   .recording_channels = 2,
                   .output_codec = es8311_,
                   .input_codec = es7210_},
      .volume = volume_,
      .muted = mute_,
      .microphone_volume = mic_volume_,
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

void EspBox::deinitialize_audio() {
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
  es8311_.reset();
  es8311_i2c_device_.reset();
  es7210_.reset();
  es7210_i2c_device_.reset();
}

void EspBox::enable_audio(bool enable) {
  gpio_set_level(sound_power_pin, enable ? 1 : 0);
  logger_.debug("Audio {}", enable ? "enabled" : "disabled");
}

void EspBox::enable_sound(bool enable) { enable_audio(enable); }

void EspBox::volume(float volume) {
  volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->volume(volume_);
  }
  logger_.debug("Volume set to {:.1f} %", volume_.load());
}

float EspBox::volume() const { return volume_; }

void EspBox::mute(bool mute) {
  mute_ = mute;
  if (auto audio = audio_.load()) {
    audio->mute(mute);
  }
  logger_.debug("Audio {}", mute ? "muted" : "unmuted");
}

bool EspBox::is_muted() const { return mute_; }

size_t EspBox::play_audio(const uint8_t *data, size_t num_bytes) {
  auto audio = audio_.load();
  return audio ? audio->play(data, num_bytes) : 0;
}

size_t EspBox::play_audio(std::span<const uint8_t> data) {
  return play_audio(data.data(), data.size());
}

size_t EspBox::play_audio(const std::vector<uint8_t> &data) {
  return play_audio(data.data(), data.size());
}

void EspBox::clear_audio() {
  if (auto audio = audio_.load()) {
    audio->clear();
  }
}

void EspBox::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (auto audio = audio_.load()) {
    audio->microphone_volume(mic_volume_);
  }
}

float EspBox::microphone_volume() const { return mic_volume_; }

bool EspBox::start_audio_recording(
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

void EspBox::stop_audio_recording() {
  if (auto audio = audio_.load()) {
    audio->stop_recording();
  }
  logger_.info("Audio recording stopped");
}

bool EspBox::initialize_microphone(const microphone_callback_t &callback,
                                   const espp::Task::BaseConfig &) {
  logger_.info("Initializing microphone");
  if (!is_audio_initialized()) {
    logger_.error("The audio subsystem must be initialized first: the ES7210 shares the I2S bus "
                  "with the ES8311 in full duplex");
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

uint32_t EspBox::audio_sample_rate() const {
  auto audio = audio_.load();
  return audio ? audio->sample_rate() : 0;
}

size_t EspBox::audio_buffer_size() const {
  auto audio = audio_.load();
  return audio ? audio->buffer_size() : 0;
}

bool EspBox::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  auto audio = audio_.load();
  if (!audio || !audio->is_initialized()) {
    logger_.warn("Audio not initialized: pass the sample rate to initialize_audio()");
    return false;
  }
  return audio->sample_rate(sample_rate);
}

} // namespace espp

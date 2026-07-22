#include <algorithm>
#include <cmath>

#include "m5stack-cardputer.hpp"

using namespace espp;

//////////////////////////
// Microphone Functions //
//////////////////////////

bool M5StackCardputer::initialize_microphone(const microphone_callback_t &callback,
                                             uint32_t sample_rate,
                                             const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing microphone");
  if (microphone_initialized_) {
    logger_.warn("Microphone already initialized, not initializing again!");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }

  bool is_adv = variant() == Variant::ADV;

  if (!is_adv && sound_initialized_) {
    // the original cannot run the speaker and PDM microphone at the same
    // time: GPIO 43 doubles as the speaker WS and the microphone clock
    logger_.error("Cannot initialize microphone: the sound subsystem is active and shares GPIO "
                  "{} with the microphone (WS / PDM clock)",
                  static_cast<int>(mic_clk_io));
    return false;
  }

  microphone_callback_ = callback;

  if (is_adv) {
    // The ADV's analog MEMS microphone goes through the ES8311 codec's ADC:
    // full duplex with the speaker on a single I2S bus (shared BCK/WS, ADC
    // data out on GPIO 46). Create (or reuse) the shared channels; the
    // sample rate is shared with the speaker.
    if (!ensure_adv_i2s(sample_rate)) {
      logger_.error("Could not initialize I2S driver");
      return false;
    }
    mic_sample_rate_ = audio_std_cfg.clk_cfg.sample_rate_hz;
    // the shared-bus RX channel captures both 16-bit slots of each frame
    // (see ensure_adv_i2s()); the task callback keeps only the left slot
    mic_stereo_capture_ = true;
  } else {
    // The original's SPM1423 is a PDM microphone (clock on GPIO 43, data on
    // GPIO 46), on its own I2S port
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(mic_i2s_port, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &audio_rx_handle));
    mic_pdm_cfg = {
        .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg =
            {
                .clk = mic_clk_io,
                .din = mic_data_io,
                .invert_flags = {.clk_inv = false},
            },
    };
    ESP_ERROR_CHECK(i2s_channel_init_pdm_rx_mode(audio_rx_handle, &mic_pdm_cfg));
    mic_sample_rate_ = sample_rate;
  }

  // buffer roughly one update period's worth of samples (at the actual,
  // possibly shared, rate); a stereo capture carries two words per sample
  // period
  size_t rx_buffer_size = calc_audio_buffer_size(mic_sample_rate_);
  if (mic_stereo_capture_) {
    rx_buffer_size *= 2;
  }
  audio_rx_buffer.resize(rx_buffer_size);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  // On the ADV, configure the codec's ADC path now that BCLK is running (the
  // codec derives its clock from it)
  if (is_adv) {
    if (!initialize_es8311_microphone()) {
      logger_.error("Could not initialize the ES8311 codec");
      i2s_channel_disable(audio_rx_handle);
      if (!sound_initialized_) {
        // the channels are shared with the speaker; only tear them down if
        // it is not using them
        i2s_del_channel(audio_tx_handle);
        i2s_del_channel(audio_rx_handle);
        audio_tx_handle = nullptr;
        audio_rx_handle = nullptr;
      }
      return false;
    }
  }

  using namespace std::placeholders;
  microphone_task_ = espp::Task::make_unique({
      .callback = std::bind(&M5StackCardputer::microphone_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  microphone_initialized_ = true;

  return microphone_task_->start();
}

uint32_t M5StackCardputer::microphone_sample_rate() const { return mic_sample_rate_; }

void M5StackCardputer::microphone_volume(float volume) {
  mic_volume_ = std::clamp(volume, 0.0f, 100.0f);
  if (variant() == Variant::ADV && microphone_initialized_) {
    // the ES8311 scales in hardware; initialize_es8311_microphone() applies
    // the stored volume when the microphone is initialized later
    es8311_write(0x17, static_cast<uint8_t>(std::lround(mic_volume_ / 100.0f * 255.0f)));
  }
}

float M5StackCardputer::microphone_volume() const { return mic_volume_; }

bool M5StackCardputer::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                                bool &task_notified) {
  size_t bytes_read = 0;
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, portMAX_DELAY);
  if (err == ESP_OK && bytes_read > 0 && microphone_callback_) {
    if (mic_stereo_capture_) {
      // compact the L,R word pairs down to mono in place, keeping the left
      // slot (the ES8311's ADC data)
      auto *samples = reinterpret_cast<int16_t *>(audio_rx_buffer.data());
      size_t num_frames = bytes_read / (2 * sizeof(int16_t));
      for (size_t i = 0; i < num_frames; i++) {
        samples[i] = samples[2 * i];
      }
      bytes_read = num_frames * sizeof(int16_t);
    } else {
      // the original's PDM microphone has no hardware gain, so apply the
      // microphone volume in software (75% = unity; the ADV scales in the
      // codec instead)
      float scale = mic_volume_ / 75.0f;
      if (scale != 1.0f) {
        auto *samples = reinterpret_cast<int16_t *>(audio_rx_buffer.data());
        size_t num_samples = bytes_read / sizeof(int16_t);
        for (size_t i = 0; i < num_samples; i++) {
          auto v = static_cast<int32_t>(samples[i] * scale);
          samples[i] = static_cast<int16_t>(std::clamp<int32_t>(v, INT16_MIN, INT16_MAX));
        }
      }
    }
    microphone_callback_(audio_rx_buffer.data(), bytes_read);
  }
  return false; // don't stop the task
}

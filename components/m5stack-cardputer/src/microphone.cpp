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
  if (sound_initialized_) {
    logger_.error("Cannot initialize microphone: the sound subsystem is active and shares I2S "
                  "pins with the microphone");
    return false;
  }
  if (!callback) {
    logger_.error("A callback is required to receive the recorded audio data");
    return false;
  }

  microphone_callback_ = callback;

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(mic_i2s_port, I2S_ROLE_MASTER);
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, nullptr, &audio_rx_handle));

  if (variant() == Variant::ADV) {
    // The ADV's analog MEMS microphone goes through the ES8311 codec's ADC,
    // which streams standard I2S on the same BCK/WS as the speaker with its
    // data out (ASDOUT) on GPIO 46
    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {.mclk = GPIO_NUM_NC,
                     .bclk = i2s_bck_io,
                     .ws = i2s_ws_io,
                     .dout = GPIO_NUM_NC,
                     .din = mic_data_io,
                     .invert_flags = {.mclk_inv = false, .bclk_inv = false, .ws_inv = false}},
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_rx_handle, &std_cfg));
  } else {
    // The original's SPM1423 is a PDM microphone (clock on GPIO 43, data on
    // GPIO 46)
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
  }

  mic_sample_rate_ = sample_rate;

  // buffer roughly one update period's worth of samples
  audio_rx_buffer.resize(sample_rate * NUM_BYTES_PER_CHANNEL / UPDATE_FREQUENCY);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_rx_handle));

  // On the ADV, configure the codec's ADC path now that BCLK is running (the
  // codec derives its clock from it)
  if (variant() == Variant::ADV) {
    if (!initialize_es8311_microphone()) {
      logger_.error("Could not initialize the ES8311 codec");
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

bool M5StackCardputer::microphone_task_callback(std::mutex &m, std::condition_variable &cv,
                                                bool &task_notified) {
  size_t bytes_read = 0;
  auto err = i2s_channel_read(audio_rx_handle, audio_rx_buffer.data(), audio_rx_buffer.size(),
                              &bytes_read, portMAX_DELAY);
  if (err == ESP_OK && bytes_read > 0 && microphone_callback_) {
    microphone_callback_(audio_rx_buffer.data(), bytes_read);
  }
  return false; // don't stop the task
}

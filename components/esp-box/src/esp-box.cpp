#include "esp-box.hpp"

using namespace espp;

EspBox::EspBox() { detect(); }

void EspBox::detect() {
  // probe the internal i2c bus for the gt911 and the tt21100 if we find the
  // gt911, we will use it as the touch controller, and detect that the box is
  // BOX3. If we find the tt21100, we will use it as the touch controller, and
  // detect that the box is BOX. If we find neither, something is wrong.
  bool found_gt911 = internal_i2c_.probe_device(espp::Gt911::DEFAULT_ADDRESS_1) ||
                     internal_i2c_.probe_device(espp::Gt911::DEFAULT_ADDRESS_2);

  bool found_tt21100 = internal_i2c_.probe_device(espp::Tt21100::DEFAULT_ADDRESS);

  if (found_gt911) {
    box_type_ = BoxType::BOX3;
    backlight_io = box3::backlight_io;
    reset_value = box3::reset_value;
    i2s_ws_io = box3::i2s_ws_io;
  } else if (found_tt21100) {
    box_type_ = BoxType::BOX;
    backlight_io = box::backlight_io;
    reset_value = box::reset_value;
    i2s_ws_io = box::i2s_ws_io;
  } else {
    box_type_ = BoxType::UNKNOWN;
  }
}

////////////////////////
// Touchpad Functions //
////////////////////////

bool EspBox::initialize_touch() {
  switch (box_type_) {
  case BoxType::BOX3:
    gt911_ = std::make_unique<espp::Gt911>(espp::Gt911::Config{
        .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN});
    break;
  case BoxType::BOX:
    tt21100_ = std::make_unique<espp::Tt21100>(espp::Tt21100::Config{
        .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN});
    break;
  default:
    return false;
  }

  touchpad_input_ = std::make_shared<espp::TouchpadInput>(espp::TouchpadInput::Config{
      .touchpad_read =
          std::bind(&EspBox::touchpad_read, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4),
      .swap_xy = touch_swap_xy,
      .invert_x = touch_invert_x,
      .invert_y = touch_invert_y,
      .log_level = espp::Logger::Verbosity::WARN});

  return true;
}

bool EspBox::update_gt911() {
  // ensure the gt911 is initialized
  if (!gt911_) {
    return false;
  }
  // get the latest data from the device
  std::error_code ec;
  bool new_data = gt911_->update(ec);
  if (ec) {
    logger_.error("could not update gt911: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  // get the latest data from the touchpad
  TouchpadData temp_data;
  gt911_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = gt911_->get_home_button_state();
  // update the touchpad data
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

bool EspBox::update_tt21100() {
  // ensure the tt21100 is initialized
  if (!tt21100_) {
    return false;
  }
  // get the latest data from the device
  std::error_code ec;
  bool new_data = tt21100_->update(ec);
  if (ec) {
    logger_.error("could not update tt21100: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  // get the latest data from the touchpad
  TouchpadData temp_data;
  tt21100_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = tt21100_->get_home_button_state();
  // update the touchpad data
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

bool EspBox::update_touch() {
  switch (box_type_) {
  case BoxType::BOX3:
    return update_gt911();
  case BoxType::BOX:
    return update_tt21100();
  default:
    return false;
  }
}

std::shared_ptr<espp::TouchpadInput> EspBox::touchpad_input() const { return touchpad_input_; }

EspBox::TouchpadData EspBox::touchpad_data() const { return touchpad_data_; }

void EspBox::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                           uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

////////////////////////
// Display Functions //
////////////////////////

////////////////////////
// Audio Functions   //
////////////////////////

static TaskHandle_t play_audio_task_handle_ = NULL;

static bool IRAM_ATTR audio_tx_sent_callback(i2s_chan_handle_t handle, i2s_event_data_t *event,
                                             void *user_ctx) {
  // notify the main task that we're done
  vTaskNotifyGiveFromISR(play_audio_task_handle_, NULL);
  return true;
}

bool EspBox::initialize_sound(uint32_t default_audio_rate) {
  logger_.info("initializing i2s driver");
  logger_.debug("Using newer I2S standard");
  i2s_chan_config_t chan_cfg = {
      .id = i2s_port,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = 16,  // TODO: calculate form audio rate
      .dma_frame_num = 48, // TODO: calculate from audio rate
      .auto_clear = true,
      .intr_priority = 0,
  };

  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &audio_tx_handle, nullptr));

  audio_std_cfg = {
      .clk_cfg =
          {
              .sample_rate_hz = default_audio_rate,
              .clk_src = I2S_CLK_SRC_DEFAULT,
              .ext_clk_freq_hz = 0,
              .mclk_multiple = I2S_MCLK_MULTIPLE_256,
          },
      .slot_cfg =
          I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = i2s_mck_io,
              .bclk = i2s_bck_io,
              .ws = i2s_ws_io,
              .dout = i2s_do_io,
              .din = i2s_di_io,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };
  audio_std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

  ESP_ERROR_CHECK(i2s_channel_init_std_mode(audio_tx_handle, &audio_std_cfg));

  auto buffer_size = calc_audio_buffer_size(default_audio_rate);
  audio_tx_buffer.resize(buffer_size);

  audio_tx_stream = xStreamBufferCreate(buffer_size * 4, 0);

  memset(&audio_tx_callbacks_, 0, sizeof(audio_tx_callbacks_));
  audio_tx_callbacks_.on_sent = audio_tx_sent_callback;
  i2s_channel_register_event_callback(audio_tx_handle, &audio_tx_callbacks_, NULL);

  audio_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .name = "audio task",
      .callback = std::bind(&EspBox::audio_task_callback, this, std::placeholders::_1,
                            std::placeholders::_2),
      .stack_size_bytes = 1024 * 4,
      .priority = 19,
      .core_id = 1,
  });

  xStreamBufferReset(audio_tx_stream);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  audio_task_->start();

  return true;
}

bool IRAM_ATTR EspBox::audio_task_callback(std::mutex &m, std::condition_variable &cv) {
  // Queue the next I2S out frame to write
  uint16_t available = xStreamBufferBytesAvailable(audio_tx_stream);
  int buffer_size = audio_tx_buffer.size();
  available = std::min<uint16_t>(available, buffer_size);
  uint8_t *buffer = &audio_tx_buffer[0];
  memset(buffer, 0, buffer_size);

  if (available == 0) {
    i2s_channel_write(audio_tx_handle, buffer, buffer_size, NULL, portMAX_DELAY);
  } else {
    xStreamBufferReceive(audio_tx_stream, buffer, available, 0);
    i2s_channel_write(audio_tx_handle, buffer, available, NULL, portMAX_DELAY);
  }
  return false; // don't stop the task
}

void EspBox::update_volume_output() {
  if (mute_) {
    // TODO: use codec to set volume to 0
  } else {
    // TODO: use codec to set volume
  }
}

void EspBox::mute(bool mute) {
  mute_ = mute;
  update_volume_output();
}

bool EspBox::is_muted() const { return mute_; }

void EspBox::volume(float volume) {
  volume_ = volume;
  update_volume_output();
}

float EspBox::volume() const { return volume_; }

uint32_t EspBox::audio_sample_rate() const { return audio_std_cfg.clk_cfg.sample_rate_hz; }

void EspBox::audio_sample_rate(uint32_t sample_rate) {
  logger_.info("Setting audio sample rate to {} Hz", sample_rate);
  // stop the channel
  i2s_channel_disable(audio_tx_handle);
  // update the sample rate
  audio_std_cfg.clk_cfg.sample_rate_hz = sample_rate;
  i2s_channel_reconfig_std_clock(audio_tx_handle, &audio_std_cfg.clk_cfg);
  // clear the buffer
  xStreamBufferReset(audio_tx_stream);
  // restart the channel
  i2s_channel_enable(audio_tx_handle);
}

void EspBox::play_audio(const std::vector<uint8_t> &data) { play_audio(data.data(), data.size()); }

void IRAM_ATTR EspBox::play_audio(const uint8_t *data, uint32_t num_bytes) {
  play_audio_task_handle_ = xTaskGetCurrentTaskHandle();
  if (has_sound) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
  // don't block here
  xStreamBufferSendFromISR(audio_tx_stream, data, num_bytes, NULL);
  has_sound = true;
}
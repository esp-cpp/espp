#include "esp-box.hpp"

using namespace espp;

EspBox::EspBox()
    : BaseComponent("EspBox") {
  detect();
}

EspBox::BoxType EspBox::box_type() const { return box_type_; }

espp::I2c &EspBox::internal_i2c() { return internal_i2c_; }

espp::Interrupt &EspBox::interrupts() { return interrupts_; }

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
  } else if (found_tt21100) {
    box_type_ = BoxType::BOX;
  } else {
    logger_.warn("Could not detect box type, are you sure you're running on a box?");
    box_type_ = BoxType::UNKNOWN;
  }
  logger_.info("Detected box type: {}", box_type_);
  // now configure the pins based on the box type
  switch (box_type_) {
  case BoxType::BOX3:
    backlight_io = box3::backlight_io;
    reset_value = box3::reset_value;
    i2s_ws_io = box3::i2s_ws_io;
    touch_invert_x = box3::touch_invert_x;
    touch_interrupt_level = box3::touch_interrupt_level;
    touch_interrupt_type = box3::touch_interrupt_type;
    touch_interrupt_pullup_enabled = box3::touch_interrupt_pullup_enabled;
    break;
  case BoxType::BOX:
    backlight_io = box::backlight_io;
    reset_value = box::reset_value;
    i2s_ws_io = box::i2s_ws_io;
    touch_invert_x = box::touch_invert_x;
    touch_interrupt_level = box::touch_interrupt_level;
    touch_interrupt_type = box::touch_interrupt_type;
    touch_interrupt_pullup_enabled = box::touch_interrupt_pullup_enabled;
    break;
  default:
    break;
  }
  // now actually set the touch_interrupt_pin members:
  touch_interrupt_pin_.active_level = touch_interrupt_level;
  touch_interrupt_pin_.interrupt_type = touch_interrupt_type;
  touch_interrupt_pin_.pullup_enabled = touch_interrupt_pullup_enabled;
}

////////////////////////
// Touchpad Functions //
////////////////////////

bool EspBox::initialize_touch(const EspBox::touch_callback_t &callback) {
  if (touchpad_input_) {
    logger_.warn("Touchpad already initialized, not initializing again!");
    return false;
  }

  if (!display_) {
    logger_.warn("You should call initialize_display() before initialize_touch(), otherwise lvgl "
                 "will not properly handle the touchpad input!");
  }
  switch (box_type_) {
  case BoxType::BOX3:
    logger_.info("Initializing GT911");
    gt911_ = std::make_unique<espp::Gt911>(espp::Gt911::Config{
        .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                           std::placeholders::_2, std::placeholders::_3),
        .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                          std::placeholders::_2, std::placeholders::_3),
        .log_level = espp::Logger::Verbosity::WARN});
    break;
  case BoxType::BOX:
    logger_.info("Initializing TT21100");
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

  // store the callback
  touch_callback_ = callback;

  // add the touch interrupt pin
  interrupts_.add_interrupt(touch_interrupt_pin_);

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

EspBox::TouchpadData EspBox::touchpad_convert(const EspBox::TouchpadData &data) const {
  TouchpadData temp_data;
  temp_data.num_touch_points = data.num_touch_points;
  temp_data.x = data.x;
  temp_data.y = data.y;
  temp_data.btn_state = data.btn_state;
  if (temp_data.num_touch_points == 0) {
    return temp_data;
  }
  if (touch_swap_xy) {
    std::swap(temp_data.x, temp_data.y);
  }
  if (touch_invert_x) {
    temp_data.x = lcd_width_ - (temp_data.x + 1);
  }
  if (touch_invert_y) {
    temp_data.y = lcd_height_ - (temp_data.y + 1);
  }
  return temp_data;
}

////////////////////////
// Display Functions //
////////////////////////

// the user flag for the callbacks does two things:
// 1. Provides the GPIO level for the data/command pin, and
// 2. Sets some bits for other signaling (such as LVGL FLUSH)
static constexpr int FLUSH_BIT = (1 << (int)espp::display_drivers::Flags::FLUSH_BIT);
static constexpr int DC_LEVEL_BIT = (1 << (int)espp::display_drivers::Flags::DC_LEVEL_BIT);

// This function is called (in irq context!) just before a transmission starts.
// It will set the D/C line to the value indicated in the user field
// (DC_LEVEL_BIT).
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  static auto lcd_dc_io = EspBox::get_lcd_dc_gpio();
  uint32_t user_flags = (uint32_t)(t->user);
  bool dc_level = user_flags & DC_LEVEL_BIT;
  gpio_set_level(lcd_dc_io, dc_level);
}

// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  uint16_t user_flags = (uint32_t)(t->user);
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_display_t *disp = _lv_refr_get_disp_refreshing();
    lv_display_flush_ready(disp);
  }
}

bool EspBox::initialize_lcd() {
  if (lcd_handle_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }

  esp_err_t ret;

  memset(&lcd_spi_bus_config_, 0, sizeof(lcd_spi_bus_config_));
  lcd_spi_bus_config_.mosi_io_num = lcd_mosi_io;
  lcd_spi_bus_config_.miso_io_num = -1;
  lcd_spi_bus_config_.sclk_io_num = lcd_sclk_io;
  lcd_spi_bus_config_.quadwp_io_num = -1;
  lcd_spi_bus_config_.quadhd_io_num = -1;
  lcd_spi_bus_config_.max_transfer_sz = frame_buffer_size * sizeof(lv_color_t) + 100;

  memset(&lcd_config_, 0, sizeof(lcd_config_));
  lcd_config_.mode = 0;
  // lcd_config_.flags = SPI_DEVICE_NO_RETURN_RESULT;
  lcd_config_.clock_speed_hz = lcd_clock_speed;
  lcd_config_.input_delay_ns = 0;
  lcd_config_.spics_io_num = lcd_cs_io;
  lcd_config_.queue_size = spi_queue_size;
  lcd_config_.pre_cb = lcd_spi_pre_transfer_callback;
  lcd_config_.post_cb = lcd_spi_post_transfer_callback;

  // Initialize the SPI bus
  ret = spi_bus_initialize(lcd_spi_num, &lcd_spi_bus_config_, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  // Attach the LCD to the SPI bus
  ret = spi_bus_add_device(lcd_spi_num, &lcd_config_, &lcd_handle_);
  ESP_ERROR_CHECK(ret);
  // initialize the controller
  using namespace std::placeholders;
  DisplayDriver::initialize(espp::display_drivers::Config{
      .lcd_write = std::bind(&EspBox::write_lcd, this, _1, _2, _3),
      .lcd_send_lines = std::bind(&EspBox::write_lcd_lines, this, _1, _2, _3, _4, _5, _6),
      .reset_pin = lcd_reset_io,
      .data_command_pin = lcd_dc_io,
      .reset_value = reset_value,
      .invert_colors = invert_colors,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y});
  return true;
}

bool EspBox::initialize_display(size_t pixel_buffer_size) {
  if (!lcd_handle_) {
    logger_.error(
        "LCD not initialized, you must call initialize_lcd() before initialize_display()!");
    return false;
  }
  if (display_) {
    logger_.warn("Display already initialized, not initializing again!");
    return false;
  }
  // initialize the display / lvgl
  using namespace std::chrono_literals;
  display_ = std::make_shared<espp::Display<Pixel>>(espp::Display<Pixel>::AllocatingConfig{
      .width = lcd_width_,
      .height = lcd_height_,
      .pixel_buffer_size = pixel_buffer_size,
      .flush_callback = DisplayDriver::flush,
      .rotation_callback = DisplayDriver::rotate,
      .backlight_pin = backlight_io,
      .backlight_on_value = backlight_value,
      .task_config =
          {
              .name = "display task",
              .priority = 10,
              .core_id = 1,
          },
      .update_period = 5ms,
      .double_buffered = true,
      .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
      .rotation = rotation,
      .software_rotation_enabled = true,
  });

  frame_buffer0_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  frame_buffer1_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  return true;
}

std::shared_ptr<espp::Display<EspBox::Pixel>> EspBox::display() const { return display_; }

void IRAM_ATTR EspBox::lcd_wait_lines() {
  spi_transaction_t *rtrans;
  esp_err_t ret;
  // logger_.debug("Waiting for {} queued transactions", num_queued_trans);
  // Wait for all transactions to be done and get back the results.
  while (num_queued_trans) {
    ret = spi_device_get_trans_result(lcd_handle_, &rtrans, 10 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
      logger_.error("Display: Could not get spi trans result: {} '{}'", ret, esp_err_to_name(ret));
    }
    num_queued_trans--;
    // We could inspect rtrans now if we received any info back. The LCD is treated as write-only,
    // though.
  }
}

void IRAM_ATTR EspBox::write_lcd(const uint8_t *data, size_t length, uint32_t user_data) {
  if (length == 0) {
    return;
  }
  lcd_wait_lines();
  esp_err_t ret;
  memset(&trans[0], 0, sizeof(spi_transaction_t));
  trans[0].length = length * 8;
  trans[0].user = (void *)user_data;
  // look at the length of the data and use tx_data if it is <= 32 bits
  if (length <= 4) {
    // copy the data pointer to trans[0].tx_data
    memcpy(trans[0].tx_data, data, length);
    trans[0].flags = SPI_TRANS_USE_TXDATA;
  } else {
    trans[0].tx_buffer = data;
    trans[0].flags = 0;
  }
  ret = spi_device_queue_trans(lcd_handle_, &trans[0], 10 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    logger_.error("Couldn't queue spi trans for display: {} '{}'", ret, esp_err_to_name(ret));
  } else {
    num_queued_trans++;
  }
}

void IRAM_ATTR EspBox::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                       uint32_t user_data) {
  // if we haven't waited by now, wait here...
  lcd_wait_lines();
  esp_err_t ret;
  size_t length = (xe - xs + 1) * (ye - ys + 1) * 2;
  if (length == 0) {
    logger_.error("lcd_send_lines: Bad length: ({},{}) to ({},{})", xs, ys, xe, ye);
  }
  // initialize the spi transactions
  for (int i = 0; i < 6; i++) {
    memset(&trans[i], 0, sizeof(spi_transaction_t));
    if ((i & 1) == 0) {
      // Even transfers are commands
      trans[i].length = 8;
      trans[i].user = (void *)0;
    } else {
      // Odd transfers are data
      trans[i].length = 8 * 4;
      trans[i].user = (void *)DC_LEVEL_BIT;
    }
    trans[i].flags = SPI_TRANS_USE_TXDATA;
  }
  trans[0].tx_data[0] = (uint8_t)DisplayDriver::Command::caset;
  trans[1].tx_data[0] = (xs) >> 8;
  trans[1].tx_data[1] = (xs)&0xff;
  trans[1].tx_data[2] = (xe) >> 8;
  trans[1].tx_data[3] = (xe)&0xff;
  trans[2].tx_data[0] = (uint8_t)DisplayDriver::Command::raset;
  trans[3].tx_data[0] = (ys) >> 8;
  trans[3].tx_data[1] = (ys)&0xff;
  trans[3].tx_data[2] = (ye) >> 8;
  trans[3].tx_data[3] = (ye)&0xff;
  trans[4].tx_data[0] = (uint8_t)DisplayDriver::Command::ramwr;
  trans[5].tx_buffer = data;
  trans[5].length = length * 8;
  // undo SPI_TRANS_USE_TXDATA flag
  trans[5].flags = SPI_TRANS_DMA_BUFFER_ALIGN_MANUAL;
  // we need to keep the dc bit set, but also add our flags
  trans[5].user = (void *)(DC_LEVEL_BIT | user_data);
  // Queue all transactions.
  for (int i = 0; i < 6; i++) {
    ret = spi_device_queue_trans(lcd_handle_, &trans[i], 10 / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
      logger_.error("Couldn't queue spi trans for display: {} '{}'", ret, esp_err_to_name(ret));
    } else {
      num_queued_trans++;
    }
  }
  // When we are here, the SPI driver is busy (in the background) getting the
  // transactions sent. That happens mostly using DMA, so the CPU doesn't have
  // much to do here. We're not going to wait for the transaction to finish
  // because we may as well spend the time calculating the next line. When that
  // is done, we can call lcd_wait_lines, which will wait for the transfers
  // to be done and check their status.
}

void EspBox::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
                             const uint16_t height, uint8_t *data) {
  if (data) {
    // have data, fill the area with the color data
    lv_area_t area{.x1 = (lv_coord_t)(xs),
                   .y1 = (lv_coord_t)(ys),
                   .x2 = (lv_coord_t)(xs + width - 1),
                   .y2 = (lv_coord_t)(ys + height - 1)};
    DisplayDriver::fill(nullptr, &area, data);
  } else {
    // don't have data, so clear the area (set to 0)
    DisplayDriver::clear(xs, ys, width, height);
  }
}

EspBox::Pixel *EspBox::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

EspBox::Pixel *EspBox::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

uint8_t *EspBox::frame_buffer0() const { return frame_buffer0_; }

uint8_t *EspBox::frame_buffer1() const { return frame_buffer1_; }

void EspBox::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f) / 100.0f;
  // display expects a value between 0 and 1
  display_->set_brightness(brightness);
}

float EspBox::brightness() const {
  // display returns a value between 0 and 1
  return display_->get_brightness() * 100.0f;
}

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

bool EspBox::initialize_codec() {
  logger_.info("initializing codec");

  set_es8311_write(std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3));
  set_es8311_read(std::bind(&espp::I2c::read_at_register, &internal_i2c_, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));

  esp_err_t ret_val = ESP_OK;
  audio_hal_codec_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.codec_mode = AUDIO_HAL_CODEC_MODE_DECODE;
  cfg.dac_output = AUDIO_HAL_DAC_OUTPUT_LINE1;
  cfg.i2s_iface.bits = AUDIO_HAL_BIT_LENGTH_16BITS;
  cfg.i2s_iface.fmt = AUDIO_HAL_I2S_NORMAL;
  cfg.i2s_iface.mode = AUDIO_HAL_MODE_SLAVE;
  cfg.i2s_iface.samples = AUDIO_HAL_16K_SAMPLES;

  ret_val |= es8311_codec_init(&cfg);
  ret_val |= es8311_set_bits_per_sample(cfg.i2s_iface.bits);
  ret_val |= es8311_config_fmt((es_i2s_fmt_t)cfg.i2s_iface.fmt);
  ret_val |= es8311_codec_set_voice_volume(volume_);
  ret_val |= es8311_codec_ctrl_state(cfg.codec_mode, AUDIO_HAL_CTRL_START);

  if (ESP_OK != ret_val) {
    logger_.error("Codec initialization failed");
    return false;
  } else {
    logger_.info("Codec initialized");
    return true;
  }
}

bool EspBox::initialize_i2s(uint32_t default_audio_rate) {
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

  play_audio_task_handle_ = xTaskGetCurrentTaskHandle();

  memset(&audio_tx_callbacks_, 0, sizeof(audio_tx_callbacks_));
  audio_tx_callbacks_.on_sent = audio_tx_sent_callback;
  i2s_channel_register_event_callback(audio_tx_handle, &audio_tx_callbacks_, NULL);

  xStreamBufferReset(audio_tx_stream);

  ESP_ERROR_CHECK(i2s_channel_enable(audio_tx_handle));

  return true;
}

bool EspBox::initialize_sound(uint32_t default_audio_rate) {

  if (!initialize_i2s(default_audio_rate)) {
    logger_.error("Could not initialize I2S driver");
    return false;
  }
  if (!initialize_codec()) {
    logger_.error("Could not initialize codec");
    return false;
  }

  // Config power control IO
  gpio_set_direction(sound_power_pin, GPIO_MODE_OUTPUT);
  enable_sound(true);

  audio_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .name = "audio task",
      .callback = std::bind(&EspBox::audio_task_callback, this, std::placeholders::_1,
                            std::placeholders::_2),
      .stack_size_bytes = 1024 * 4,
      .priority = 19,
      .core_id = 1,
  });

  audio_task_->start();

  return true;
}

void EspBox::enable_sound(bool enable) { gpio_set_level(sound_power_pin, enable); }

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
    es8311_codec_set_voice_volume(0);
  } else {
    es8311_codec_set_voice_volume(volume_);
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

size_t EspBox::audio_buffer_size() const { return audio_tx_buffer.size(); }

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

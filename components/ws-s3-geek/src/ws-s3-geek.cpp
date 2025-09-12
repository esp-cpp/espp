#include "ws-s3-geek.hpp"

using namespace espp;

WsS3Geek::WsS3Geek()
    : BaseComponent("WsS3Geek") {}

espp::Interrupt &WsS3Geek::interrupts() { return interrupts_; }

///////////////////////
// Button Functions  //
///////////////////////

bool WsS3Geek::initialize_button(const WsS3Geek::button_callback_t &callback) {
  if (button_initialized_) {
    logger_.warn("Button already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing button");

  // save the callback
  button_callback_ = callback;

  // configure the button
  interrupts_.add_interrupt(button_interrupt_pin_);
  button_initialized_ = true;
  return true;
}

bool WsS3Geek::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
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
//
// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_pre_transfer_callback(spi_transaction_t *t) {
  static auto lcd_dc_io = WsS3Geek::get_lcd_dc_gpio();
  uint32_t user_flags = (uint32_t)(t->user);
  bool dc_level = user_flags & DC_LEVEL_BIT;
  gpio_set_level(lcd_dc_io, dc_level);
}

// This function is called (in irq context!) just after a transmission ends. It
// will indicate to lvgl that the next flush is ready to be done if the
// FLUSH_BIT is set.
//
// cppcheck-suppress constParameterCallback
static void IRAM_ATTR lcd_spi_post_transfer_callback(spi_transaction_t *t) {
  uint16_t user_flags = (uint32_t)(t->user);
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_display_t *disp = lv_disp_get_default();
    lv_display_flush_ready(disp);
  }
}

bool WsS3Geek::initialize_lcd() {
  if (lcd_handle_ || backlight_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing LCD...");

  // Initialize backlight PWM (moved out of Display)
  backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(backlight_io),
                                        .channel = LEDC_CHANNEL_0,
                                        .timer = LEDC_TIMER_0,
                                        .output_invert = !backlight_value});
  backlight_ = std::make_shared<Led>((Led::Config{.timer = LEDC_TIMER_0,
                                                  .frequency_hz = 5000,
                                                  .channels = backlight_channel_configs_,
                                                  .duty_resolution = LEDC_TIMER_10_BIT}));
  brightness(100.0f);

  esp_err_t ret;

  memset(&lcd_spi_bus_config_, 0, sizeof(lcd_spi_bus_config_));
  lcd_spi_bus_config_.mosi_io_num = lcd_mosi_io;
  lcd_spi_bus_config_.miso_io_num = -1;
  lcd_spi_bus_config_.sclk_io_num = lcd_sclk_io;
  lcd_spi_bus_config_.quadwp_io_num = -1;
  lcd_spi_bus_config_.quadhd_io_num = -1;
  lcd_spi_bus_config_.max_transfer_sz = SPI_MAX_TRANSFER_BYTES;

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
      .write_command = std::bind(&WsS3Geek::write_command, this, _1, _2, _3),
      .lcd_send_lines = std::bind(&WsS3Geek::write_lcd_lines, this, _1, _2, _3, _4, _5, _6),
      .reset_pin = lcd_reset_io,
      .data_command_pin = lcd_dc_io,
      .reset_value = reset_value,
      .invert_colors = invert_colors,
      .swap_color_order = swap_color_order,
      .offset_x = lcd_offset_x,
      .offset_y = lcd_offset_y,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y});
  return true;
}

bool WsS3Geek::initialize_display(size_t pixel_buffer_size) {
  if (!lcd_handle_) {
    logger_.error(
        "LCD not initialized, you must call initialize_lcd() before initialize_display()!");
    return false;
  }
  if (display_) {
    logger_.warn("Display already initialized, not initializing again!");
    return false;
  }
  logger_.info("Initializing display with pixel buffer size: {} bytes", pixel_buffer_size);
  // initialize the display / lvgl
  using namespace std::chrono_literals;
  display_ = std::make_shared<Display<Pixel>>(
      Display<Pixel>::LvglConfig{.width = lcd_width_,
                                 .height = lcd_height_,
                                 .flush_callback = DisplayDriver::flush,
                                 .rotation_callback = DisplayDriver::rotate,
                                 .rotation = rotation},
      Display<Pixel>::OledConfig{
          .set_brightness_callback =
              [this](float brightness) { this->brightness(brightness * 100.0f); },
          .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
      Display<Pixel>::DynamicMemoryConfig{
          .pixel_buffer_size = pixel_buffer_size,
          .double_buffered = true,
          .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
      });
  return true;
}

std::shared_ptr<espp::Display<WsS3Geek::Pixel>> WsS3Geek::display() const { return display_; }

void IRAM_ATTR WsS3Geek::lcd_wait_lines() {
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

void WsS3Geek::write_command(uint8_t command, std::span<const uint8_t> parameters,
                             uint32_t user_data) {
  lcd_wait_lines();
  memset(&trans[0], 0, sizeof(spi_transaction_t));
  memset(&trans[1], 0, sizeof(spi_transaction_t));

  trans[0].length = 8;
  trans[0].user = reinterpret_cast<void *>(user_data);
  trans[0].flags = SPI_TRANS_USE_TXDATA;
  trans[0].tx_data[0] = command;

  trans[1].length = parameters.size() * 8;
  if (parameters.size() <= 4) {
    // copy the data pointer to trans[1].tx_data
    memcpy(trans[1].tx_data, parameters.data(), parameters.size());
    trans[1].flags = SPI_TRANS_USE_TXDATA;
  } else if (!parameters.empty()) {
    trans[1].tx_buffer = parameters.data();
    trans[1].flags = 0;
  }
  trans[1].user = reinterpret_cast<void *>(
      user_data | (1 << static_cast<int>(display_drivers::Flags::DC_LEVEL_BIT)));

  esp_err_t ret = spi_device_queue_trans(lcd_handle_, &trans[0], 10 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    logger_.error("Couldn't queue spi command trans for display: {} '{}'", ret,
                  esp_err_to_name(ret));
  } else {
    ++num_queued_trans;
    if (!parameters.empty()) {
      ret = spi_device_queue_trans(lcd_handle_, &trans[1], 10 / portTICK_PERIOD_MS);
      if (ret != ESP_OK) {
        logger_.error("Couldn't queue spi data trans for display: {} '{}'", ret,
                      esp_err_to_name(ret));
      } else {
        ++num_queued_trans;
      }
    }
  }
}

void IRAM_ATTR WsS3Geek::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
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

void WsS3Geek::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
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

WsS3Geek::Pixel *WsS3Geek::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

WsS3Geek::Pixel *WsS3Geek::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

void WsS3Geek::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_)
    backlight_->set_duty(backlight_channel_configs_[0].channel, brightness);
}

float WsS3Geek::brightness() const {
  if (backlight_) {
    auto d = backlight_->get_duty(backlight_channel_configs_[0].channel);
    if (d.has_value())
      return d.value();
  }
  return 0.0f;
}

#include "t-dongle-s3.hpp"

using namespace espp;

TDongleS3::TDongleS3()
    : BaseComponent("TDongleS3") {}

///////////////////////
// RGB LED Functions //
///////////////////////

void TDongleS3::led_write(const uint8_t *data, size_t length) {
  static spi_transaction_t t;
  memset(&t, 0, sizeof(t));
  t.length = length * 8;
  t.tx_buffer = data;
  spi_device_polling_transmit(led_handle_, &t);
}

/// Initialize the RGB LED
/// \return true if the RGB LED was successfully initialized, false otherwise
bool TDongleS3::initialize_led() {
  if (led_) {
    logger_.warn("LED already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing LED with {} LEDs", num_leds_);

  esp_err_t ret;

  memset(&led_spi_bus_config_, 0, sizeof(led_spi_bus_config_));
  led_spi_bus_config_.mosi_io_num = led_mosi_io;
  led_spi_bus_config_.miso_io_num = -1;
  led_spi_bus_config_.sclk_io_num = led_sclk_io;
  led_spi_bus_config_.quadwp_io_num = -1;
  led_spi_bus_config_.quadhd_io_num = -1;
  led_spi_bus_config_.max_transfer_sz = 4 + 4 + num_leds_ * 4;

  memset(&led_config_, 0, sizeof(led_config_));
  led_config_.mode = 0;
  led_config_.clock_speed_hz = led_clock_speed;
  led_config_.input_delay_ns = 0;
  led_config_.spics_io_num = -1;
  led_config_.queue_size = 1;

  // Initialize the SPI bus
  ret = spi_bus_initialize(led_spi_num, &led_spi_bus_config_, SPI_DMA_CH_AUTO);
  ESP_ERROR_CHECK(ret);
  // Attach the LED to the SPI bus
  ret = spi_bus_add_device(led_spi_num, &led_config_, &led_handle_);
  ESP_ERROR_CHECK(ret);

  // initialize the led
  led_ = std::make_shared<espp::LedStrip>(espp::LedStrip::Config{
      .num_leds = num_leds_,
      .write = std::bind(&TDongleS3::led_write, this, std::placeholders::_1, std::placeholders::_2),
      .send_brightness = true,
      .byte_order = espp::LedStrip::ByteOrder::BGR,
      .start_frame = {0x00, 0x00, 0x00, 0x00}, // APA102 start frame
      .end_frame = {0xFF, 0xFF, 0xFF, 0xFF},   // APA102 end frame
      .log_level = espp::Logger::Verbosity::INFO});
  led_->set_all(espp::Rgb(0, 0, 0));
  led_->show();
  return true;
}

/// Get a shared pointer to the RGB LED
/// \return A shared pointer to the RGB LED
std::shared_ptr<LedStrip> TDongleS3::led() const { return led_; }

/// Set the color of the LED
/// \param hsv The color of the LED in HSV format
/// \param brightness The brightness of the LED as a percentage (0 - 100)
bool TDongleS3::led(const Hsv &hsv, float brightness) {
  if (led_) {
    brightness = std::clamp(brightness, 0.0f, 100.0f) / 100.0f;
    led_->set_pixel(0, hsv, brightness);
    led_->show();
    return true;
  }
  return false;
}

/// Set the color of the LED
/// \param rgb The color of the LED in RGB format
/// \param brightness The brightness of the LED as a percentage (0 - 100)
bool TDongleS3::led(const Rgb &rgb, float brightness) { return led(rgb.hsv(), brightness); }

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
  static auto lcd_dc_io = TDongleS3::get_lcd_dc_gpio();
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

bool TDongleS3::initialize_lcd() {
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
      .lcd_write = std::bind(&TDongleS3::write_lcd, this, _1, _2, _3),
      .lcd_send_lines = std::bind(&TDongleS3::write_lcd_lines, this, _1, _2, _3, _4, _5, _6),
      .reset_pin = lcd_reset_io,
      .data_command_pin = lcd_dc_io,
      .reset_value = reset_value,
      .invert_colors = invert_colors,
      .offset_x = lcd_offset_x,
      .offset_y = lcd_offset_y,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y});
  return true;
}

bool TDongleS3::initialize_display(size_t pixel_buffer_size, const espp::Task::BaseConfig &task_config, int update_period_ms) {
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
      // NOTE: for some reason, we have to swap the width and height here
      .width = lcd_height_,
      .height = lcd_width_,
      .pixel_buffer_size = pixel_buffer_size,
      .flush_callback = DisplayDriver::flush,
      .rotation_callback = DisplayDriver::rotate,
      .backlight_pin = backlight_io,
      .backlight_on_value = backlight_value,
      .task_config = task_config,
      .update_period = 1ms * update_period_ms,
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

std::shared_ptr<espp::Display<TDongleS3::Pixel>> TDongleS3::display() const { return display_; }

void IRAM_ATTR TDongleS3::lcd_wait_lines() {
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

void IRAM_ATTR TDongleS3::write_lcd(const uint8_t *data, size_t length, uint32_t user_data) {
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

void IRAM_ATTR TDongleS3::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
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

void TDongleS3::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
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

TDongleS3::Pixel *TDongleS3::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

TDongleS3::Pixel *TDongleS3::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

uint8_t *TDongleS3::frame_buffer0() const { return frame_buffer0_; }

uint8_t *TDongleS3::frame_buffer1() const { return frame_buffer1_; }

void TDongleS3::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f) / 100.0f;
  // display expects a value between 0 and 1
  display_->set_brightness(brightness);
}

float TDongleS3::brightness() const {
  // display returns a value between 0 and 1
  return display_->get_brightness() * 100.0f;
}

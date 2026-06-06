#include "t-dongle-s3.hpp"

#include <array>

using namespace espp;

TDongleS3::TDongleS3()
    : BaseComponent("TDongleS3") {}

espp::Interrupt &TDongleS3::interrupts() { return interrupts_; }

///////////////////////
// Button Functions  //
///////////////////////

bool TDongleS3::initialize_button(const TDongleS3::button_callback_t &callback) {
  logger_.info("Initializing button");

  // save the callback
  button_callback_ = callback;

  // configure the button
  interrupts_.add_interrupt(button_interrupt_pin_);
  button_initialized_ = true;
  return true;
}

bool TDongleS3::button_state() const {
  if (!button_initialized_) {
    return false;
  }
  return interrupts_.is_active(button_interrupt_pin_);
}

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
      .start_frame = std::array<uint8_t, 4>{0x00, 0x00, 0x00, 0x00}, // APA102 start frame
      .end_frame = std::array<uint8_t, 4>{0xFF, 0xFF, 0xFF, 0xFF},   // APA102 end frame
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

static void IRAM_ATTR lcd_spi_flush_ready(uint32_t) {
  lv_display_t *disp = lv_display_get_default();
  lv_display_flush_ready(disp);
}

bool TDongleS3::initialize_lcd() {
  if (lcd_ || backlight_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }
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

  lcd_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = lcd_spi_num,
      .sclk_io_num = lcd_sclk_io,
      .mosi_io_num = lcd_mosi_io,
      .miso_io_num = GPIO_NUM_NC,
      .max_transfer_sz = SPI_MAX_TRANSFER_BYTES,
      .log_level = get_log_level(),
  });
  lcd_ = std::make_unique<SpiPanelIo>(SpiPanelIo::Config{
      .spi = lcd_spi_.get(),
      .device_config =
          {
              .mode = 0,
              .clock_speed_hz = lcd_clock_speed,
              .input_delay_ns = 0,
              .cs_io_num = lcd_cs_io,
              .queue_size = spi_queue_size,
          },
      .data_command_io = lcd_dc_io,
      .data_command_bit_mask = DC_LEVEL_BIT,
      .post_transaction_callback_bit_mask = FLUSH_BIT,
      .post_transaction_callback = lcd_spi_flush_ready,
      .log_level = get_log_level(),
  });
  if (!lcd_->initialized()) {
    lcd_.reset();
    lcd_spi_.reset();
    return false;
  }
  display_driver_ = std::make_shared<DisplayDriver>(
      espp::display_drivers::Config{.panel_io = lcd_.get(),
                                    .write_command = nullptr,
                                    .read_command = nullptr,
                                    .lcd_send_lines = nullptr,
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
  if (!display_driver_ || !display_driver_->initialize()) {
    display_driver_.reset();
    lcd_.reset();
    lcd_spi_.reset();
    return false;
  }
  return true;
}

bool TDongleS3::initialize_display(size_t pixel_buffer_size) {
  if (!lcd_) {
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
  display_ = std::make_shared<Display<Pixel>>(
      // NOTE: for some reason, we have to swap the width and height here
      Display<Pixel>::LvglConfig{
          .width = lcd_height_,
          .height = lcd_width_,
          .flush_callback =
              [this](lv_display_t *disp, const lv_area_t *area, uint8_t *color_map) {
                if (display_driver_) {
                  display_driver_->flush(disp, area, color_map);
                }
              },
          .rotation_callback =
              [this](const DisplayRotation &new_rotation) {
                if (display_driver_) {
                  display_driver_->set_rotation(new_rotation);
                }
              },
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

  frame_buffer0_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  frame_buffer1_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  return true;
}

std::shared_ptr<espp::Display<TDongleS3::Pixel>> TDongleS3::display() const { return display_; }

void IRAM_ATTR TDongleS3::lcd_wait_lines() {
  if (lcd_) {
    lcd_->wait();
  }
}

void TDongleS3::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
                                const uint16_t height, uint8_t *data) {
  if (!display_driver_) {
    return;
  }
  if (data) {
    // have data, fill the area with the color data
    lv_area_t area{.x1 = (lv_coord_t)(xs),
                   .y1 = (lv_coord_t)(ys),
                   .x2 = (lv_coord_t)(xs + width - 1),
                   .y2 = (lv_coord_t)(ys + height - 1)};
    display_driver_->fill(nullptr, &area, data);
  } else {
    // don't have data, so clear the area (set to 0)
    display_driver_->clear(xs, ys, width, height);
  }
}

void IRAM_ATTR TDongleS3::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
                                          uint32_t user_data) {
  if (!lcd_) {
    return;
  }
  if (data == nullptr) {
    logger_.error("lcd_send_lines: Null data for ({},{}) to ({},{})", xs, ys, xe, ye);
    return;
  }
  if (xs < 0 || ys < 0 || xe < xs || ye < ys) {
    logger_.error("lcd_send_lines: Bad region: ({},{}) to ({},{})", xs, ys, xe, ye);
    return;
  }
  size_t width = static_cast<size_t>(xe - xs + 1);
  size_t height = static_cast<size_t>(ye - ys + 1);
  size_t length = width * height * lcd_bytes_per_pixel;
  lcd_->wait();
  std::array<uint8_t, 4> window = {
      static_cast<uint8_t>((xs >> 8) & 0xff),
      static_cast<uint8_t>(xs & 0xff),
      static_cast<uint8_t>((xe >> 8) & 0xff),
      static_cast<uint8_t>(xe & 0xff),
  };
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::caset));
  lcd_->queue_data(window);
  window = {
      static_cast<uint8_t>((ys >> 8) & 0xff),
      static_cast<uint8_t>(ys & 0xff),
      static_cast<uint8_t>((ye >> 8) & 0xff),
      static_cast<uint8_t>(ye & 0xff),
  };
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::raset));
  lcd_->queue_data(window);
  lcd_->queue_command(static_cast<uint8_t>(DisplayDriver::Command::ramwr));
  lcd_->queue_pixels(data, length, user_data);
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
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_)
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
}

float TDongleS3::brightness() const {
  if (backlight_) {
    auto d = backlight_->get_duty(LEDC_CHANNEL_0);
    if (d.has_value())
      return d.value();
  }
  return 0.0f;
}

size_t TDongleS3::rotated_display_width() const {
  auto *display = lv_display_get_default();
  auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
  switch (rotation) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_height_;
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_width_;
  }
}

size_t TDongleS3::rotated_display_height() const {
  auto *display = lv_display_get_default();
  auto rotation = display ? lv_display_get_rotation(display) : LV_DISPLAY_ROTATION_0;
  switch (rotation) {
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_width_;
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_height_;
  }
}

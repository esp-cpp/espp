#include "seeed-studio-round-display.hpp"

using namespace espp;

SsRoundDisplay::PinConfig SsRoundDisplay::pin_config_;

SsRoundDisplay::SsRoundDisplay()
    : BaseComponent("SsRoundDisplay")
    , internal_i2c_({.port = internal_i2c_port,
                     .sda_io_num = pin_config_.sda,
                     .scl_io_num = pin_config_.scl,
                     .sda_pullup_en = GPIO_PULLUP_ENABLE,
                     .scl_pullup_en = GPIO_PULLUP_ENABLE})
    , touch_interrupt_pin_({
          .gpio_num = pin_config_.touch_interrupt,
          .callback =
              std::bind(&SsRoundDisplay::touch_interrupt_handler, this, std::placeholders::_1),
          .active_level = touch_interrupt_level,
          .interrupt_type = espp::Interrupt::Type::FALLING_EDGE,
      }) {
  if (pin_config_ == PinConfig{}) {
    logger_.error("PinConfig not set, you must call set_pin_config() before initializing the "
                  "SsRoundDisplay! Hardware will not work properly!");
  }
}

espp::I2c &SsRoundDisplay::internal_i2c() { return internal_i2c_; }

espp::Interrupt &SsRoundDisplay::interrupts() { return interrupts_; }

////////////////////////
// Touchpad Functions //
////////////////////////

bool SsRoundDisplay::initialize_touch(const SsRoundDisplay::touch_callback_t &callback) {
  if (touch_) {
    logger_.warn("Touchpad already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing Touch Driver");
  touch_ = std::make_unique<TouchDriver>(TouchDriver::Config{
      .write = std::bind(&espp::I2c::write, &internal_i2c_, std::placeholders::_1,
                         std::placeholders::_2, std::placeholders::_3),
      .read = std::bind(&espp::I2c::read, &internal_i2c_, std::placeholders::_1,
                        std::placeholders::_2, std::placeholders::_3),
      .log_level = espp::Logger::Verbosity::WARN});

  // store the callback
  touch_callback_ = callback;

  // add the touch interrupt pin
  interrupts_.add_interrupt(touch_interrupt_pin_);

  return true;
}

void SsRoundDisplay::touch_interrupt_handler(const espp::Interrupt::Event &event) {
  if (update_touch()) {
    if (touch_callback_) {
      touch_callback_(touchpad_data());
    }
  }
}

bool SsRoundDisplay::update_touch() {
  // ensure the touch is initialized
  if (!touch_) {
    return false;
  }
  // get the latest data from the device
  std::error_code ec;
  bool new_data = touch_->update(ec);
  if (ec) {
    logger_.error("could not update touch driver: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  // get the latest data from the touchpad
  TouchpadData temp_data;
  touch_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  // update the touchpad data
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

std::shared_ptr<espp::TouchpadInput> SsRoundDisplay::touchpad_input() const {
  return touchpad_input_;
}

TouchpadData SsRoundDisplay::touchpad_data() const { return touchpad_data_; }

void SsRoundDisplay::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                                   uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

TouchpadData SsRoundDisplay::touchpad_convert(const TouchpadData &data) const {
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
  // get the orientation of the display
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  case LV_DISPLAY_ROTATION_0:
    break;
  case LV_DISPLAY_ROTATION_90:
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  case LV_DISPLAY_ROTATION_180:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    temp_data.y = lcd_height_ - (temp_data.y + 1);
    break;
  case LV_DISPLAY_ROTATION_270:
    temp_data.x = lcd_width_ - (temp_data.x + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  default:
    break;
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

static void lcd_spi_flush_ready(uint32_t) {
  lv_display_t *disp = lv_display_get_default();
  lv_display_flush_ready(disp);
}

bool SsRoundDisplay::initialize_lcd() {
  if (lcd_ || backlight_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }

  lcd_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = lcd_spi_num,
      .sclk_io_num = pin_config_.sck,
      .mosi_io_num = pin_config_.mosi,
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
              .cs_io_num = pin_config_.lcd_cs,
              .queue_size = 6,
          },
      .data_command_io = pin_config_.lcd_dc,
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
  display_driver_ = std::make_unique<DisplayDriver>(
      espp::display_drivers::Config{.panel_io = lcd_.get(),
                                    .write_command = nullptr,
                                    .read_command = nullptr,
                                    .lcd_send_lines = nullptr,
                                    .reset_pin = lcd_reset_io,
                                    .data_command_pin = pin_config_.lcd_dc,
                                    .reset_value = reset_value,
                                    .invert_colors = invert_colors,
                                    .swap_color_order = swap_color_order,
                                    .swap_xy = swap_xy,
                                    .mirror_x = mirror_x,
                                    .mirror_y = mirror_y});
  if (!display_driver_ || !display_driver_->initialize()) {
    display_driver_.reset();
    lcd_.reset();
    lcd_spi_.reset();
    return false;
  }
  // Initialize backlight PWM (moved out of Display)
  backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(pin_config_.lcd_backlight),
                                        .channel = LEDC_CHANNEL_0,
                                        .timer = LEDC_TIMER_0,
                                        .output_invert = !backlight_value});
  backlight_ =
      std::make_shared<espp::Led>((espp::Led::Config{.timer = LEDC_TIMER_0,
                                                     .frequency_hz = 5000,
                                                     .channels = backlight_channel_configs_,
                                                     .duty_resolution = LEDC_TIMER_10_BIT}));
  brightness(100.0f);
  return true;
}

bool SsRoundDisplay::initialize_display(size_t pixel_buffer_size) {
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
      Display<Pixel>::LvglConfig{
          .width = lcd_width_,
          .height = lcd_height_,
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

  touchpad_input_ = std::make_shared<espp::TouchpadInput>(espp::TouchpadInput::Config{
      .touchpad_read =
          std::bind(&SsRoundDisplay::touchpad_read, this, std::placeholders::_1,
                    std::placeholders::_2, std::placeholders::_3, std::placeholders::_4),
      .swap_xy = touch_swap_xy,
      .invert_x = touch_invert_x,
      .invert_y = touch_invert_y,
      .log_level = espp::Logger::Verbosity::WARN});

  frame_buffer0_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  frame_buffer1_ =
      (uint8_t *)heap_caps_malloc(frame_buffer_size, MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM);
  return true;
}

std::shared_ptr<espp::Display<SsRoundDisplay::Pixel>> SsRoundDisplay::display() const {
  return display_;
}

void SsRoundDisplay::lcd_wait_lines() {
  if (lcd_) {
    lcd_->wait();
  }
}

void SsRoundDisplay::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
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

SsRoundDisplay::Pixel *SsRoundDisplay::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

SsRoundDisplay::Pixel *SsRoundDisplay::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

uint8_t *SsRoundDisplay::frame_buffer0() const { return frame_buffer0_; }

uint8_t *SsRoundDisplay::frame_buffer1() const { return frame_buffer1_; }

void SsRoundDisplay::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_) {
    backlight_->set_duty(backlight_channel_configs_[0].channel, brightness);
  }
}

float SsRoundDisplay::brightness() const {
  if (backlight_) {
    auto maybe_duty = backlight_->get_duty(backlight_channel_configs_[0].channel);
    if (maybe_duty.has_value()) {
      return maybe_duty.value();
    }
  }
  return 0.0f; // if no backlight, return 0
}

size_t SsRoundDisplay::rotated_display_width() const {
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

size_t SsRoundDisplay::rotated_display_height() const {
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

#include "t-deck.hpp"

using namespace espp;

TDeck::TDeck()
    : BaseComponent("TDeck") {
  // initialize the pweripheral power pin and set it to high
  gpio_set_direction(peripheral_power_pin_, GPIO_MODE_OUTPUT);
  peripheral_power(true);
  // set all CS pins to be high to disable the devices
  gpio_set_direction(lcd_cs_io, GPIO_MODE_OUTPUT);
  gpio_set_level(lcd_cs_io, 1);
  gpio_set_direction(sdcard_cs, GPIO_MODE_OUTPUT);
  gpio_set_level(sdcard_cs, 1);
  gpio_set_direction(lora_cs_io, GPIO_MODE_OUTPUT);
  gpio_set_level(lora_cs_io, 1);
}

void TDeck::peripheral_power(bool on) { gpio_set_level(peripheral_power_pin_, on); }

bool TDeck::peripheral_power() const { return gpio_get_level(peripheral_power_pin_); }

////////////////////////
//   SPI Functions    //
////////////////////////

bool TDeck::init_spi_bus() {
  if (lcd_spi_) {
    return lcd_spi_->initialized();
  }

  lcd_spi_ = std::make_unique<Spi>(Spi::Config{
      .host = spi_num,
      .sclk_io_num = spi_sclk_io,
      .mosi_io_num = spi_mosi_io,
      .miso_io_num = spi_miso_io,
      .max_transfer_sz = SPI_MAX_TRANSFER_BYTES,
      .dma_channel = static_cast<spi_dma_chan_t>(SDSPI_DEFAULT_DMA),
      .log_level = get_log_level(),
  });
  if (!lcd_spi_->initialized()) {
    logger_.error("Failed to initialize bus.");
    lcd_spi_.reset();
    return false;
  }
  return true;
}

////////////////////////
// Keyboard Functions //
////////////////////////

bool TDeck::initialize_keyboard(bool start_task, const TDeck::keypress_callback_t &key_cb,
                                std::chrono::milliseconds poll_interval) {
  if (keyboard_) {
    logger_.warn("Keyboard already initialized, not initializing again!");
    return false;
  }
  logger_.info("Initializing keyboard input");
  std::error_code ec;
  keyboard_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = espp::TKeyboard::DEFAULT_ADDRESS,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!keyboard_i2c_device_) {
    logger_.error("Could not initialize keyboard I2C device: {}", ec.message());
    return false;
  }
  keyboard_ = std::make_shared<espp::TKeyboard>(
      espp::TKeyboard::Config{.write = espp::make_i2c_addressed_write(keyboard_i2c_device_),
                              .read = espp::make_i2c_addressed_read(keyboard_i2c_device_),
                              .key_cb = key_cb,
                              .polling_interval = poll_interval,
                              .auto_start = start_task,
                              .log_level = espp::Logger::Verbosity::WARN});
  return true;
}

std::shared_ptr<espp::TKeyboard> TDeck::keyboard() const { return keyboard_; }

/////////////////////////
// Trackball Functions //
/////////////////////////

bool TDeck::initialize_trackball(const TDeck::trackball_callback_t &trackball_cb, int sensitivity) {
  if (pointer_input_) {
    logger_.warn("Trackball already initialized, not initializing again!");
    return false;
  }
  logger_.info("Initializing trackball input");
  pointer_input_ = std::make_shared<espp::PointerInput>(espp::PointerInput::Config{
      .read = std::bind(&TDeck::trackball_read, this, std::placeholders::_1, std::placeholders::_2,
                        std::placeholders::_3, std::placeholders::_4),
      .log_level = espp::Logger::Verbosity::WARN});

  // store the callback
  trackball_callback_ = trackball_cb;

  // add the interrupts for the trackball
  interrupts_.add_interrupt(trackball_up_interrupt_pin);
  interrupts_.add_interrupt(trackball_down_interrupt_pin);
  interrupts_.add_interrupt(trackball_left_interrupt_pin);
  interrupts_.add_interrupt(trackball_right_interrupt_pin);
  interrupts_.add_interrupt(trackball_btn_interrupt_pin);

  // set the sensitivity
  set_trackball_sensitivity(sensitivity);

  return true;
}

std::shared_ptr<espp::PointerInput> TDeck::pointer_input() const { return pointer_input_; }

espp::PointerData TDeck::trackball_data() const { return trackball_data_; }

void TDeck::trackball_read(int &x, int &y, bool &left_pressed, bool &right_pressed) {
  std::lock_guard<std::recursive_mutex> lock(trackball_data_mutex_);
  x = trackball_data_.x;
  y = trackball_data_.y;
  left_pressed = trackball_data_.left_pressed;
  right_pressed = trackball_data_.right_pressed;
}

void TDeck::set_trackball_sensitivity(int sensitivity) { trackball_sensitivity_ = sensitivity; }

void TDeck::on_trackball_interrupt(const espp::Interrupt::Event &event) {
  int diff = trackball_sensitivity_;
  std::lock_guard lock(trackball_data_mutex_);
  if (event.gpio_num == trackball_up) {
    trackball_data_.y += diff;
  } else if (event.gpio_num == trackball_down) {
    trackball_data_.y -= diff;
  } else if (event.gpio_num == trackball_left) {
    trackball_data_.x -= diff;
  } else if (event.gpio_num == trackball_right) {
    trackball_data_.x += diff;
  } else if (event.gpio_num == trackball_btn) {
    trackball_data_.left_pressed = event.active;
  }
  trackball_data_.x = std::clamp<int>(trackball_data_.x, 0, lcd_width_ - 1);
  trackball_data_.y = std::clamp<int>(trackball_data_.y, 0, lcd_height_ - 1);
  if (trackball_callback_) {
    trackball_callback_(trackball_data_);
  }
}

////////////////////////
// Touchpad Functions //
////////////////////////

bool TDeck::initialize_touch(const TDeck::touch_callback_t &touch_cb) {
  if (gt911_) {
    logger_.warn("Touch already initialized, not initializing again!");
    return false;
  }

  logger_.info("Initializing touch input");
  std::error_code ec;
  touch_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = espp::Gt911::DEFAULT_ADDRESS_1,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!touch_i2c_device_) {
    logger_.error("Could not initialize touch I2C device: {}", ec.message());
    return false;
  }

  gt911_ = std::make_unique<espp::Gt911>(
      espp::Gt911::Config{.write = espp::make_i2c_addressed_write(touch_i2c_device_),
                          .read = espp::make_i2c_addressed_read(touch_i2c_device_),
                          .log_level = espp::Logger::Verbosity::WARN});

  // store the callback
  touch_callback_ = touch_cb;

  // add the touch interrupt pin
  interrupts_.add_interrupt(touch_interrupt_pin_);

  return true;
}

bool TDeck::update_gt911() {
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

std::shared_ptr<espp::TouchpadInput> TDeck::touchpad_input() const { return touchpad_input_; }

espp::TouchpadData TDeck::touchpad_data() const { return touchpad_data_; }

void TDeck::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y, uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

espp::TouchpadData TDeck::touchpad_convert(const espp::TouchpadData &data) const {
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

static void IRAM_ATTR lcd_spi_flush_ready(uint32_t user_flags) {
  bool should_flush = user_flags & FLUSH_BIT;
  if (should_flush) {
    lv_display_t *disp = lv_display_get_default();
    lv_display_flush_ready(disp);
  }
}

bool TDeck::initialize_lcd() {
  if (lcd_ || backlight_) {
    logger_.warn("LCD already initialized, not initializing again!");
    return false;
  }
  if (!init_spi_bus()) {
    logger_.error("Failed to initialize SPI bus for LCD.");
    return false;
  }

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
                                    .swap_xy = swap_xy,
                                    .mirror_x = mirror_x,
                                    .mirror_y = mirror_y,
                                    .mirror_portrait = mirror_portrait});
  if (!display_driver_ || !display_driver_->initialize()) {
    display_driver_.reset();
    lcd_.reset();
    return false;
  }

  backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(backlight_io),
                                        .channel = LEDC_CHANNEL_0,
                                        .timer = LEDC_TIMER_0,
                                        .output_invert = !backlight_value});
  backlight_ = std::make_shared<Led>((Led::Config{.timer = LEDC_TIMER_0,
                                                  .frequency_hz = 5000,
                                                  .channels = backlight_channel_configs_,
                                                  .duty_resolution = LEDC_TIMER_10_BIT}));
  brightness(100.0f);
  return true;
}

bool TDeck::initialize_display(size_t pixel_buffer_size) {
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
          std::bind(&TDeck::touchpad_read, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4),
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

std::shared_ptr<espp::Display<TDeck::Pixel>> TDeck::display() const { return display_; }

void IRAM_ATTR TDeck::lcd_wait_lines() {
  if (lcd_) {
    lcd_->wait();
  }
}

void IRAM_ATTR TDeck::write_command(uint8_t command, std::span<const uint8_t> parameters,
                                    uint32_t user_data) {
  if (lcd_) {
    lcd_->write_command(command, parameters, user_data);
  }
}

void IRAM_ATTR TDeck::write_lcd_lines(int xs, int ys, int xe, int ye, const uint8_t *data,
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
  size_t length = width * height * 2;
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

void TDeck::write_lcd_frame(const uint16_t xs, const uint16_t ys, const uint16_t width,
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

TDeck::Pixel *TDeck::vram0() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram0();
}

TDeck::Pixel *TDeck::vram1() const {
  if (!display_) {
    return nullptr;
  }
  return display_->vram1();
}

uint8_t *TDeck::frame_buffer0() const { return frame_buffer0_; }

uint8_t *TDeck::frame_buffer1() const { return frame_buffer1_; }

void TDeck::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_)
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
}

float TDeck::brightness() const {
  if (backlight_) {
    auto d = backlight_->get_duty(LEDC_CHANNEL_0);
    if (d.has_value())
      return d.value();
  }
  return 0.0f;
}

size_t TDeck::rotated_display_width() const {
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  // swap
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_height_;
  // as configured
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_width_;
  }
}

size_t TDeck::rotated_display_height() const {
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  // swap
  case LV_DISPLAY_ROTATION_90:
  case LV_DISPLAY_ROTATION_270:
    return lcd_width_;
  // as configured
  case LV_DISPLAY_ROTATION_0:
  case LV_DISPLAY_ROTATION_180:
  default:
    return lcd_height_;
  }
}

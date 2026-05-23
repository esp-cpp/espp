#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_touch(const touch_callback_t &callback) {
  if (touch_driver_ || st7123_touch_driver_) {
    logger_.warn("Touch driver already initialized");
    return true;
  }

  logger_.info("Initializing touch controller");

  touch_callback_ = callback;

  // add touch interrupt
  interrupts_.add_interrupt(touch_interrupt_pin_);

  // Determine which touch controller is present based on the detected display.
  // If the LCD has already been initialised, reuse the cached result; otherwise
  // run the I2C probe now.
  auto controller = display_controller_;
  if (controller == DisplayController::UNKNOWN) {
    controller = detect_display_controller();
  }

  using namespace std::chrono_literals;

  if (controller == DisplayController::ST7123) {
    // The ST7123 is a TDDI chip: its touch engine is enabled by LCD_RST, which
    // was already pulsed during initialize_lcd().  Toggling TP_RST here is
    // harmful — on some boards it takes the touch I2C endpoint offline.
    logger_.info("ST7123 variant detected — using integrated touch controller (skipping TP_RST)");

    st7123_touch_driver_ = std::make_shared<St7123TouchDriver>(St7123TouchDriver::Config{
        .write = std::bind_front(&I2c::write, &internal_i2c_),
        .read = std::bind_front(&I2c::read, &internal_i2c_),
        .address = St7123TouchDriver::DEFAULT_ADDRESS,
        .log_level = espp::Logger::Verbosity::WARN});
  } else {
    // ILI9881 (and UNKNOWN fallback) use a standalone GT911 touch controller
    // that requires a hardware reset via the IO expander before being used.
    logger_.info("ILI9881/default variant — using GT911 touch controller");

    touch_reset(true);
    std::this_thread::sleep_for(10ms);
    touch_reset(false);
    std::this_thread::sleep_for(50ms);

    touch_driver_ = std::make_shared<TouchDriver>(
        TouchDriver::Config{.write = std::bind_front(&I2c::write, &internal_i2c_),
                            .read = std::bind_front(&I2c::read, &internal_i2c_),
                            .address = TouchDriver::DEFAULT_ADDRESS_2, // GT911 0x14
                            .log_level = espp::Logger::Verbosity::WARN});
  }

  // Create touchpad input wrapper (identical for both drivers)
  touchpad_input_ = std::make_shared<TouchpadInput>(
      TouchpadInput::Config{.touchpad_read = std::bind_front(&M5StackTab5::touchpad_read, this),
                            .swap_xy = false,
                            .invert_x = false,
                            .invert_y = false,
                            .log_level = espp::Logger::Verbosity::WARN});

  logger_.info("Touch controller initialized successfully");
  return true;
}

bool M5StackTab5::update_touch() {
  logger_.debug("Updating touch data");

  std::error_code ec;
  TouchpadData temp_data;
  bool new_data = false;

  if (st7123_touch_driver_) {
    new_data = st7123_touch_driver_->update(ec);
    if (ec) {
      logger_.error("could not update ST7123 touch driver: {}", ec.message());
      std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
      touchpad_data_ = {};
      return false;
    }
    if (!new_data)
      return false;
    st7123_touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
    temp_data.btn_state = st7123_touch_driver_->get_home_button_state();
  } else if (touch_driver_) {
    new_data = touch_driver_->update(ec);
    if (ec) {
      logger_.error("could not update touch driver: {}", ec.message());
      std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
      touchpad_data_ = {};
      return false;
    }
    if (!new_data)
      return false;
    touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
    temp_data.btn_state = touch_driver_->get_home_button_state();
  } else {
    logger_.error("No touch driver initialized");
    return false;
  }

  // update the touchpad data
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

void M5StackTab5::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                                uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

M5StackTab5::TouchpadData
M5StackTab5::touchpad_convert(const M5StackTab5::TouchpadData &data) const {
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
    temp_data.x = display_width_ - (temp_data.x + 1);
  }
  if (touch_invert_y) {
    temp_data.y = display_height_ - (temp_data.y + 1);
  }
  // get the orientation of the display
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
  case LV_DISPLAY_ROTATION_0:
    break;
  case LV_DISPLAY_ROTATION_90:
    temp_data.y = display_height_ - (temp_data.y + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  case LV_DISPLAY_ROTATION_180:
    temp_data.x = display_width_ - (temp_data.x + 1);
    temp_data.y = display_height_ - (temp_data.y + 1);
    break;
  case LV_DISPLAY_ROTATION_270:
    temp_data.x = display_width_ - (temp_data.x + 1);
    std::swap(temp_data.x, temp_data.y);
    break;
  default:
    break;
  }
  return temp_data;
}

} // namespace espp

#include "esp-box.hpp"

using namespace espp;

////////////////////////
// Touchpad Functions //
////////////////////////

bool EspBox::initialize_touch(const EspBox::touch_callback_t &callback) {
  if (touch_driver_) {
    logger_.warn("Touch already initialized, not initializing again!");
    return false;
  }

  switch (box_type_) {
  case BoxType::BOX3: {
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
      logger_.error("Could not initialize GT911 I2C device: {}", ec.message());
      return false;
    }
    logger_.info("Initializing GT911");
    touch_driver_ = std::make_shared<espp::Gt911>(
        espp::Gt911::Config{.write = espp::make_i2c_addressed_write(touch_i2c_device_),
                            .read = espp::make_i2c_addressed_read(touch_i2c_device_),
                            .log_level = espp::Logger::Verbosity::WARN});
  } break;
  case BoxType::BOX: {
    std::error_code ec;
    touch_i2c_device_ = internal_i2c_.add_device<uint8_t>(
        {
            .device_address = espp::Tt21100::DEFAULT_ADDRESS,
            .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
            .scl_speed_hz = internal_i2c_.config().clk_speed,
            .log_level = espp::Logger::Verbosity::WARN,
        },
        ec);
    if (!touch_i2c_device_) {
      logger_.error("Could not initialize TT21100 I2C device: {}", ec.message());
      return false;
    }
    logger_.info("Initializing TT21100");
    touch_driver_ = std::make_shared<espp::Tt21100>(
        espp::Tt21100::Config{.read = espp::make_i2c_addressed_read(touch_i2c_device_),
                              .log_level = espp::Logger::Verbosity::WARN});
  } break;
  default:
    return false;
  }

  // store the callback
  touch_callback_ = callback;

  // add the touch interrupt pin
  interrupts_.add_interrupt(touch_interrupt_pin_);

  return true;
}

bool EspBox::update_touch() {
  if (!touch_driver_) {
    return false;
  }
  std::error_code ec;
  bool new_data = touch_driver_->update(ec);
  if (ec) {
    logger_.error("could not update touch driver: {}\n", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = touch_driver_->touchpad_data();
  return true;
}

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

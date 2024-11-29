#include "esp-box.hpp"

using namespace espp;

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

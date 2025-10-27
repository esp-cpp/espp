#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_touch(const touch_callback_t &callback) {
  if (touch_driver_) {
    logger_.warn("Touch driver already initialized");
    return true;
  }

  logger_.info("Initializing multi-touch controller");

  touch_callback_ = callback;

  // add touch interrupt
  interrupts_.add_interrupt(touch_interrupt_pin_);

  // Reset touch controller via expander if available
  touch_reset(true);
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(10ms);
  touch_reset(false);
  std::this_thread::sleep_for(50ms);

  // Create touch driver instance
  touch_driver_ = std::make_shared<TouchDriver>(
      TouchDriver::Config{.write = std::bind_front(&I2c::write, &internal_i2c_),
                          .read = std::bind_front(&I2c::read, &internal_i2c_),
                          .address = TouchDriver::DEFAULT_ADDRESS_2, // GT911 0x14 address
                          .log_level = espp::Logger::Verbosity::WARN});

  // Create touchpad input wrapper
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
  if (!touch_driver_) {
    logger_.error("Touch driver not initialized");
    return false;
  }

  // get the latest data from the device
  std::error_code ec;
  bool new_data = touch_driver_->update(ec);
  if (ec) {
    logger_.error("could not update touch_driver: {}", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  // get the latest data from the touchpad
  TouchpadData temp_data;
  touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = touch_driver_->get_home_button_state();
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

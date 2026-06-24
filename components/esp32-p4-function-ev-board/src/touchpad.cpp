#include "esp32-p4-function-ev-board.hpp"

using namespace std::chrono_literals;

namespace espp {

bool Esp32P4FunctionEvBoard::initialize_touch(const touch_callback_t &callback) {
  if (touch_driver_) {
    logger_.warn("Touch driver already initialized");
    return true;
  }

  logger_.info("Initializing GT911 multi-touch controller");
  touch_callback_ = callback;

  // The HMI subboard does not route the GT911 reset; the address is fixed at
  // power-on. Probe the primary (0x5D) then the backup (0x14) address.
  uint8_t address = gt911_default_address;
  if (!internal_i2c_.probe_device(gt911_default_address)) {
    if (internal_i2c_.probe_device(gt911_backup_address)) {
      address = gt911_backup_address;
    } else {
      logger_.warn("GT911 not found at 0x{:02X} or 0x{:02X}; continuing with 0x{:02X}",
                   gt911_default_address, gt911_backup_address, address);
    }
  }
  logger_.info("Using GT911 at address 0x{:02X}", address);

  std::error_code ec;
  touch_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = address,
          .timeout_ms = static_cast<int>(internal_i2c_.config().timeout_ms),
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!touch_i2c_device_) {
    logger_.error("Could not initialize touch I2C device: {}", ec.message());
    return false;
  }

  touch_driver_ = std::make_shared<TouchDriver>(
      TouchDriver::Config{.write = espp::make_i2c_addressed_write(touch_i2c_device_),
                          .read = espp::make_i2c_addressed_read(touch_i2c_device_),
                          .address = address,
                          .log_level = espp::Logger::Verbosity::WARN});

  touchpad_input_ = std::make_shared<TouchpadInput>(TouchpadInput::Config{
      .touchpad_read = std::bind_front(&Esp32P4FunctionEvBoard::touchpad_read, this),
      .swap_xy = touch_swap_xy,
      .invert_x = touch_invert_x,
      .invert_y = touch_invert_y,
      .log_level = espp::Logger::Verbosity::WARN});

  // The touch interrupt is not routed to the ESP32-P4 on this board, so poll the
  // GT911 in a task and invoke the user callback on new data.
  touch_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &m, std::condition_variable &cv) -> bool {
        if (update_touch()) {
          if (touch_callback_) {
            touch_callback_(touchpad_data());
          }
        }
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 16ms);
        return false; // don't stop
      },
      .task_config = {.name = "p4-ev touch",
                      .stack_size_bytes = CONFIG_ESP_P4_EV_BOARD_TOUCH_TASK_STACK_SIZE}});
  touch_task_->start();

  logger_.info("Touch controller initialized");
  return true;
}

bool Esp32P4FunctionEvBoard::update_touch() {
  if (!touch_driver_) {
    return false;
  }
  std::error_code ec;
  bool new_data = touch_driver_->update(ec);
  if (ec) {
    logger_.error("could not update touch driver: {}", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  TouchpadData temp_data;
  touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = touch_driver_->get_home_button_state();
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = temp_data;
  return true;
}

void Esp32P4FunctionEvBoard::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                                           uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *x = touchpad_data_.x;
  *y = touchpad_data_.y;
  *btn_state = touchpad_data_.btn_state;
}

Esp32P4FunctionEvBoard::TouchpadData
Esp32P4FunctionEvBoard::touchpad_convert(const TouchpadData &data) const {
  TouchpadData temp_data = data;
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
  // Map the (panel-native) touch point into the current LVGL display rotation so
  // it lines up with what is drawn on screen.
  auto rotation = lv_display_get_rotation(lv_display_get_default());
  switch (rotation) {
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
  case LV_DISPLAY_ROTATION_0:
  default:
    break;
  }
  return temp_data;
}

} // namespace espp

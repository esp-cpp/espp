#include "m5stack-tab5.hpp"

namespace espp {

bool M5StackTab5::initialize_touch(const touch_callback_t &callback) {
  if (touch_driver_) {
    logger_.warn("Touch driver already initialized");
    return true;
  }

  logger_.info("Initializing touch controller");

  touch_callback_ = callback;

  // Determine which touch controller is present based on the detected display.
  // If the LCD has already been initialised, reuse the cached result; otherwise
  // run the I2C probe now.
  auto controller = display_controller_;
  if (controller == DisplayController::UNKNOWN) {
    controller = detect_display_controller();
  }

  using namespace std::chrono_literals;

  std::error_code ec;
  touch_i2c_device_ = internal_i2c_.add_device<uint8_t>(
      {
          .device_address = controller == DisplayController::ST7123
                                ? St7123TouchDriver::DEFAULT_ADDRESS
                                : TouchDriver::DEFAULT_ADDRESS_2,
          // Keep the touch transaction timeout short. A touch read normally
          // completes in well under 1 ms; the bus default (200 ms, sized for the
          // IMU's large config write) would hold the shared internal I2C bus for
          // a fifth of a second whenever the ST7123 stalls, starving the IMU/RTC/
          // battery reads. Fail fast and let the next read retry instead.
          .timeout_ms = 20,
          .scl_speed_hz = internal_i2c_.config().clk_speed,
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!touch_i2c_device_) {
    logger_.error("Could not initialize touch I2C device: {}", ec.message());
    return false;
  }

  if (controller == DisplayController::ST7123) {
    // The pin we historically called "TP_RST" (IO expander 0x43 P5) is in fact
    // the ST7123 touch ENABLE line (esp-bsp's BSP_TOUCH_EN). The touch engine
    // needs a clean disable→enable (low→high) edge to boot its firmware and
    // start scanning; without it the chip answers I2C and reports correct
    // firmware/config registers but never produces coordinates. Pulse it here.
    logger_.info(
        "ST7123 variant detected — pulsing touch enable (0x43 P5) to start the touch engine");
    touch_reset(true); // drive 0x43 P5 low  (disable / reset touch)
    std::this_thread::sleep_for(20ms);
    touch_reset(false); // drive 0x43 P5 high (enable touch)
    std::this_thread::sleep_for(50ms);

    auto driver = std::make_shared<St7123TouchDriver>(St7123TouchDriver::Config{
        // Use two separate transactions (write the register pointer, then read)
        // rather than a combined repeated-START. The longer combined transaction
        // is more prone to I/O errors when reads run from the touch interrupt
        // handler on this board.
        .write = espp::make_i2c_addressed_write(touch_i2c_device_),
        .read = espp::make_i2c_addressed_read(touch_i2c_device_),
        .address = St7123TouchDriver::DEFAULT_ADDRESS,
        .log_level = espp::Logger::Verbosity::WARN});
    touch_driver_ = espp::make_touch_driver(std::move(driver));

    // Now that the touch engine is actually scanning (it needed the correct DPI
    // pixel clock to run), the ST7123 drives its TP_INT line low on each new
    // touch report — just like the GT911 — so read it from the touch interrupt
    // rather than polling. Reading only when a frame is ready avoids hammering
    // the shared internal I2C bus and the mid-scan I2C stalls that aggressive
    // polling provoked (which eventually wedged the touch engine).
    interrupts_.add_interrupt(touch_interrupt_pin_);
  } else {
    // ILI9881 (and UNKNOWN fallback) use a standalone GT911 touch controller
    // that requires a hardware reset via the IO expander before being used.
    logger_.info("ILI9881/default variant — using GT911 touch controller");

    touch_reset(true);
    std::this_thread::sleep_for(10ms);
    touch_reset(false);
    std::this_thread::sleep_for(50ms);

    auto driver = std::make_shared<TouchDriver>(
        TouchDriver::Config{.write = espp::make_i2c_addressed_write(touch_i2c_device_),
                            .read = espp::make_i2c_addressed_read(touch_i2c_device_),
                            .address = TouchDriver::DEFAULT_ADDRESS_2, // GT911 0x14
                            .log_level = espp::Logger::Verbosity::WARN});
    touch_driver_ = espp::make_touch_driver(std::move(driver));

    // The GT911 drives TP_INT, so read it from the touch interrupt.
    interrupts_.add_interrupt(touch_interrupt_pin_);
  }

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
    logger_.error("No touch driver initialized");
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
  logger_.debug("Touch driver update returned new_data={}", new_data);
  if (!new_data)
    return false;

  TouchpadData temp_data;
  touch_driver_->get_touch_point(&temp_data.num_touch_points, &temp_data.x, &temp_data.y);
  temp_data.btn_state = touch_driver_->get_home_button_state();

  logger_.debug("Touch data: num_touch_points={}, x={}, y={}, btn_state={}",
                temp_data.num_touch_points, temp_data.x, temp_data.y, temp_data.btn_state);

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

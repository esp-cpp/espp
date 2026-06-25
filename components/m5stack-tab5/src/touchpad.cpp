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
          // battery reads. Fail fast and let the next poll retry instead.
          .timeout_ms = 10,
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
        // The ST7123 only holds its register pointer within a single
        // transaction, so reads must be a repeated-START write-then-read.
        .write_then_read = espp::make_i2c_addressed_write_then_read(touch_i2c_device_),
        .address = St7123TouchDriver::DEFAULT_ADDRESS,
        .log_level = espp::Logger::Verbosity::WARN});
    touch_driver_ = espp::make_touch_driver(std::move(driver));

    // The ST7123's TP_INT line is not asserted like a standalone GT911's, so an
    // interrupt-driven read never fires. Poll the controller from a background
    // task instead (the same update()/get_touch_point() flow the standalone
    // st7123touch example uses).
    touch_poll_task_ = std::make_unique<espp::Task>(espp::Task::Config{
        .callback = [this](std::mutex &m, std::condition_variable &cv) -> bool {
          auto now = std::chrono::high_resolution_clock::now();
          if (update_touch() && touch_callback_) {
            touch_callback_(touchpad_data());
          }
          std::unique_lock<std::mutex> lock(m);
          // ~33 Hz is plenty for responsive touch and is gentle on the
          // shared internal I2C bus. Polling the TDDI ST7123 at 100 Hz while
          // it time-shares scanning with the display stresses it into I2C
          // stalls and eventually wedges its touch engine.
          cv.wait_until(lock, now + std::chrono::milliseconds(20));
          return false; // keep running
        },
        .task_config = {.name = "tab5 touch poll", .stack_size_bytes = 4 * 1024}});
    touch_poll_task_->start();
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

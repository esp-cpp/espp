#include "lilygo-t5-47.hpp"

#include <chrono>
#include <functional>
#include <thread>
#include <utility>

#include <driver/gpio.h>

using namespace std::chrono_literals;

namespace espp {

namespace {
// Reset the GT911 and strap it to the low I2C address (0x5D): hold RST low with
// INT low, then release RST while INT is low (INT is sampled on the RST rising
// edge; low -> 0x5D, high -> 0x14). Afterwards INT becomes the controller's
// data-ready output, which we leave as an input (we poll over I2C).
void gt911_reset(gpio_num_t rst_io, gpio_num_t int_io) {
  gpio_config_t out_cfg = {};
  out_cfg.pin_bit_mask = (1ULL << rst_io) | (1ULL << int_io);
  out_cfg.mode = GPIO_MODE_OUTPUT;
  gpio_config(&out_cfg);

  gpio_set_level(rst_io, 0);
  gpio_set_level(int_io, 0);
  std::this_thread::sleep_for(20ms); // hold in reset
  gpio_set_level(int_io, 0);         // INT low -> address 0x5D
  std::this_thread::sleep_for(1ms);
  gpio_set_level(rst_io, 1); // release reset; INT sampled low -> 0x5D
  // Keep INT low (as an output) while the controller latches its address and
  // boots. Releasing it too early leaves the GT911 half-initialised, where it
  // ACKs its address but NACKs register reads and can wedge the shared bus.
  std::this_thread::sleep_for(50ms);

  gpio_config_t in_cfg = {};
  in_cfg.pin_bit_mask = (1ULL << int_io);
  in_cfg.mode = GPIO_MODE_INPUT;
  in_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&in_cfg);
  std::this_thread::sleep_for(100ms); // let the controller finish booting
}
} // namespace

bool LilyGoT547::initialize_touch(const LilyGoT547::touch_callback_t &callback) {
  if (touch_initialized_) {
    logger_.warn("Touch already initialized");
    return true;
  }
  if (!internal_i2c_) {
    logger_.error("initialize_display() must be called before initialize_touch(): it creates the "
                  "shared I2C bus");
    return false;
  }
  logger_.info("Initializing GT911 touch on the internal I2C bus (RST={}, INT={})",
               static_cast<int>(touch_reset_io), static_cast<int>(touch_interrupt_io));

  // Bring the controller out of reset and strap it to address 0x5D.
  gt911_reset(touch_reset_io, touch_interrupt_io);

  std::error_code ec;
  touch_i2c_device_ = internal_i2c_->add_device<std::uint8_t>(
      {
          .device_address = espp::Gt911::DEFAULT_ADDRESS_1,
          .scl_speed_hz = 100 * 1000, // match epdiy / the board's 100 kHz bus
          .log_level = espp::Logger::Verbosity::WARN,
      },
      ec);
  if (!touch_i2c_device_) {
    logger_.error("Could not add GT911 I2C device: {}", ec.message());
    return false;
  }

  gt911_ = std::make_unique<espp::Gt911>(
      espp::Gt911::Config{.write = espp::make_i2c_addressed_write(touch_i2c_device_),
                          .read = espp::make_i2c_addressed_read(touch_i2c_device_),
                          .log_level = espp::Logger::Verbosity::WARN});

  touch_callback_ = callback;

  // Register the LVGL input device backed by our cached touch data. The
  // orientation (swap/invert) and display-rotation transform are applied in
  // touchpad_read(), so the input device itself does no swapping/inverting.
  touchpad_input_ = std::make_shared<espp::TouchpadInput>(espp::TouchpadInput::Config{
      .touchpad_read =
          std::bind(&LilyGoT547::touchpad_read, this, std::placeholders::_1, std::placeholders::_2,
                    std::placeholders::_3, std::placeholders::_4),
      .swap_xy = false,
      .invert_x = false,
      .invert_y = false,
      .log_level = espp::Logger::Verbosity::WARN});

  // Poll the GT911 for new data. The controller's config is query-mode (it does
  // not reliably signal via an edge on INT), so polling is the right approach
  // here - it matches the vendor firmware's isPressed()/getPoint() loop.
  touch_task_ = std::make_unique<espp::Task>(espp::Task::Config{
      .callback = [this](std::mutex &m, std::condition_variable &cv) -> bool {
        if (update_gt911() && touch_callback_) {
          touch_callback_(touchpad_data());
        }
        std::unique_lock<std::mutex> lock(m);
        cv.wait_for(lock, 15ms);
        return false; // keep running
      },
      .task_config = {.name = "gt911 poll", .stack_size_bytes = 4 * 1024, .priority = 6}});
  touch_task_->start();

  touch_initialized_ = true;
  return true;
}

bool LilyGoT547::update_gt911() {
  if (!gt911_) {
    return false;
  }
  std::error_code ec;
  bool new_data = gt911_->update(ec);
  if (ec) {
    logger_.error("Could not update GT911: {}", ec.message());
    std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
    touchpad_data_ = {};
    return false;
  }
  if (!new_data) {
    return false;
  }
  espp::TouchpadData data;
  gt911_->get_touch_point(&data.num_touch_points, &data.x, &data.y);
  data.btn_state = gt911_->get_home_button_state();
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  touchpad_data_ = data;
  return true;
}

espp::TouchpadData LilyGoT547::touchpad_data() const {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  return touchpad_data_;
}

void LilyGoT547::touchpad_read(uint8_t *num_touch_points, uint16_t *x, uint16_t *y,
                               uint8_t *btn_state) {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  *num_touch_points = touchpad_data_.num_touch_points;
  *btn_state = touchpad_data_.btn_state;

  // 1) Apply the panel's fixed orientation (swap/invert) to get the landscape
  //    coordinates, which for EPD_ROT_LANDSCAPE are also the physical coords.
  int lx = touchpad_data_.x;
  int ly = touchpad_data_.y;
  if (touch_swap_xy) {
    std::swap(lx, ly);
  }
  if (touch_invert_x) {
    lx = (panel_width - 1) - lx;
  }
  if (touch_invert_y) {
    ly = (panel_height - 1) - ly;
  }

  // 2) Map those physical coordinates into the current rotation's logical frame.
  //    This is the inverse of epdiy's coordinate rotation (see epd_draw_pixel),
  //    so touches line up with what LVGL draws at any rotation.
  switch (rotation_) {
  case EPD_ROT_LANDSCAPE:
    *x = lx;
    *y = ly;
    break;
  case EPD_ROT_PORTRAIT:
    *x = ly;
    *y = (panel_width - 1) - lx;
    break;
  case EPD_ROT_INVERTED_LANDSCAPE:
    *x = (panel_width - 1) - lx;
    *y = (panel_height - 1) - ly;
    break;
  case EPD_ROT_INVERTED_PORTRAIT:
    *x = (panel_height - 1) - ly;
    *y = lx;
    break;
  }
}

bool LilyGoT547::home_button_pressed() const {
  std::lock_guard<std::recursive_mutex> lock(touchpad_data_mutex_);
  return touchpad_data_.btn_state != 0;
}

} // namespace espp

#pragma once

#include <algorithm>
#include <functional>

#include "lvgl.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/**
 *  @brief Light wrapper around LVGL input device driver, specifically
 *         designed for touchpads with optional home buttons.
 */
class TouchpadInput : public BaseComponent {
public:
  /**
   * @brief Function prototype for getting the latest input data from the
   *        touchpad.
   * @param num_touches Number of touch points / presses (pointer to data to
   *                    be filled).
   * @param x Current x position (pointer to data to be filled).
   * @param y Current y position (pointer to data to be filled).
   * @param state Home button state if there is a home button (pointer to data
   *              to be filled).
   */
  typedef std::function<void(uint8_t *num_touches, uint16_t *x, uint16_t *y, uint8_t *state)>
      touchpad_read_fn;

  /**
   *  @brief Configuration structure, containing the read function for the
   *         touchpad itself.
   */
  struct Config {
    touchpad_read_fn touchpad_read; /**< Input function for the touchpad itself. */
    bool swap_xy{
        false}; /**< If true, swap the x/y coordinates retrieved from the touchpad_read_fn. */
    bool invert_x{false}; /**< If true, Invert the output of the x coordinate. */
    bool invert_y{false}; /**< If true, Invert the output of the y coordinate. */
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Log verbosity for the input driver.  */
  };

  /**
   * @brief Initialize and register the input drivers associated with the
   *        touchpad.
   * @param config Configuration structure for the TouchpadInput.
   */
  explicit TouchpadInput(const Config &config)
      : BaseComponent("TouchpadInput", config.log_level)
      , touchpad_read_(config.touchpad_read)
      , swap_xy_(config.swap_xy)
      , invert_x_(config.invert_x)
      , invert_y_(config.invert_y) {
    init();
  }

  /**
   * @brief Unregister the input drivers associated with the Touchpad.
   */
  ~TouchpadInput() {
    if (indev_touchpad_) {
      lv_indev_delete(indev_touchpad_);
    }
    if (indev_button_) {
      lv_indev_delete(indev_button_);
    }
  }

  /**
   * @brief Get a pointer to the LVGL input device driver for the touchpad.
   * @return Pointer to the LVGL input device driver for the touchpad.
   */
  lv_indev_t *get_touchpad_input_device() { return indev_touchpad_; }

  /**
   * @brief Get a pointer to the LVGL input device driver for the home button.
   * @return Pointer to the LVGL input device driver for the home button.
   */
  lv_indev_t *get_home_button_input_device() { return indev_button_; }

protected:
  static void touchpad_read(lv_indev_t *drv, lv_indev_data_t *data) {
    TouchpadInput *tpi = (TouchpadInput *)lv_indev_get_user_data(drv);
    if (tpi) {
      tpi->touchpad_read_impl(data);
    }
  }

  void touchpad_read_impl(lv_indev_data_t *data) {
    uint8_t num_touch_points, home_button_state;
    uint16_t x, y;
    if (!touchpad_read_) {
      logger_.error("Invalid touchpad_read function!");
      return;
    }
    touchpad_read_(&num_touch_points, &x, &y, &home_button_state);
    if (home_button_state) {
      home_button_pressed_ = true;
    }
    data->state = (num_touch_points > 0) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    if (num_touch_points > 0) {
      if (swap_xy_) {
        std::swap(x, y);
      }
      if (invert_x_) {
        x = screen_size_x_ - (x + 1);
      }
      if (invert_y_) {
        y = screen_size_y_ - (y + 1);
      }
      data->point.x = x;
      data->point.y = y;
    }
  }

  static void home_button_read(lv_indev_t *drv, lv_indev_data_t *data) {
    const TouchpadInput *tpi = (const TouchpadInput *)lv_indev_get_user_data(drv);
    if (tpi) {
      tpi->home_button_read_impl(data);
    }
  }

  void home_button_read_impl(lv_indev_data_t *data) const {
    data->state = home_button_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  }

  void init() {
    using namespace std::placeholders;
    logger_.info("Add TP input device to LVGL");
    indev_touchpad_ = lv_indev_create();
    if (!indev_touchpad_) {
      logger_.error("Failed to register touchpad input device!");
      return;
    }
    lv_indev_set_type(indev_touchpad_, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_touchpad_, &TouchpadInput::touchpad_read);
    lv_indev_set_user_data(indev_touchpad_, (void *)this);

    logger_.info("Add HOME button input to LVGL");
    indev_button_ = lv_indev_create();
    if (!indev_button_) {
      logger_.error("Failed to register home button input device!");
      return;
    }
    lv_indev_set_type(indev_button_, LV_INDEV_TYPE_BUTTON);
    lv_indev_set_read_cb(indev_button_, &TouchpadInput::home_button_read);
    lv_indev_set_user_data(indev_button_, (void *)this);

    auto disp = lv_display_get_default();
    screen_size_x_ = (uint16_t)lv_display_get_horizontal_resolution(disp);
    screen_size_y_ = (uint16_t)lv_display_get_vertical_resolution(disp);
  }

  touchpad_read_fn touchpad_read_;
  uint16_t screen_size_x_;
  uint16_t screen_size_y_;
  std::atomic<bool> swap_xy_{false};
  std::atomic<bool> invert_x_{false};
  std::atomic<bool> invert_y_{false};
  std::atomic<bool> home_button_pressed_{false};
  lv_indev_t *indev_touchpad_;
  lv_indev_t *indev_button_;
};
} // namespace espp

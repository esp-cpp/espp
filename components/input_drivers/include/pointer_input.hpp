#pragma once

#include <algorithm>
#include <functional>

#include "lvgl.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/**
 *  @brief Light wrapper around LVGL input device driver, specifically
 *         designed for pointers / cursors / mice input devices.
 */
class PointerInput : public BaseComponent {
public:
  /**
   * @brief Function prototype for getting the latest input data from the
   *        pointer.
   * @param num_touches Number of touch points / presses (pointer to data to
   *                    be filled).
   * @param x Current x position (pointer to data to be filled).
   * @param y Current y position (pointer to data to be filled).
   * @param state Home button state if there is a home button (pointer to data
   *              to be filled).
   */
  typedef std::function<void(int &x, int &y, bool &left_pressed, bool &right_pressed)>
      read_fn;

  /**
   *  @brief Configuration structure, containing the read function for the
   *         pointer itself.
   */
  struct Config {
    read_fn read; /**< Input function for the pointer hardware itself. */
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Log verbosity for the input driver.  */
  };

  /**
   * @brief Initialize and register the input drivers associated with the
   *        pointer.
   * @param config Configuration structure for the PointerInput.
   */
  explicit PointerInput(const Config &config)
      : BaseComponent("PointerInput", config.log_level)
      , read_(config.read) {
    init();
  }

  /**
   * @brief Unregister the input drivers associated with the Pointer.
   */
  ~PointerInput() {
    if (indev_pointer_) {
      lv_indev_delete(indev_pointer_);
    }
  }

  /**
   * @brief Get a pointer to the LVGL input device driver for the pointer.
   * @return Pointer to the LVGL input device driver for the pointer.
   */
  lv_indev_t *get_pointer_input_device() { return indev_pointer_; }

protected:
  static void read(lv_indev_t *drv, lv_indev_data_t *data) {
    PointerInput *pi = (PointerInput *)lv_indev_get_user_data(drv);
    if (pi) {
      pi->read_impl(data);
    }
  }

  void read_impl(lv_indev_data_t *data) {
    int x = 0;
    int y = 0;
    bool left_pressed = false;
    bool right_pressed = false;
    if (!read_) {
      logger_.error("Invalid read function!");
      return;
    }
    read_(x, y, left_pressed, right_pressed);
    last_x_ += x;
    last_y_ += y;
    data->point.x = std::clamp(last_x_, 0, LV_HOR_RES_MAX);
    data->point.y = std::clamp(last_y_, 0, LV_VER_RES_MAX);
    data->state = left_pressed ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
  }

  void init() {
    logger_.info("Add pointer input device to LVGL");
    indev_pointer_ = lv_indev_create();
    if (!indev_pointer_) {
      logger_.error("Failed to register pointer input device!");
      return;
    }
    lv_indev_set_type(indev_pointer_, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev_pointer_, &PointerInput::read);
    lv_indev_set_user_data(indev_pointer_, (void *)this);
  }

  pointer_read_fn read_;
  int last_x_{0};
  int last_y_{0};
  lv_indev_t *indev_pointer_{nullptr};
};
} // namespace espp

#pragma once

#include <algorithm>
#include <functional>

#include "lvgl.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/**
 *  @brief Light wrapper around LVGL input device driver, specifically
 *         designed for keypads.
 */
class KeypadInput : public BaseComponent {
public:
  typedef std::function<void(bool *up, bool *down, bool *left, bool *right, bool *enter,
                             bool *escape)>
      read_fn;

  /**
   *  @brief Configuration structure, containing the read function for the
   *         keypad itself.
   */
  struct Config {
    read_fn read; /**< Input function for the keypad. */
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Log verbosity for the input driver.  */
  };

  /**
   * @brief Initialize and register the input drivers associated with the
   *        keypad.
   * @param config Configuration structure for the KeypadInput.
   */
  explicit KeypadInput(const Config &config)
      : BaseComponent("KeypadInput", config.log_level)
      , read_(config.read) {
    init();
  }

  /**
   * @brief Unregister the input drivers associated with the Keypad.
   */
  ~KeypadInput() {
    if (indev_keypad_) {
      lv_indev_delete(indev_keypad_);
    }
  }

  /**
   * @brief Get the input device driver associated with the keypad.
   * @return The input device driver associated with the keypad.
   */
  lv_indev_t *get_input_device() { return indev_keypad_; }

protected:
  static void keypad_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    KeypadInput *ki = (KeypadInput *)drv->user_data;
    if (ki) {
      ki->keypad_read_impl(data);
    }
  }

  void keypad_read_impl(lv_indev_data_t *data) {
    if (!read_) {
      logger_.error("Invalid read function!");
      return;
    }
    logger_.info("Reading keypad...");
    bool up, down, left, right, enter, escape;
    read_(&up, &down, &left, &right, &enter, &escape);
    data->state = LV_INDEV_STATE_PRESSED;
    if (escape) {
      data->key = LV_KEY_ESC;
    } else if (enter) {
      data->key = LV_KEY_ENTER;
    } else if (up) {
      data->key = LV_KEY_UP;
    } else if (down) {
      data->key = LV_KEY_DOWN;
    } else if (left) {
      data->key = LV_KEY_LEFT;
    } else if (right) {
      data->key = LV_KEY_RIGHT;
    } else {
      data->state = LV_INDEV_STATE_RELEASED;
    }
    logger_.debug("Keypad state: up: {}, down: {}, left: {}, right: {}, enter: {}, escape: {}", up,
                  down, left, right, enter, escape);
    logger_.debug("Keypad data: state: {}, key: {}", (int)data->state, (int)data->key);
  }

  void init() {
    using namespace std::placeholders;
    logger_.info("Add keypad input device to LVGL");
    lv_indev_drv_init(&indev_drv_keypad_);
    indev_drv_keypad_.type = LV_INDEV_TYPE_KEYPAD;
    indev_drv_keypad_.read_cb = &KeypadInput::keypad_read;
    indev_drv_keypad_.user_data = (void *)this;
    indev_keypad_ = lv_indev_drv_register(&indev_drv_keypad_);
    if (!indev_keypad_) {
      logger_.error("Failed to register keypad input device!");
    }
  }

  read_fn read_;
  lv_indev_drv_t indev_drv_keypad_;
  lv_indev_t *indev_keypad_;
};
} // namespace espp

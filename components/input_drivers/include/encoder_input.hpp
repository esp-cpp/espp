#pragma once

#include <algorithm>
#include <functional>

#include "lvgl.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/**
 *  @brief Light wrapper around LVGL input device driver, specifically
 *         designed for encoders with optional home buttons.
 */
class EncoderInput : public BaseComponent {
public:
  typedef std::function<void(int *enc_diff, bool button_pressed)> read_fn;

  /**
   *  @brief Configuration structure, containing the read function for the
   *         encoder itself.
   */
  struct Config {
    read_fn read; /**< Input function for the encoder and button itself. */
    Logger::Verbosity log_level{
        Logger::Verbosity::WARN}; /**< Log verbosity for the input driver.  */
  };

  /**
   * @brief Initialize and register the input drivers associated with the
   *        encoder.
   * @param config Configuration structure for the EncoderInput.
   */
  explicit EncoderInput(const Config &config)
      : BaseComponent("EncoderInput", config.log_level)
      , read_(config.read) {
    init();
  }

  /**
   * @brief Unregister the input drivers associated with the Encoder.
   */
  ~EncoderInput() {
    if (indev_encoder_) {
      lv_indev_delete(indev_encoder_);
    }
    if (indev_button_) {
      lv_indev_delete(indev_button_);
    }
  }

  /**
   * @brief Get the input device driver associated with the encoder.
   * @return The input device driver associated with the encoder.
   */
  lv_indev_t *get_encoder_input_device() { return indev_encoder_; }

  /**
   * @brief Get the input device driver associated with the button.
   * @return The input device driver associated with the button.
   */
  lv_indev_t *get_button_input_device() { return indev_button_; }

protected:
  static void encoder_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    EncoderInput *ei = (EncoderInput *)drv->user_data;
    if (ei) {
      ei->encoder_read_impl(data);
    }
  }

  void encoder_read_impl(lv_indev_data_t *data) {
    int enc_diff;
    bool button_pressed;
    if (!read_) {
      logger_.error("Invalid read function!");
      return;
    }
    read_(&enc_diff, &button_pressed);
    if (button_pressed) {
      button_pressed_ = true;
    }
    data->state = (button_pressed) ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
    data->enc_diff = enc_diff;
  }

  static void button_read(lv_indev_drv_t *drv, lv_indev_data_t *data) {
    EncoderInput *ei = (EncoderInput *)drv->user_data;
    if (ei) {
      ei->home_button_read_impl(data);
    }
  }

  void button_read_impl(lv_indev_data_t *data) const {
    data->state = button_pressed_ ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
  }

  void init() {
    using namespace std::placeholders;
    logger_.info("Add encoder input device to LVGL");
    lv_indev_drv_init(&indev_drv_enc_);
    indev_drv_enc_.type = LV_INDEV_TYPE_POINTER;
    indev_drv_enc_.read_cb = &EncoderInput::encoder_read;
    indev_drv_enc_.user_data = (void *)this;
    indev_encoder_ = lv_indev_drv_register(&indev_drv_enc_);
    if (!indev_encoder_) {
      logger_.error("Failed to register encoder input device!");
      return;
    }

    logger_.info("Add button input to LVGL");
    lv_indev_drv_init(&indev_drv_btn_);
    indev_drv_btn_.type = LV_INDEV_TYPE_BUTTON;
    indev_drv_btn_.read_cb = &EncoderInput::button_read;
    indev_drv_btn_.user_data = (void *)this;
    indev_button_ = lv_indev_drv_register(&indev_drv_btn_);
    if (!indev_button_) {
      logger_.error("Failed to register button input device!");
      return;
    }
  }

  encoder_read_fn encoder_read_;
  std::atomic<bool> button_pressed_{false};
  lv_indev_drv_t indev_drv_enc_;
  lv_indev_t *indev_encoder_;
  lv_indev_drv_t indev_drv_btn_;
  lv_indev_t *indev_button_;
};
} // namespace espp

#pragma once

#include <algorithm>
#include <functional>

#include "lvgl.h"
#include "sdkconfig.h"

#include "base_component.hpp"

namespace espp {
/// The data structure for the touchpad
struct PointerData {
  int x = 0;                  ///< The x coordinate
  int y = 0;                  ///< The y coordinate
  bool left_pressed = false;  ///< The left button pressed state
  bool right_pressed = false; ///< The right button pressed state

  /// @brief Compare two PointerData objects for equality
  /// @param rhs The right hand side of the comparison
  /// @return true if the two PointerData objects are equal, false otherwise
  bool operator==(const PointerData &rhs) const = default;
};

/**
 *  @brief Light wrapper around LVGL input device driver, specifically
 *         designed for pointers / cursors / mice input devices.
 */
class PointerInput : public BaseComponent {
public:
  /**
   * @brief Function prototype for getting the latest input data from the
   *        pointer.
   * @param[out] x Current x position.
   * @param[out] y Current y position.
   * @param[out] left_pressed Whether the left button is pressed.
   * @param[out] right_pressed Whether the right button is pressed.
   */
  typedef std::function<void(int &x, int &y, bool &left_pressed, bool &right_pressed)> read_fn;

  /**
   *  @brief Configuration structure, containing the read function for the
   *         pointer itself.
   */
  struct Config {
    read_fn read;         /**< Input function for the pointer hardware itself. */
    int cursor_radius{8}; /**< Radius of the cursor object. */
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
      , read_(config.read)
      , cursor_radius_(config.cursor_radius) {
    init();
  }

  /**
   * @brief Unregister the input drivers associated with the Pointer.
   */
  ~PointerInput() {
    if (indev_pointer_) {
      lv_indev_delete(indev_pointer_);
    }
    if (cursor_obj_) {
      lv_obj_del(cursor_obj_);
    }
  }

  /**
   * @brief Get a pointer to the LVGL input device driver for the pointer.
   * @return Pointer to the LVGL input device driver for the pointer.
   */
  lv_indev_t *get_pointer_input_device() { return indev_pointer_; }

  /**
   * @brief Get a pointer to the cursor object.
   * @return Pointer to the cursor object.
   */
  lv_obj_t *get_cursor_object() { return cursor_obj_; }

  /**
   * @brief Set the cursor to a specific icon.
   * @param icon The icon to set.
   * @note The icon must be a valid LVGL image descriptor.
   */
  void set_cursor(const lv_image_dsc_t *icon) {
    if (!indev_pointer_) {
      logger_.error("Invalid input device!");
      return;
    }
    if (!icon) {
      logger_.error("Invalid icon!");
      return;
    }
    if (cursor_obj_) {
      lv_obj_del(cursor_obj_);
    }
    cursor_obj_ = lv_img_create(lv_scr_act());
    lv_img_set_src(cursor_obj_, icon);
    lv_indev_set_cursor(indev_pointer_, cursor_obj_);
    // get the size of the icon
    lv_coord_t w = lv_obj_get_width(cursor_obj_);
    lv_coord_t h = lv_obj_get_height(cursor_obj_);
    cursor_radius_ = std::max(w, h) / 2;
  }

  /**
   * @brief Set the cursor to a specific object.
   * @param cursor_obj The cursor object to set.
   */
  void set_cursor(lv_obj_t *cursor_obj) {
    if (!indev_pointer_) {
      logger_.error("Invalid input device!");
      return;
    }
    if (!cursor_obj) {
      logger_.error("Invalid cursor object!");
      return;
    }
    if (cursor_obj_) {
      lv_obj_del(cursor_obj_);
    }
    cursor_obj_ = cursor_obj;
    lv_indev_set_cursor(indev_pointer_, cursor_obj_);
    // get the size of the icon
    lv_coord_t w = lv_obj_get_width(cursor_obj_);
    lv_coord_t h = lv_obj_get_height(cursor_obj_);
    cursor_radius_ = std::max(w, h) / 2;
  }

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
    data->point.x = std::clamp<int>(x, 0, screen_size_x_ - cursor_radius_ * 2);
    data->point.y = std::clamp<int>(y, 0, screen_size_y_ - cursor_radius_ * 2);
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

    // make the cursor a simple circle for now
    cursor_obj_ = lv_obj_create(lv_scr_act());
    lv_obj_set_scrollbar_mode(cursor_obj_, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(cursor_obj_, cursor_radius_ * 2, cursor_radius_ * 2);
    lv_obj_set_style_radius(cursor_obj_, LV_RADIUS_CIRCLE, 0);
    // set the color to primary color
    lv_obj_set_style_bg_color(cursor_obj_, lv_palette_lighten(LV_PALETTE_BLUE, 2), 0);
    lv_obj_clear_flag(cursor_obj_, LV_OBJ_FLAG_CLICKABLE);
    lv_indev_set_cursor(indev_pointer_, cursor_obj_);

    auto disp = lv_display_get_default();
    screen_size_x_ = (uint16_t)lv_display_get_horizontal_resolution(disp);
    screen_size_y_ = (uint16_t)lv_display_get_vertical_resolution(disp);
  }

  read_fn read_;
  int cursor_radius_{8};
  uint16_t screen_size_x_;
  uint16_t screen_size_y_;
  lv_indev_t *indev_pointer_{nullptr};
  lv_obj_t *cursor_obj_{nullptr};
};
} // namespace espp

// for easy printing of PointerData using libfmt
template <> struct fmt::formatter<espp::PointerData> : fmt::formatter<std::string> {
  template <typename FormatContext>
  auto format(const espp::PointerData &c, FormatContext &ctx) const {
    return fmt::format_to(ctx.out(), "PointerData{{x={}, y={}, left_pressed={}, right_pressed={}}}",
                          c.x, c.y, c.left_pressed, c.right_pressed);
  }
};

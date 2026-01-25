#pragma once

#include "eye_drawer.hpp"
#include <cmath>
#include <mutex>

#include <esp_heap_caps.h>

namespace eye_drawer {

/// \brief Monochrome eye drawer with electric blue eyes on black background
/// \details Draws simple eyes with:
///          - Electric blue eye shapes (no pupils)
///          - Black background
///          - Eyebrows in black (cut out from the eye)
///          - Cheeks in black (cut out from eye)
class MonochromeBlueDrawer : public EyeDrawer {
public:
  struct Config {
    int screen_width;
    int screen_height;
    lv_color_t color = lv_color_hex(0x00BFFF); // Electric blue color
    lv_obj_t *canvas;
    lv_color_t *canvas_buffer;
    std::recursive_mutex &lvgl_mutex;
  };

  /**
   * @brief Construct monochrome blue drawer
   * @param config Configuration structure
   */
  explicit MonochromeBlueDrawer(const Config &config)
      : screen_width_(config.screen_width)
      , screen_height_(config.screen_height)
      , canvas_(config.canvas)
      , canvas_buffer_(config.canvas_buffer)
      , lvgl_mutex_(config.lvgl_mutex)
      , electric_blue_(config.color) {
    // Calculate eye dimensions
    original_eye_height_ = screen_height_ * 0.55f;
    eye_base_width_ = screen_width_ * 0.35f;
  }

  virtual ~MonochromeBlueDrawer() override { cleanup(); }

  virtual DrawCallback get_draw_callback() override {
    return [this](const espp::ExpressiveEyes::EyeState &left,
                  const espp::ExpressiveEyes::EyeState &right) {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);

      // Clear canvas with black background
      lv_canvas_fill_bg(canvas_, lv_color_black(), LV_OPA_COVER);

      // Draw both eyes
      draw_single_eye(left, true);
      draw_single_eye(right, false);
    };
  }

  virtual void cleanup() override {
    // No dynamic resources to free
  }

private:
  /**
   * @brief Draw a filled ellipse using LVGL layer API
   * @param cx Center X coordinate
   * @param cy Center Y coordinate
   * @param width Ellipse width
   * @param height Ellipse height
   * @param color Fill color
   */
  void draw_ellipse(int cx, int cy, int width, int height, lv_color_t color) {
    lv_layer_t layer;
    lv_canvas_init_layer(canvas_, &layer);

    lv_draw_rect_dsc_t rect_dsc;
    lv_draw_rect_dsc_init(&rect_dsc);
    rect_dsc.bg_color = color;
    rect_dsc.bg_opa = LV_OPA_COVER;
    rect_dsc.radius = LV_RADIUS_CIRCLE;
    rect_dsc.border_width = 0;

    lv_area_t area;
    area.x1 = cx - width / 2;
    area.y1 = cy - height / 2;
    area.x2 = cx + width / 2;
    area.y2 = cy + height / 2;

    lv_draw_rect(&layer, &rect_dsc, &area);
    lv_canvas_finish_layer(canvas_, &layer);
  }

  /**
   * @brief Draw a single eye with all components
   * @param eye_state Eye state data from ExpressiveEyes
   * @param is_left True if left eye, false if right eye (affects eyebrow mirroring)
   */
  void draw_single_eye(const espp::ExpressiveEyes::EyeState &eye_state, bool is_left) {
    // Draw eye in electric blue (no pupil)
    draw_ellipse(eye_state.x, eye_state.y, eye_state.width, eye_state.height, electric_blue_);

    // Draw eyebrow as rotated line
    if (eye_state.expression.eyebrow.enabled) {
      int brow_width =
          static_cast<int>(eye_base_width_ * eye_state.expression.eyebrow.width * 1.5f);
      int brow_height =
          static_cast<int>(eye_base_width_ * eye_state.expression.eyebrow.thickness * 4.0f);
      int brow_y = eye_state.y - static_cast<int>(original_eye_height_ * 0.4f);

      // For left eye, positive angle tilts left side down (clockwise rotation)
      // For right eye, positive angle tilts right side down (counter-clockwise rotation)
      float angle_rad = eye_state.expression.eyebrow.angle * M_PI / 180.0f;
      if (!is_left)
        angle_rad = -angle_rad; // Mirror for right eye

      // Calculate the two endpoints of the line
      float half_w = brow_width / 2.0f;
      float cos_a = std::cos(angle_rad);
      float sin_a = std::sin(angle_rad);

      lv_point_precise_t p1, p2;
      p1.x = eye_state.x - half_w * cos_a;
      p1.y = brow_y - half_w * sin_a;
      p2.x = eye_state.x + half_w * cos_a;
      p2.y = brow_y + half_w * sin_a;

      lv_layer_t layer;
      lv_canvas_init_layer(canvas_, &layer);

      lv_draw_line_dsc_t line_dsc;
      lv_draw_line_dsc_init(&line_dsc);
      line_dsc.color = lv_color_black();
      line_dsc.width = brow_height;
      line_dsc.opa = LV_OPA_COVER;
      line_dsc.round_start = 1;
      line_dsc.round_end = 1;
      line_dsc.p1 = p1;
      line_dsc.p2 = p2;

      lv_draw_line(&layer, &line_dsc);

      lv_canvas_finish_layer(canvas_, &layer);
    }

    // Draw cheek in electric blue
    if (eye_state.expression.cheek.enabled) {
      int cheek_width = static_cast<int>(eye_base_width_ * eye_state.expression.cheek.size * 2.5f);
      int cheek_height = static_cast<int>(eye_base_width_ * eye_state.expression.cheek.size * 1.5f);
      int cheek_y = eye_state.y + static_cast<int>(original_eye_height_ * 0.4f) +
                    static_cast<int>(eye_state.expression.cheek_offset_y * screen_height_);

      lv_layer_t layer;
      lv_canvas_init_layer(canvas_, &layer);

      lv_draw_rect_dsc_t rect_dsc;
      lv_draw_rect_dsc_init(&rect_dsc);
      rect_dsc.bg_color = lv_color_black();
      rect_dsc.bg_opa = LV_OPA_COVER;
      rect_dsc.radius = LV_RADIUS_CIRCLE;
      rect_dsc.border_width = 0;

      lv_area_t area;
      area.x1 = eye_state.x - cheek_width / 2;
      area.y1 = cheek_y - cheek_height / 2;
      area.x2 = eye_state.x + cheek_width / 2;
      area.y2 = cheek_y + cheek_height / 2;

      lv_draw_rect(&layer, &rect_dsc, &area);
      lv_canvas_finish_layer(canvas_, &layer);
    }
  }

  int screen_width_;
  int screen_height_;
  lv_obj_t *canvas_;
  lv_color_t *canvas_buffer_;
  std::recursive_mutex &lvgl_mutex_;

  int original_eye_height_;
  int eye_base_width_;
  lv_color_t electric_blue_;
};

} // namespace eye_drawer

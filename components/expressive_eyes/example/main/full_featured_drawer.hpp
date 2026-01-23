#pragma once

#include "eye_drawer.hpp"
#include <cmath>
#include <mutex>

#include <esp_heap_caps.h>

namespace eye_drawer {

/// \brief Full-featured eye drawer with white eyes, black pupils, and eyebrows
/// \details Draws realistic eyes with:
///          - White eye background
///          - Black pupils with position control
///          - Eyebrows with rotation
///          - Cheeks
class FullFeaturedDrawer : public EyeDrawer {
public:
  struct Config {
    int screen_width;
    int screen_height;
    lv_obj_t *canvas;
    lv_color_t *canvas_buffer;
    std::recursive_mutex &lvgl_mutex;
  };

  explicit FullFeaturedDrawer(const Config &config)
      : screen_width_(config.screen_width)
      , screen_height_(config.screen_height)
      , canvas_(config.canvas)
      , canvas_buffer_(config.canvas_buffer)
      , lvgl_mutex_(config.lvgl_mutex) {
    // Calculate eye dimensions
    original_eye_height_ = screen_height_ * 0.55f;
    eye_base_width_ = screen_width_ * 0.35f;
    pupil_size_ = static_cast<int>(std::min(eye_base_width_, original_eye_height_) * 0.3f);
  }

  ~FullFeaturedDrawer() override { cleanup(); }

  std::function<void(const espp::ExpressiveEyes::EyeState &,
                     const espp::ExpressiveEyes::EyeState &)>
  get_draw_callback() override {
    return [this](const espp::ExpressiveEyes::EyeState &left,
                  const espp::ExpressiveEyes::EyeState &right) {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);

      // Get background color
      lv_color_t bg_color = lv_obj_get_style_bg_color(lv_screen_active(), LV_PART_MAIN);

      // Clear canvas with background color
      lv_canvas_fill_bg(canvas_, bg_color, LV_OPA_COVER);

      // Draw both eyes
      draw_single_eye(left, true, bg_color);
      draw_single_eye(right, false, bg_color);
    };
  }

  void cleanup() override {
    // Nothing to clean up in this implementation
  }

private:
  // Helper to draw filled ellipse (for eyes) using layer API
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

  void draw_single_eye(const espp::ExpressiveEyes::EyeState &eye_state, bool is_left,
                       lv_color_t bg_color) {
    // Draw eye white
    draw_ellipse(eye_state.x, eye_state.y, eye_state.width, eye_state.height, lv_color_white());

    // Draw pupil (use original height so blink doesn't move pupil)
    // Only draw if eye is open enough (not blinking)
    float openness = eye_state.height / static_cast<float>(original_eye_height_);
    if (eye_state.expression.pupil.enabled && openness > 0.3f) {
      int px_offset = static_cast<int>(eye_state.expression.pupil.x * eye_state.width * 0.3f);
      int py_offset = static_cast<int>(eye_state.expression.pupil.y * original_eye_height_ * 0.3f);
      int pupil_x = eye_state.x + px_offset;
      int pupil_y = eye_state.y + py_offset;

      lv_layer_t layer;
      lv_canvas_init_layer(canvas_, &layer);

      lv_draw_rect_dsc_t rect_dsc;
      lv_draw_rect_dsc_init(&rect_dsc);
      rect_dsc.bg_color = lv_color_black();
      rect_dsc.bg_opa = LV_OPA_COVER;
      rect_dsc.radius = LV_RADIUS_CIRCLE;
      rect_dsc.border_width = 0;

      lv_area_t area;
      int r = pupil_size_ / 2;
      area.x1 = pupil_x - r;
      area.y1 = pupil_y - r;
      area.x2 = pupil_x + r;
      area.y2 = pupil_y + r;

      lv_draw_rect(&layer, &rect_dsc, &area);
      lv_canvas_finish_layer(canvas_, &layer);
    }

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
      line_dsc.color = bg_color;
      line_dsc.width = brow_height;
      line_dsc.opa = LV_OPA_COVER;
      line_dsc.round_start = 1;
      line_dsc.round_end = 1;
      line_dsc.p1 = p1;
      line_dsc.p2 = p2;

      lv_draw_line(&layer, &line_dsc);

      lv_canvas_finish_layer(canvas_, &layer);
    }

    // Draw cheek
    if (eye_state.expression.cheek.enabled) {
      int cheek_width = static_cast<int>(eye_base_width_ * eye_state.expression.cheek.size * 2.5f);
      int cheek_height = static_cast<int>(eye_base_width_ * eye_state.expression.cheek.size * 1.5f);
      int cheek_y = eye_state.y + static_cast<int>(original_eye_height_ * 0.4f) +
                    static_cast<int>(eye_state.expression.cheek_offset_y * screen_height_);

      lv_layer_t layer;
      lv_canvas_init_layer(canvas_, &layer);

      lv_draw_rect_dsc_t rect_dsc;
      lv_draw_rect_dsc_init(&rect_dsc);
      rect_dsc.bg_color = bg_color;
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
  int pupil_size_;
};

} // namespace eye_drawer

#include <algorithm>

#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "lilygo-t5-47.hpp"

namespace espp {

// LVGL tick source: elapsed milliseconds.
static uint32_t lvgl_tick_ms() {
  return static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
}

bool LilyGoT547::initialize_lvgl(int buffer_lines) {
  if (!initialized_) {
    logger_.error("initialize_display() must be called before initialize_lvgl()");
    return false;
  }
  if (lvgl_display_) {
    logger_.warn("LVGL already initialized");
    return true;
  }

  // lv_init() must run exactly once, even across a retry after a partial
  // failure below.
  if (!lv_is_initialized()) {
    lv_init();
  }
  lv_tick_set_cb(lvgl_tick_ms);

  const int w = width();
  const int h = height();
  if (buffer_lines <= 0 || buffer_lines > h) {
    buffer_lines = h;
  }
  // L8 = 1 byte per pixel. Two partial draw buffers in PSRAM.
  const size_t buffer_bytes = static_cast<size_t>(w) * buffer_lines;
  lvgl_buffer0_ =
      static_cast<uint8_t *>(heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  lvgl_buffer1_ =
      static_cast<uint8_t *>(heap_caps_malloc(buffer_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (lvgl_buffer0_ == nullptr || lvgl_buffer1_ == nullptr) {
    logger_.error("Could not allocate LVGL draw buffers ({} bytes each)", buffer_bytes);
    heap_caps_free(lvgl_buffer0_); // free(nullptr) is a no-op
    heap_caps_free(lvgl_buffer1_);
    lvgl_buffer0_ = lvgl_buffer1_ = nullptr;
    return false;
  }

  lvgl_display_ = lv_display_create(w, h);
  if (lvgl_display_ == nullptr) {
    logger_.error("Failed to create the LVGL display");
    heap_caps_free(lvgl_buffer0_);
    heap_caps_free(lvgl_buffer1_);
    lvgl_buffer0_ = lvgl_buffer1_ = nullptr;
    return false;
  }
  lv_display_set_color_format(lvgl_display_, LV_COLOR_FORMAT_L8);
  lv_display_set_buffers(lvgl_display_, lvgl_buffer0_, lvgl_buffer1_, buffer_bytes,
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_user_data(lvgl_display_, this);
  lv_display_set_flush_cb(lvgl_display_, &LilyGoT547::lvgl_flush_cb);
  lv_display_set_default(lvgl_display_);

  logger_.info("LVGL initialized ({}x{}, L8 grayscale, {}-line buffers)", w, h, buffer_lines);
  return true;
}

void LilyGoT547::lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  auto *self = static_cast<LilyGoT547 *>(lv_display_get_user_data(disp));
  self->lvgl_flush(disp, area, px_map);
}

void LilyGoT547::lvgl_flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  uint8_t *fb = epd_hl_get_framebuffer(&hl_);
  const int area_w = area->x2 - area->x1 + 1;
  // px_map is L8 (1 byte/pixel, 8-bit gray, 0=black..255=white). Write it into
  // the epdiy framebuffer; epd_draw_pixel is rotation-aware and packs the 8-bit
  // value into the panel's 4-bit gray nibble.
  for (int y = area->y1; y <= area->y2; ++y) {
    const uint8_t *row = px_map + static_cast<size_t>(y - area->y1) * area_w;
    for (int x = area->x1; x <= area->x2; ++x) {
      epd_draw_pixel(x, y, row[x - area->x1], fb);
    }
  }

  // Accumulate the dirty bounding box across this LVGL refresh cycle so we push
  // one panel update rather than one per flushed area.
  if (!lvgl_dirty_valid_) {
    lvgl_dirty_ = *area;
    lvgl_dirty_valid_ = true;
  } else {
    lvgl_dirty_.x1 = std::min(lvgl_dirty_.x1, area->x1);
    lvgl_dirty_.y1 = std::min(lvgl_dirty_.y1, area->y1);
    lvgl_dirty_.x2 = std::max(lvgl_dirty_.x2, area->x2);
    lvgl_dirty_.y2 = std::max(lvgl_dirty_.y2, area->y2);
  }

  if (lv_display_flush_is_last(disp)) {
    EpdRect rect = {.x = lvgl_dirty_.x1,
                    .y = lvgl_dirty_.y1,
                    .width = lvgl_dirty_.x2 - lvgl_dirty_.x1 + 1,
                    .height = lvgl_dirty_.y2 - lvgl_dirty_.y1 + 1};
    logger_.debug("panel update rect: {}x{} at ({},{})", rect.width, rect.height, rect.x, rect.y);
    bool was_off = !powered_on_;
    if (was_off) {
      power_on();
    }
    epd_hl_update_area(&hl_, lvgl_update_mode_, temperature(), rect);
    if (was_off) {
      power_off();
    }
    lvgl_dirty_valid_ = false;
  }

  lv_display_flush_ready(disp);
}

void LilyGoT547::full_refresh() {
  if (!initialized_) {
    return;
  }
  bool was_off = !powered_on_;
  if (was_off) {
    power_on();
  }
  epd_hl_update_screen(&hl_, MODE_GC16, temperature());
  if (was_off) {
    power_off();
  }
}

void LilyGoT547::set_rotation(EpdRotation rotation) {
  epd_set_rotation(rotation);
  rotation_ = rotation;
  if (lvgl_display_) {
    // Resize the LVGL display to the rotated dimensions so the UI re-lays-out.
    lv_display_set_resolution(lvgl_display_, epd_rotated_display_width(),
                              epd_rotated_display_height());
  }
  logger_.info("Rotation set to {} ({}x{})", static_cast<int>(rotation), width(), height());
}

void LilyGoT547::rotate() {
  set_rotation(static_cast<EpdRotation>((static_cast<int>(rotation_) + 1) % 4));
}

} // namespace espp

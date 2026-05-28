/**
 * @file smartpanlee_sc01_plus_example.cpp
 * @brief Smart Panlee SC01 Plus BSP Example
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <thread>
#include <vector>

#include "smartpanlee-sc01-plus.hpp"

#include "task.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
struct Circle {
  int x{0};
  int y{0};
  int radius{0};
  bool visible{false};
};
static std::array<Circle, MAX_CIRCLES> circles;
static size_t next_circle_index = 0;
static size_t visible_circle_count = 0;
static std::vector<uint8_t> audio_bytes;
static std::recursive_mutex lvgl_mutex;
static lv_obj_t *background = nullptr;
static lv_obj_t *circle_layer = nullptr;
static lv_obj_t *info_label = nullptr;
static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();
static void update_label_layout(int width);
static void rotate_display();
static bool load_audio(size_t &out_size, size_t &out_sample_rate);
static void play_click(espp::SmartPanleeSc01Plus &board);

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "SC01 Plus Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [smartpanlee sc01 plus example]
  auto &board = espp::SmartPanleeSc01Plus::get();

  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }

  if (!board.initialize_display(board.display_width() * 40)) {
    logger.error("Failed to initialize display!");
    return;
  }

  auto touch_callback = [&](const auto &touch) {
    static auto previous_touchpad_data = board.touchpad_convert(touch);
    auto touchpad_data = board.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      previous_touchpad_data = touchpad_data;
      if (touchpad_data.num_touch_points > 0) {
        play_click(board);
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  if (!board.initialize_audio()) {
    logger.warn("Audio initialization did not complete cleanly");
  } else {
    size_t wav_size = 0;
    size_t wav_sample_rate = 0;
    if (load_audio(wav_size, wav_sample_rate)) {
      logger.info("Loaded {} bytes of audio at {} Hz", wav_size, wav_sample_rate);
      board.audio_sample_rate(wav_sample_rate);
      board.volume(30.0f);
      board.mute(false);
    } else {
      logger.warn("Could not load the embedded click sound");
    }
  }

  board.brightness(80.0f);

  if (!board.initialize_sdcard()) {
    logger.info("No microSD card mounted");
  }

  auto i2s = board.i2s_pins();
  auto rs485 = board.rs485_pins();
  logger.info("I2S pins: bclk={}, ws={}, dout={}", i2s.bclk, i2s.ws, i2s.dout);
  logger.info("RS485 pins: rts={}, rxd={}, txd={}", rs485.rts, rs485.rxd, rs485.txd);

  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, board.display_width(), board.display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(8, 12, 24), 0);

  info_label = lv_label_create(lv_screen_active());
  lv_label_set_text(info_label, "Smart Panlee SC01 Plus\n\nTouch the screen to draw and play a "
                                "click.\nPress refresh to rotate.\nCheck serial output for SD "
                                "card, pin, and audio info.");
  lv_label_set_long_mode(info_label, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_align(info_label, LV_TEXT_ALIGN_LEFT, 0);
  update_label_layout(static_cast<int>(board.display_width()));

  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 56, 56);
  lv_obj_align(btn, LV_ALIGN_TOP_RIGHT, -12, 12);
  lv_obj_t *btn_label = lv_label_create(btn);
  lv_label_set_text(btn_label, LV_SYMBOL_REFRESH);
  lv_obj_align(btn_label, LV_ALIGN_CENTER, 0, 0);
  background = bg;
  if (!initialize_circle_layer(board.display_width(), board.display_height())) {
    logger.error("Failed to initialize circle layer!");
    return;
  }
  lv_obj_add_event_cb(
      btn,
      [](lv_event_t *event) {
        (void)event;
        rotate_display();
      },
      LV_EVENT_PRESSED, nullptr);

  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_foreground(circle_layer);

  if (!board.initialize_touch(touch_callback)) {
    logger.warn("Touch initialization did not complete cleanly");
  }

  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                          .stack_size_bytes = 6 * 1024,
                      }});
  lv_task.start();
  //! [smartpanlee sc01 plus example]

  while (true) {
    std::this_thread::sleep_for(1s);
    if (board.is_sd_card_available()) {
      uint32_t size_mb = 0;
      uint32_t free_mb = 0;
      if (board.get_sd_card_info(&size_mb, &free_mb)) {
        logger.info("microSD: size={} MB free={} MB", size_mb, free_mb);
      }
    }
  }
}

static bool initialize_circle_layer(int width, int height) {
  if (circle_layer) {
    return true;
  }
  circle_layer = lv_obj_create(lv_screen_active());
  if (!circle_layer) {
    return false;
  }
  lv_obj_remove_style_all(circle_layer);
  lv_obj_set_size(circle_layer, width, height);
  lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_clear_flag(circle_layer, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_opa(circle_layer, LV_OPA_TRANSP, 0);
  lv_obj_set_style_border_width(circle_layer, 0, 0);
  lv_obj_set_style_outline_width(circle_layer, 0, 0);
  lv_obj_set_style_shadow_width(circle_layer, 0, 0);
  lv_obj_add_event_cb(circle_layer, draw_circle_layer, LV_EVENT_DRAW_MAIN, nullptr);
  return true;
}

static void draw_circle_layer(lv_event_t *event) {
  if (visible_circle_count == 0) {
    return;
  }

  auto *obj = static_cast<lv_obj_t *>(lv_event_get_current_target(event));
  auto *layer = lv_event_get_layer(event);
  lv_area_t obj_coords;
  lv_obj_get_coords(obj, &obj_coords);

  lv_draw_rect_dsc_t rect_dsc;
  lv_draw_rect_dsc_init(&rect_dsc);
  rect_dsc.base.layer = layer;
  rect_dsc.radius = LV_RADIUS_CIRCLE;
  rect_dsc.bg_opa = LV_OPA_70;
  rect_dsc.bg_color = lv_color_make(0, 255, 255);
  rect_dsc.border_width = 0;
  rect_dsc.outline_width = 0;
  rect_dsc.shadow_width = 0;

  for (const auto &circle : circles) {
    if (!circle.visible) {
      continue;
    }
    lv_area_t coords = {
        .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
        .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
        .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
        .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
    };
    lv_draw_rect(layer, &rect_dsc, &coords);
  }
}

static void invalidate_circle_area(const Circle &circle) {
  if (!circle_layer || circle.radius <= 0) {
    return;
  }

  lv_area_t obj_coords;
  lv_obj_get_coords(circle_layer, &obj_coords);
  lv_area_t coords = {
      .x1 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x - circle.radius),
      .y1 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y - circle.radius),
      .x2 = static_cast<lv_coord_t>(obj_coords.x1 + circle.x + circle.radius - 1),
      .y2 = static_cast<lv_coord_t>(obj_coords.y1 + circle.y + circle.radius - 1),
  };
  lv_obj_invalidate_area(circle_layer, &coords);
}

static void draw_circle(int x0, int y0, int radius) {
  if (!circle_layer) {
    return;
  }
  lv_obj_move_foreground(circle_layer);
  Circle previous_circle = circles[next_circle_index];
  circles[next_circle_index] = {.x = x0, .y = y0, .radius = radius, .visible = true};
  next_circle_index = (next_circle_index + 1) % circles.size();
  if (visible_circle_count < circles.size()) {
    visible_circle_count++;
  }
  if (previous_circle.visible) {
    invalidate_circle_area(previous_circle);
  }
  invalidate_circle_area(circles[(next_circle_index + circles.size() - 1) % circles.size()]);
}

static void clear_circles() {
  for (auto &circle : circles) {
    if (circle.visible) {
      invalidate_circle_area(circle);
    }
    circle.visible = false;
  }
  next_circle_index = 0;
  visible_circle_count = 0;
}

static void update_label_layout(int width) {
  if (!info_label) {
    return;
  }

  auto label_width = std::max(width - 96, 120);
  lv_obj_set_width(info_label, label_width);
  lv_obj_align(info_label, LV_ALIGN_TOP_LEFT, 16, 16);
}

static void rotate_display() {
  auto &board = espp::SmartPanleeSc01Plus::get();
  std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
  clear_circles();
  static auto rotation = LV_DISPLAY_ROTATION_0;
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  auto *display = lv_display_get_default();
  lv_disp_set_rotation(display, rotation);
  if (background) {
    lv_obj_set_size(background, board.rotated_display_width(), board.rotated_display_height());
  }
  update_label_layout(static_cast<int>(board.rotated_display_width()));
  if (circle_layer) {
    lv_obj_set_size(circle_layer, board.rotated_display_width(), board.rotated_display_height());
    lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
    lv_obj_move_foreground(circle_layer);
    lv_obj_invalidate(circle_layer);
  }
}

static bool load_audio(size_t &out_size, size_t &out_sample_rate) {
  if (!audio_bytes.empty()) {
    out_size = audio_bytes.size();
    return true;
  }

  extern const uint8_t click_wav_start[] asm("_binary_click_wav_start");
  extern const uint8_t click_wav_end[] asm("_binary_click_wav_end");
  audio_bytes = std::vector<uint8_t>(click_wav_start, click_wav_end);
  if (audio_bytes.size() < 44) {
    audio_bytes.clear();
    return false;
  }

  auto sample_rate = *(reinterpret_cast<const uint32_t *>(&audio_bytes[24]));
  if (audio_bytes.size() > 44) {
    audio_bytes.erase(audio_bytes.begin(), audio_bytes.begin() + 44);
  }
  out_size = audio_bytes.size();
  out_sample_rate = sample_rate;
  return true;
}

static void play_click(espp::SmartPanleeSc01Plus &board) {
  if (audio_bytes.empty()) {
    return;
  }

  auto audio_buffer_size = board.audio_buffer_size();
  if (audio_buffer_size == 0) {
    return;
  }

  size_t offset = 0;
  while (offset < audio_bytes.size()) {
    auto bytes_to_play = std::min(audio_buffer_size, audio_bytes.size() - offset);
    board.play_audio(audio_bytes.data() + offset, static_cast<uint32_t>(bytes_to_play));
    offset += bytes_to_play;
  }
}

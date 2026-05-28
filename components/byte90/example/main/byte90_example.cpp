#include <array>
#include <chrono>
#include <stdlib.h>

#include "byte90.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 10;
struct Circle {
  int x{0};
  int y{0};
  int radius{0};
  bool visible{false};
};
static std::array<Circle, MAX_CIRCLES> circles;
static size_t next_circle_index = 0;
static size_t visible_circle_count = 0;
static lv_obj_t *circle_layer = nullptr;

static std::recursive_mutex lvgl_mutex;
static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Byte90 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [byte90 example]
  espp::Byte90 &byte90 = espp::Byte90::get();
  byte90.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the accelerometer
  if (!byte90.initialize_accelerometer()) {
    logger.error("Failed to initialize accelerometer!");
  }
  // initialize the LCD
  if (!byte90.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = byte90.lcd_width() * 50;
  // initialize the LVGL display for the Byte90
  if (!byte90.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // initialize the button, which we'll use to cycle the rotation of the display
  logger.info("Initializing the button");
  lv_obj_t *bg = nullptr;
  lv_obj_t *label = nullptr;
  static auto update_layout = [&]() {
    int width = byte90.rotated_display_width();
    int height = byte90.rotated_display_height();
    lv_obj_set_size(bg, width, height);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    if (circle_layer) {
      lv_obj_set_size(circle_layer, width, height);
      lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
      lv_obj_move_foreground(circle_layer);
      lv_obj_invalidate(circle_layer);
    }
  };
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      // increment the brightness by 10%, looping back to 0% after 100%
      auto brightness = byte90.brightness();
      brightness = std::fmod(brightness + 10.0f, 100.0f);
      logger.info("Setting brightness to {:.0f}%", brightness);
      byte90.brightness(brightness);
      // lock the display mutex
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      static auto rotation = LV_DISPLAY_ROTATION_0;
      rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
      lv_display_t *disp = lv_display_get_default();
      lv_disp_set_rotation(disp, rotation);
      update_layout();
    }
  };
  byte90.initialize_button(on_button_pressed);

  // set the background color to black
  bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, byte90.rotated_display_width(), byte90.rotated_display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);
  if (!initialize_circle_layer(byte90.rotated_display_width(), byte90.rotated_display_height())) {
    logger.error("Failed to initialize circle layer!");
    return;
  }

  // add text in the center of the screen
  label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Drawing circles\nto the screen.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
  update_layout();

  lv_obj_move_foreground(circle_layer);

  // start a simple thread to do the lv_task_handler every 16ms
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
                      }});
  lv_task.start();

  // set the display brightness to be 75%
  byte90.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // if there are 10 circles on the screen, clear them
    if (visible_circle_count >= MAX_CIRCLES) {
      // lock the lvgl mutex
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = byte90.rotated_display_width() / 2;
      int middle_y = byte90.rotated_display_height() / 2;
      static constexpr int radius = 30;
      float angle = visible_circle_count * 2.0f * M_PI / MAX_CIRCLES;
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);
      // lock the lvgl mutex
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [byte90 example]
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

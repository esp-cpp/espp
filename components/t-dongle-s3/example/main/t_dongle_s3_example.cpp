#include <array>
#include <chrono>
#include <stdlib.h>

#include "t-dongle-s3.hpp"

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

static std::mutex lvgl_mutex;
static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "T-Dongle-S3 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [t-dongle-s3 example]
  espp::TDongleS3 &tdongle = espp::TDongleS3::get();
  tdongle.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LED
  if (!tdongle.initialize_led()) {
    logger.error("Failed to initialize led!");
    return;
  }
  // initialize the LCD
  if (!tdongle.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be a full screen buffer
  static constexpr size_t pixel_buffer_size = tdongle.lcd_width() * tdongle.lcd_height();
  // initialize the LVGL display for the T-Dongle-S3
  if (!tdongle.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the uSD card
  using SdCardConfig = espp::TDongleS3::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!tdongle.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize SD card, continuing without it.");
  }

  // initialize the button, which we'll use to cycle the rotation of the display
  logger.info("Initializing the button");
  lv_obj_t *bg = nullptr;
  lv_obj_t *label = nullptr;
  static auto update_layout = [&]() {
    int width = tdongle.rotated_display_width();
    int height = tdongle.rotated_display_height();
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
      // lock the display mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      static auto rotation = LV_DISPLAY_ROTATION_0;
      rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
      fmt::print("Setting rotation to {}\n", (int)rotation);
      lv_display_t *disp = lv_display_get_default();
      lv_disp_set_rotation(disp, rotation);
      update_layout();
    }
  };
  tdongle.initialize_button(on_button_pressed);

  // set the LED to be red
  espp::Hsv hsv(150.0f, 1.0f, 1.0f);
  float brightness = 5.0f; // 5% brightness
  tdongle.led(hsv, brightness);

  // set the background color to black
  bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, tdongle.rotated_display_width(), tdongle.rotated_display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);
  if (!initialize_circle_layer(tdongle.rotated_display_width(), tdongle.rotated_display_height())) {
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
                          // lock the display mutex
                          std::lock_guard<std::mutex> lock(lvgl_mutex);
                          lv_task_handler();
                        }
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                          .core_id = 1,
                      }});
  lv_task.start();

  // set the display brightness to be 75%
  tdongle.brightness(75.0f);

  // make a task to constantly shift the hue of the LED
  espp::Task led_task({.callback = [&](std::mutex &m, std::condition_variable &cv) -> bool {
                         static float hue = 0.0f;
                         hue += 1.0f;
                         if (hue >= 360.0f) {
                           hue = 0.0f;
                         }
                         espp::Hsv hsv(hue, 1.0f, 1.0f);
                         espp::TDongleS3::get().led(hsv, brightness);
                         std::unique_lock<std::mutex> lock(m);
                         cv.wait_for(lock, 25ms);
                         return false;
                       },
                       .task_config = {
                           .name = "led_task",
                       }});
  led_task.start();

  while (true) {
    auto start = esp_timer_get_time();
    // if there are 10 circles on the screen, clear them
    if (visible_circle_count >= MAX_CIRCLES) {
      // lock the lvgl mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = tdongle.rotated_display_width() / 2;
      int middle_y = tdongle.rotated_display_height() / 2;
      static constexpr int radius = 30;
      float angle = visible_circle_count * 2.0f * M_PI / MAX_CIRCLES;
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);
      // lock the lvgl mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [t-dongle-s3 example]
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

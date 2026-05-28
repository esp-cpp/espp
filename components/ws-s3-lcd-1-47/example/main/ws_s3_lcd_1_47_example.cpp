#include <array>
#include <chrono>

#include "ws-s3-lcd-1-47.hpp"

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
  espp::Logger logger({.tag = "WsS3Lcd147 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [ws-s3-lcd-1-47 example]
  using Bsp = espp::WsS3Lcd147;
  Bsp &board = Bsp::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be a full screen buffer
  static constexpr size_t pixel_buffer_size = board.lcd_width() * 50;
  // initialize the LVGL display for the WsS3Lcd147
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the uSD card
  using SdCardConfig = Bsp::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!board.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize SD card, continuing without it.");
  }

  // initialize the LED
  if (!board.initialize_led()) {
    logger.error("Failed to initialize LED!");
    return;
  }

  // initialize the button, which we'll use to cycle the rotation of the display
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      // lock the display mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      static auto rotation = LV_DISPLAY_ROTATION_0;
      rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
      fmt::print("Setting rotation to {}\n", (int)rotation);
      lv_display_t *disp = lv_disp_get_default();
      lv_disp_set_rotation(disp, rotation);
      if (circle_layer) {
        lv_obj_set_size(circle_layer, lv_display_get_horizontal_resolution(disp),
                        lv_display_get_vertical_resolution(disp));
        lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
        lv_obj_move_foreground(circle_layer);
        lv_obj_invalidate(circle_layer);
      }
    }
  };
  board.initialize_button(on_button_pressed);

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, board.lcd_width(), board.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);
  if (!initialize_circle_layer(board.lcd_width(), board.lcd_height())) {
    logger.error("Failed to initialize circle layer!");
    return;
  }

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Drawing circles\nto the screen.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

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
  board.brightness(75.0f);

  espp::Task led_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                         static auto &board = Bsp::get();
                         auto start = std::chrono::steady_clock::now();
                         // set the LED color
                         static float hue = 0;
                         hue = std::fmod(hue + 1.0f, 360.0f);
                         board.led(espp::Hsv(hue, 100.0f, 100.0f));
                         // wait for 100ms
                         std::unique_lock<std::mutex> lock(m);
                         cv.wait_until(lock, start + 10ms);
                         return false;
                       },
                       .task_config = {
                           .name = "led_task",
                           .core_id = 1,
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
      static constexpr int middle_x = Bsp::lcd_width() / 2;
      static constexpr int middle_y = Bsp::lcd_height() / 2;
      static constexpr int radius = 50;
      float angle = visible_circle_count * 2.0f * M_PI / MAX_CIRCLES;
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);

      // handle the rotation of the display to ensure the circles are centered
      auto rotation = lv_display_get_rotation(lv_disp_get_default());
      switch (rotation) {
      default:
      case LV_DISPLAY_ROTATION_0:
      case LV_DISPLAY_ROTATION_180:
        break;
      case LV_DISPLAY_ROTATION_90:
      case LV_DISPLAY_ROTATION_270:
        std::swap(x, y);
        break;
      }

      // lock the lvgl mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      draw_circle(x, y, 10);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [ws-s3-lcd-1-47 example]
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

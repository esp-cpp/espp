#include <array>
#include <chrono>
#include <stdlib.h>

#include "seeed-studio-round-display.hpp"

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
static lv_obj_t *circle_layer = nullptr;

static std::recursive_mutex lvgl_mutex;
static bool initialize_circle_layer(int width, int height);
static void draw_circle_layer(lv_event_t *event);
static void invalidate_circle_area(const Circle &circle);
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Seeed Studio Round Display Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [seeed studio round display example]
#if CONFIG_EXAMPLE_HARDWARE_XIAOS3
  logger.info("Using XiaoS3 hardware configuration");
  espp::SsRoundDisplay::set_pin_config(espp::SsRoundDisplay::XiaoS3Config);
#elif CONFIG_EXAMPLE_HARDWARE_QTPYS3
  logger.info("Using QtpyS3 hardware configuration");
  espp::SsRoundDisplay::set_pin_config(espp::SsRoundDisplay::QtpyS3Config);
#else
#error "Please select a hardware configuration"
#endif
  espp::SsRoundDisplay &round_display = espp::SsRoundDisplay::get();

  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = round_display.touchpad_convert(touch);
    auto touchpad_data = round_display.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
      }
      // if there is a touch point, draw a circle
      if (touchpad_data.num_touch_points > 0) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };

  // initialize the LCD
  if (!round_display.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = round_display.lcd_width() * 50;
  // initialize the LVGL display for the seeed-studio-round-display
  if (!round_display.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, round_display.rotated_display_width(),
                  round_display.rotated_display_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);
  if (!initialize_circle_layer(round_display.rotated_display_width(),
                               round_display.rotated_display_height())) {
    logger.error("Failed to initialize circle layer!");
    return;
  }

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Touch the screen!");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

  // add a button in the top middel which (when pressed) will rotate the display
  // through 0, 90, 180, 270 degrees
  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 50, 50);
  lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_t *label_btn = lv_label_create(btn);
  lv_label_set_text(label_btn, LV_SYMBOL_REFRESH);
  lv_obj_align(label_btn, LV_ALIGN_CENTER, 0, 0);

  // add a button in the bottom middle which (when pressed) will clear the
  // circles
  lv_obj_t *btn_clear = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn_clear, 50, 50);
  lv_obj_align(btn_clear, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_state(btn_clear, LV_STATE_CHECKED); // make the button red
  lv_obj_t *label_btn_clear = lv_label_create(btn_clear);
  lv_label_set_text(label_btn_clear, LV_SYMBOL_TRASH);
  lv_obj_align(label_btn_clear, LV_ALIGN_CENTER, 0, 0);
  static auto update_layout = [&]() {
    int width = round_display.rotated_display_width();
    int height = round_display.rotated_display_height();
    lv_obj_set_size(bg, width, height);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
    lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_align(btn_clear, LV_ALIGN_BOTTOM_MID, 0, 0);
    if (circle_layer) {
      lv_obj_set_size(circle_layer, width, height);
      lv_obj_align(circle_layer, LV_ALIGN_CENTER, 0, 0);
      lv_obj_move_foreground(circle_layer);
      lv_obj_invalidate(circle_layer);
    }
  };
  static auto rotate_display = [&]() {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
    clear_circles();
    static auto rotation = LV_DISPLAY_ROTATION_0;
    rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
    lv_display_t *disp = lv_display_get_default();
    lv_disp_set_rotation(disp, rotation);
    update_layout();
  };
  lv_obj_add_event_cb(
      btn, [](auto event) { rotate_display(); }, LV_EVENT_PRESSED, nullptr);
  lv_obj_add_event_cb(
      btn_clear,
      [](auto event) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
      },
      LV_EVENT_PRESSED, nullptr);
  update_layout();

  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_move_foreground(circle_layer);

  // initialize the touchpad after the circle layer exists so touch events can
  // update it immediately.
  if (!round_display.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

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
  round_display.brightness(75.0f);

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [seeed studio round display example]
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

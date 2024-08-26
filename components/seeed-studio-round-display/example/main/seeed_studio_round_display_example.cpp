#include <chrono>
#include <deque>
#include <stdlib.h>
#include <vector>

#include "seeed-studio-round-display.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::deque<lv_obj_t *> circles;

static std::recursive_mutex lvgl_mutex;
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();
static void on_rotate_pressed(lv_event_t *event);
static void on_clear_pressed(lv_event_t *event);

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
        clear_circles();
      }
      // if there is a touch point, draw a circle
      if (touchpad_data.num_touch_points > 0) {
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
  espp::Task::BaseConfig display_task_config = {
      .name = "Display",
      .stack_size_bytes = 6 * 1024,
      .priority = 10,
      .core_id = 0,
  };
  // initialize the LVGL display for the seeed-studio-round-display
  if (!round_display.initialize_display(pixel_buffer_size, display_task_config)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // initialize the touchpad
  if (!round_display.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, round_display.lcd_width(), round_display.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

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
  lv_obj_add_event_cb(btn, on_rotate_pressed, LV_EVENT_PRESSED, nullptr);

  // add a button in the bottom middle which (when pressed) will clear the
  // circles
  lv_obj_t *btn_clear = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn_clear, 50, 50);
  lv_obj_align(btn_clear, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_add_state(btn_clear, LV_STATE_CHECKED); // make the button red
  lv_obj_t *label_btn_clear = lv_label_create(btn_clear);
  lv_label_set_text(label_btn_clear, LV_SYMBOL_TRASH);
  lv_obj_align(label_btn_clear, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(btn_clear, on_clear_pressed, LV_EVENT_PRESSED, nullptr);

  // disable scrolling on the screen (so that it doesn't behave weirdly when
  // rotated and drawing with your finger)
  lv_obj_set_scrollbar_mode(lv_screen_active(), LV_SCROLLBAR_MODE_OFF);
  lv_obj_clear_flag(lv_screen_active(), LV_OBJ_FLAG_SCROLLABLE);

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

static void on_rotate_pressed(lv_event_t *event) {
  clear_circles();
  std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
  static auto rotation = LV_DISPLAY_ROTATION_0;
  rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
  lv_display_t *disp = _lv_refr_get_disp_refreshing();
  lv_disp_set_rotation(disp, rotation);
}

static void on_clear_pressed(lv_event_t *event) { clear_circles(); }

static void draw_circle(int x0, int y0, int radius) {
  std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
  // if the number of circles is greater than the max, remove the oldest circle
  if (circles.size() > MAX_CIRCLES) {
    lv_obj_delete(circles.front());
    circles.pop_front();
  }
  lv_obj_t *my_Cir = lv_obj_create(lv_screen_active());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, radius * 2, radius * 2);
  lv_obj_set_pos(my_Cir, x0 - radius, y0 - radius);
  lv_obj_set_style_radius(my_Cir, LV_RADIUS_CIRCLE, 0);
  // ensure the circle ignores touch events (so things behind it can still be
  // interacted with)
  lv_obj_clear_flag(my_Cir, LV_OBJ_FLAG_CLICKABLE);
  circles.push_back(my_Cir);
}

static void clear_circles() {
  std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
  // remove the circles from lvgl
  for (auto circle : circles) {
    lv_obj_delete(circle);
  }
  // clear the vector
  circles.clear();
}

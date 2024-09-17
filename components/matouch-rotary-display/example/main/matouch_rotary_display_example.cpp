#include <chrono>
#include <deque>
#include <stdlib.h>

#include "matouch-rotary-display.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::deque<lv_obj_t *> circles;

static std::recursive_mutex lvgl_mutex;

static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Matouch-Rotary-Display Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [matouch-rotary-display example]
  espp::MatouchRotaryDisplay &mt_display = espp::MatouchRotaryDisplay::get();
  mt_display.set_log_level(espp::Logger::Verbosity::INFO);

  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      logger.info("Button pressed!");
    } else {
      logger.info("Button released!");
      // clear the screen
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      clear_circles();
    }
  };

  auto on_touch = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = mt_display.touchpad_convert(touch);
    auto touchpad_data = mt_display.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if there is a touch point, draw a circle
      if (touchpad_data.num_touch_points > 0) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
    previous_touchpad_data = touchpad_data;
  };

  // initialize the LCD
  if (!mt_display.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = mt_display.lcd_width() * 50;
  // initialize the LVGL display for the Matouch-Rotary-Display
  if (!mt_display.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // initialize the touchpad
  if (!mt_display.initialize_touch(on_touch)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }
  // initialize the rotary encoder
  if (!mt_display.initialize_encoder()) {
    logger.error("Failed to initialize rotary encoder!");
    return;
  }
  // initialize the button
  if (!mt_display.initialize_button(on_button_pressed)) {
    logger.error("Failed to initialize button!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, mt_display.lcd_width(), mt_display.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Touch the screen!\nPress the button to clear circles.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

  // add a button in the top left which (when pressed) will rotate the display
  // through 0, 90, 180, 270 degrees
  lv_obj_t *btn = lv_btn_create(lv_screen_active());
  lv_obj_set_size(btn, 50, 50);
  lv_obj_align(btn, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_t *label_btn = lv_label_create(btn);
  lv_label_set_text(label_btn, LV_SYMBOL_REFRESH);
  // center the text in the button
  lv_obj_align(label_btn, LV_ALIGN_CENTER, 0, 0);
  lv_obj_add_event_cb(
      btn,
      [](auto event) {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
        clear_circles();
        static auto rotation = LV_DISPLAY_ROTATION_0;
        rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
        lv_display_t *disp = _lv_refr_get_disp_refreshing();
        lv_disp_set_rotation(disp, rotation);
      },
      LV_EVENT_PRESSED, nullptr);

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
  mt_display.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // get the encoder count and update the label with it
    {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex);
      int encoder_count = mt_display.encoder_value();
      lv_label_set_text_fmt(label,
                            "Touch the screen!\nPress the button to clear circles.\nEncoder: %d",
                            encoder_count);
    }
    // sleep for the remaining time
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(50ms - std::chrono::microseconds(elapsed));
  }
  //! [matouch-rotary-display example]
}

static void draw_circle(int x0, int y0, int radius) {
  // if the number of circles exceeds the max, remove the oldest circle
  if (circles.size() >= MAX_CIRCLES) {
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
  // remove the circles from lvgl
  for (auto circle : circles) {
    lv_obj_delete(circle);
  }
  // clear the vector
  circles.clear();
}

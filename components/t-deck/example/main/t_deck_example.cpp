#include <chrono>
#include <deque>
#include <stdlib.h>

#include "t-deck.hpp"

using namespace std::chrono_literals;

static constexpr size_t MAX_CIRCLES = 100;
static std::deque<lv_obj_t *> circles;

static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "T-Deck Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [t-deck example]
  espp::TDeck &tdeck = espp::TDeck::get();
  tdeck.set_log_level(espp::Logger::Verbosity::INFO);

  auto keypress_callback = [&](uint8_t key) {
    logger.info("Key pressed: {}", key);
    if (key == 8) {
      clear_circles();
    }
  };

  // initialize the Keyboard
  bool start_task = true;
  if (!tdeck.initialize_keyboard(start_task, keypress_callback)) {
    logger.error("Failed to initialize Keyboard!");
    return;
  }
  // initialize the LCD
  if (!tdeck.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = tdeck.lcd_width() * 50;
  // initialize the LVGL display for the T-Deck
  if (!tdeck.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }
  // initialize the touchpad
  if (!tdeck.initialize_touch()) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_scr_act());
  lv_obj_set_size(bg, tdeck.lcd_width(), tdeck.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Touch the screen!\nPress the delete key to clear circles.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

  static std::mutex lvgl_mutex;

  // start a simple thread to do the lv_task_handler every 16ms
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        {
                          std::lock_guard<std::mutex> lock(lvgl_mutex);
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
  tdeck.brightness(75.0f);

  auto previous_touchpad_data = tdeck.touchpad_convert(tdeck.touchpad_data());
  while (true) {
    auto start = esp_timer_get_time();
    if (tdeck.update_touch()) {
      // NOTE: since we're directly using the touchpad data, and not using the
      // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
      // converted into proper screen coordinates instead of simply using the
      // raw values.
      auto touchpad_data = tdeck.touchpad_convert(tdeck.touchpad_data());
      if (touchpad_data != previous_touchpad_data) {
        logger.info("Touch: {}", touchpad_data);
        previous_touchpad_data = touchpad_data;
        // if the button is pressed, clear the circles
        if (touchpad_data.btn_state) {
          std::lock_guard<std::mutex> lock(lvgl_mutex);
          clear_circles();
        }
        // if there is a touch point, draw a circle and play a click sound
        if (touchpad_data.num_touch_points > 0) {
          std::lock_guard<std::mutex> lock(lvgl_mutex);
          draw_circle(touchpad_data.x, touchpad_data.y, 10);
        }
      }
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(50ms - std::chrono::microseconds(elapsed));
  }
  //! [t-deck example]
}

static void draw_circle(int x0, int y0, int radius) {
  // if we have too many circles, remove the oldest one
  if (circles.size() >= MAX_CIRCLES) {
    lv_obj_del(circles.front());
    circles.pop_front();
  }
  lv_obj_t *my_Cir = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, radius * 2, radius * 2);
  lv_obj_set_pos(my_Cir, x0 - radius, y0 - radius);
  lv_obj_set_style_radius(my_Cir, LV_RADIUS_CIRCLE, 0);
  circles.push_back(my_Cir);
}

static void clear_circles() {
  // remove the circles from lvgl
  for (auto circle : circles) {
    lv_obj_del(circle);
  }
  // clear the vector
  circles.clear();
}

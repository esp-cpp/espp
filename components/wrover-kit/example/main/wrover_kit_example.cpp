#include <chrono>
#include <stdlib.h>
#include <vector>

#include "wrover-kit.hpp"

using namespace std::chrono_literals;

static std::vector<lv_obj_t *> circles;

static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Wrover-Kit Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [wrover-kit example]
  espp::WroverKit &wrover = espp::WroverKit::get();
  wrover.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!wrover.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = wrover.lcd_width() * 50;
  // initialize the LVGL display for the wrover-kit
  if (!wrover.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  logger.info("Adding LVGL objects to the screen.");

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_scr_act());
  lv_obj_set_size(bg, wrover.lcd_width(), wrover.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Drawing circles to the screen.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

  // start a simple thread to do the lv_task_handler every 16ms
  espp::Task lv_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                        lv_task_handler();
                        std::unique_lock<std::mutex> lock(m);
                        cv.wait_for(lock, 16ms);
                        return false;
                      },
                      .task_config = {
                          .name = "lv_task",
                      }});
  lv_task.start();

  // set the display brightness to be 75%
  wrover.brightness(75.0f);

  while (true) {
    std::this_thread::sleep_for(100ms);
    // if there are 10 circles on the screen, clear them
    if (circles.size() >= 10) {
      clear_circles();
    } else {
      // draw a random circle on the screen
      int x = rand() % wrover.lcd_width();
      int y = rand() % wrover.lcd_height();
      draw_circle(x, y, 10);
    }
  }
  //! [wrover-kit example]
}

static void draw_circle(int x0, int y0, int radius) {
  lv_obj_t *my_Cir = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, 42, 42);
  lv_obj_set_pos(my_Cir, x0 - 21, y0 - 21);
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

#include <chrono>
#include <vector>

#include "esp-box.hpp"

using namespace std::chrono_literals;

static std::vector<lv_obj_t *> circles;

static void draw_circle(int x0, int y0, int radius) {
  lv_obj_t *my_Cir = lv_obj_create(lv_scr_act());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, 42, 42);
  lv_obj_set_pos(my_Cir, x0 - 21, y0 - 21);
  lv_obj_set_style_radius(my_Cir, LV_RADIUS_CIRCLE, 0);
  circles.push_back(my_Cir);
}

static void clear_circles() {
  for (auto circle : circles) {
    lv_obj_del(circle);
  }
  circles.clear();
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ESP BOX Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [esp box example]
  espp::EspBox box;
  logger.info("Running on {}", box.box_type());
  if (!box.initialize_touch()) {
    logger.error("Failed to initialize touchpad!");
    return;
  }
  if (!box.initialize_sound()) {
    logger.error("Failed to initialize sound!");
    return;
  }
  if (!box.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // pixel buffer is 50 lines high
  static constexpr size_t pixel_buffer_size = box.lcd_width() * 50;
  if (!box.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_scr_act());
  lv_obj_set_size(bg, box.lcd_width(), box.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "Touch the screen!\nPress the home button to clear circles.");
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

  while (true) {
    std::this_thread::sleep_for(100ms);
    if (box.update_touch()) {
      auto touchpad_data = box.touchpad_data();
      logger.info("Touch: {}", touchpad_data);
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        clear_circles();
      }
      if (touchpad_data.num_touch_points > 0) {
        draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  }
  //! [esp box example]
}

#include <chrono>
#include <stdlib.h>
#include <vector>

#include "ws-s3-geek.hpp"

using namespace std::chrono_literals;

static std::vector<lv_obj_t *> circles;

static std::mutex lvgl_mutex;
static void draw_circle(int x0, int y0, int radius);
static void clear_circles();

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WsS3Geek Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [ws-s3-geek example]
  espp::WsS3Geek &board = espp::WsS3Geek::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be a full screen buffer
  static constexpr size_t pixel_buffer_size = board.lcd_width() * 50;
  // initialize the LVGL display for the WsS3Geek
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the uSD card
  using SdCardConfig = espp::WsS3Geek::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!board.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize SD card, continuing without it.");
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
    }
  };
  board.initialize_button(on_button_pressed);

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, board.lcd_width(), board.lcd_height());
  lv_obj_set_style_bg_color(bg, lv_color_make(0, 0, 0), 0);

  // add text in the center of the screen
  lv_obj_t *label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Drawing circles\nto the screen.");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);

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

  while (true) {
    auto start = esp_timer_get_time();
    // if there are 10 circles on the screen, clear them
    static constexpr int max_circles = 10;
    if (circles.size() >= max_circles) {
      // lock the lvgl mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      static constexpr int middle_x = espp::WsS3Geek::lcd_width() / 2;
      static constexpr int middle_y = espp::WsS3Geek::lcd_height() / 2;
      static constexpr int radius = 30;
      float angle = circles.size() * 2.0f * M_PI / max_circles;
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
      draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [ws-s3-geek example]
}

static void draw_circle(int x0, int y0, int radius) {
  lv_obj_t *my_Cir = lv_obj_create(lv_screen_active());
  lv_obj_set_scrollbar_mode(my_Cir, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(my_Cir, radius * 2, radius * 2);
  lv_obj_set_pos(my_Cir, x0 - radius, y0 - radius);
  lv_obj_set_style_radius(my_Cir, LV_RADIUS_CIRCLE, 0);
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

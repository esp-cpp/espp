#include <chrono>
#include <stdlib.h>
#include <vector>

#include "button.hpp"
#include "t-dongle-s3.hpp"

using namespace std::chrono_literals;

static std::vector<lv_obj_t *> circles;

static std::mutex lvgl_mutex;
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

  // initialize the button, which we'll use to cycle the rotation of the display
  espp::Button button(espp::Button::Config{
      .name = "Boot Button",
      .interrupt_config = espp::Interrupt::PinConfig{
          .gpio_num = GPIO_NUM_0,
          .callback = [](const auto &event) {
            if (event.active) {
              // lock the display mutex
              std::lock_guard<std::mutex> lock(lvgl_mutex);
              static auto rotation = LV_DISPLAY_ROTATION_0;
              rotation = static_cast<lv_display_rotation_t>((static_cast<int>(rotation) + 1) % 4);
              fmt::print("Setting rotation to {}\n", (int)rotation);
              lv_display_t *disp = _lv_refr_get_disp_refreshing();
              lv_disp_set_rotation(disp, rotation);
            }
          },
          .active_level = espp::Interrupt::ActiveLevel::LOW,
          .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
          .pullup_enabled = false,
          .pulldown_enabled = false
      },
    });

  // set the LED to be red
  espp::Hsv hsv(150.0f, 1.0f, 1.0f);
  float brightness = 5.0f; // 5% brightness
  tdongle.led(hsv, brightness);

  // set the background color to black
  lv_obj_t *bg = lv_obj_create(lv_screen_active());
  lv_obj_set_size(bg, tdongle.lcd_width(), tdongle.lcd_height());
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
    static constexpr int max_circles = 10;
    if (circles.size() >= max_circles) {
      // lock the lvgl mutex
      std::lock_guard<std::mutex> lock(lvgl_mutex);
      clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      static constexpr int middle_x = tdongle.lcd_width() / 2;
      static constexpr int middle_y = tdongle.lcd_height() / 2;
      static constexpr int radius = 30;
      float angle = circles.size() * 2.0f * M_PI / max_circles;
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

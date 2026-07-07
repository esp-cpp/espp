#include <chrono>
#include <cmath>
#include <numbers>
#include <stdlib.h>

#include "t-dongle-s3.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

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

  // create the GUI: builds the UI (label, circle layer) and starts the task
  // which updates LVGL. All of its public methods are thread-safe, so the
  // button callback and the main loop below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Drawing circles\nto the screen.");

  // initialize the button, which we'll use to cycle the rotation of the
  // display
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      gui.next_rotation();
    }
  };
  tdongle.initialize_button(on_button_pressed);

  // set the LED to be red
  espp::Hsv hsv(150.0f, 1.0f, 1.0f);
  float brightness = 5.0f; // 5% brightness
  tdongle.led(hsv, brightness);

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
    // if the maximum number of circles are on the screen, clear them
    if (gui.circle_count() >= Gui::MAX_CIRCLES) {
      gui.clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = tdongle.rotated_display_width() / 2;
      int middle_y = tdongle.rotated_display_height() / 2;
      static constexpr int radius = 30;
      float angle = gui.circle_count() * 2.0f * std::numbers::pi_v<float> / Gui::MAX_CIRCLES;
      int x = middle_x + radius * std::cos(angle);
      int y = middle_y + radius * std::sin(angle);
      gui.draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [t-dongle-s3 example]
}

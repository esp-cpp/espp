#include <chrono>
#include <stdlib.h>

#include "ws-s3-geek.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

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
  // set the pixel buffer to be 50 lines high
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

  // create the GUI: builds the UI (label, circle layer) and starts the task
  // which updates LVGL. All of its public methods are thread-safe, so the
  // button callback and the main loop below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Drawing circles\nto the screen.");

  // initialize the button, which we'll use to cycle the rotation of the display
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      logger.info("Button pressed, rotating the display");
      gui.next_rotation();
    }
  };
  board.initialize_button(on_button_pressed);

  // set the display brightness to be 75%
  board.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // if the maximum number of circles are on the screen, clear them
    if (gui.circle_count() >= Gui::MAX_CIRCLES) {
      gui.clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = board.rotated_display_width() / 2;
      int middle_y = board.rotated_display_height() / 2;
      static constexpr int radius = 30;
      float angle = gui.circle_count() * 2.0f * M_PI / Gui::MAX_CIRCLES;
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);
      gui.draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [ws-s3-geek example]
}

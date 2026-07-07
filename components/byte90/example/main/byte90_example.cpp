#include <chrono>
#include <cmath>
#include <numbers>
#include <stdlib.h>

#include "byte90.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Byte90 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [byte90 example]
  espp::Byte90 &byte90 = espp::Byte90::get();
  byte90.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the accelerometer
  if (!byte90.initialize_accelerometer()) {
    logger.error("Failed to initialize accelerometer!");
  }
  // initialize the LCD
  if (!byte90.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = byte90.lcd_width() * 50;
  // initialize the LVGL display for the Byte90
  if (!byte90.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // create the GUI: builds the UI (label and circle layer) and starts the
  // task which updates LVGL. All of its public methods are thread-safe, so
  // the button callback and main loop below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Drawing circles\nto the screen.");

  // initialize the button; each press increments the display brightness by
  // 10% (wrapping back around after 100%) and cycles the display rotation
  logger.info("Initializing the button");
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      // increment the brightness by 10%, looping back to 0% after 100%
      auto brightness = byte90.brightness();
      brightness = std::fmod(brightness + 10.0f, 100.0f);
      logger.info("Setting brightness to {:.0f}%", brightness);
      byte90.brightness(brightness);
      // cycle the rotation of the display; the Gui resizes / re-aligns the
      // UI to match
      gui.next_rotation();
    }
  };
  byte90.initialize_button(on_button_pressed);

  // set the display brightness to be 75%
  byte90.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // if there are 10 circles on the screen, clear them
    if (gui.visible_circle_count() >= Gui::MAX_CIRCLES) {
      gui.clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = byte90.rotated_display_width() / 2;
      int middle_y = byte90.rotated_display_height() / 2;
      static constexpr int radius = 30;
      float angle =
          gui.visible_circle_count() * 2.0f * std::numbers::pi_v<float> / Gui::MAX_CIRCLES;
      int x = middle_x + radius * std::cos(angle);
      int y = middle_y + radius * std::sin(angle);
      gui.draw_circle(x, y, 5);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [byte90 example]
}

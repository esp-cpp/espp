#include <chrono>
#include <stdlib.h>

#include "button.hpp"
#include "wrover-kit.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

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

  // create the GUI: builds the UI (label, circle layer) and starts the task
  // which updates LVGL. All of its public methods are thread-safe, so the
  // button callback and drawing loop below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Drawing circles to the screen.");

  // initialize the button, which we'll use to cycle the rotation of the display
  espp::Button button(espp::Button::Config{
      .name = "Boot Button",
      .interrupt_config =
          espp::Interrupt::PinConfig{.gpio_num = GPIO_NUM_0,
                                     .callback =
                                         [](const auto &event) {
                                           if (event.active) {
                                             gui.next_rotation();
                                           }
                                         },
                                     .active_level = espp::Interrupt::ActiveLevel::LOW,
                                     .interrupt_type = espp::Interrupt::Type::ANY_EDGE,
                                     .pullup_enabled = false,
                                     .pulldown_enabled = false},
  });

  // set the display brightness to be 75%
  wrover.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // if there are 10 circles on the screen, clear them
    if (gui.circle_count() >= Gui::MAX_CIRCLES) {
      gui.clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = wrover.rotated_display_width() / 2;
      int middle_y = wrover.rotated_display_height() / 2;
      static constexpr int radius = 50;
      float angle = gui.circle_count() * 2.0f * M_PI / Gui::MAX_CIRCLES;
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);
      gui.draw_circle(x, y, 10);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [wrover-kit example]
}

#include <chrono>
#include <stdlib.h>

#include "matouch-rotary-display.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Matouch-Rotary-Display Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [matouch-rotary-display example]
  espp::MatouchRotaryDisplay &mt_display = espp::MatouchRotaryDisplay::get();
  mt_display.set_log_level(espp::Logger::Verbosity::INFO);

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
  // initialize the rotary encoder
  if (!mt_display.initialize_encoder()) {
    logger.error("Failed to initialize rotary encoder!");
    return;
  }

  // create the GUI: builds the UI (label, buttons, circle layer) and starts
  // the task which updates LVGL. All of its public methods are thread-safe,
  // so the button / touch callbacks and the main loop below can call them
  // directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  static const std::string instructions =
      fmt::format("Touch the screen!\nPress the button or the {} button to clear circles.\nPress "
                  "the {} button to rotate the display.",
                  LV_SYMBOL_TRASH, LV_SYMBOL_REFRESH);
  gui.set_label_text(instructions);

  // initialize the hardware button (pressing the encoder); releasing it
  // clears the circles
  auto on_button_pressed = [&](const auto &event) {
    if (event.active) {
      logger.info("Button pressed!");
    } else {
      logger.info("Button released!");
      // clear the screen
      gui.clear_circles();
    }
  };
  if (!mt_display.initialize_button(on_button_pressed)) {
    logger.error("Failed to initialize button!");
    return;
  }

  // initialize the touchpad; each touch draws a circle
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
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  if (!mt_display.initialize_touch(on_touch)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // set the display brightness to be 75%
  mt_display.brightness(75.0f);

  while (true) {
    auto start = esp_timer_get_time();
    // get the encoder count and update the label with it
    int encoder_count = mt_display.encoder_value();
    gui.set_label_text(fmt::format("{}\nEncoder: {}", instructions, encoder_count));
    // sleep for the remaining time
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(50ms - std::chrono::microseconds(elapsed));
  }
  //! [matouch-rotary-display example]
}

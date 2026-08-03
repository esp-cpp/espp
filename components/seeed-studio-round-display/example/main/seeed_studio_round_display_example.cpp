#include <chrono>
#include <stdlib.h>

#include "seeed-studio-round-display.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger(
      {.tag = "Seeed Studio Round Display Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [seeed studio round display example]
#if CONFIG_EXAMPLE_HARDWARE_XIAOS3
  logger.info("Using XiaoS3 hardware configuration");
  espp::SsRoundDisplay::set_pin_config(espp::SsRoundDisplay::XiaoS3Config);
#elif CONFIG_EXAMPLE_HARDWARE_QTPYS3
  logger.info("Using QtpyS3 hardware configuration");
  espp::SsRoundDisplay::set_pin_config(espp::SsRoundDisplay::QtpyS3Config);
#else
#error "Please select a hardware configuration"
#endif
  espp::SsRoundDisplay &round_display = espp::SsRoundDisplay::get();

  // initialize the LCD
  if (!round_display.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be 50 lines high
  static constexpr size_t pixel_buffer_size = round_display.lcd_width() * 50;
  // initialize the LVGL display for the seeed-studio-round-display
  if (!round_display.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // create the GUI: builds the UI (label, buttons, circle layer) and starts
  // the task which updates LVGL. All of its public methods are thread-safe,
  // so the touch callback below can call them directly.
  static Gui gui({.log_level = espp::Logger::Verbosity::INFO});
  gui.set_label_text("Touch the screen!");

  // initialize the touchpad after the GUI exists so touch events can update
  // it immediately; each touch draws a circle, while the touchpad button
  // clears the circles
  auto touch_callback = [&](const auto &touch) {
    // NOTE: since we're directly using the touchpad data, and not using the
    // TouchpadInput + LVGL, we'll need to ensure the touchpad data is
    // converted into proper screen coordinates instead of simply using the
    // raw values.
    static auto previous_touchpad_data = round_display.touchpad_convert(touch);
    auto touchpad_data = round_display.touchpad_convert(touch);
    if (touchpad_data != previous_touchpad_data) {
      logger.info("Touch: {}", touchpad_data);
      previous_touchpad_data = touchpad_data;
      // if the button is pressed, clear the circles
      if (touchpad_data.btn_state) {
        gui.clear_circles();
      }
      // if there is a touch point, draw a circle
      if (touchpad_data.num_touch_points > 0) {
        gui.draw_circle(touchpad_data.x, touchpad_data.y, 10);
      }
    }
  };
  if (!round_display.initialize_touch(touch_callback)) {
    logger.error("Failed to initialize touchpad!");
    return;
  }

  // set the display brightness to be 75%
  round_display.brightness(75.0f);

  // loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
  //! [seeed studio round display example]
}

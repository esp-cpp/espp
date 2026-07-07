#include <chrono>

#include "ws-s3-lcd-1-47.hpp"

#include "gui.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "WsS3Lcd147 Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting example!");

  //! [ws-s3-lcd-1-47 example]
  using Bsp = espp::WsS3Lcd147;
  Bsp &board = Bsp::get();
  board.set_log_level(espp::Logger::Verbosity::INFO);

  // initialize the LCD
  if (!board.initialize_lcd()) {
    logger.error("Failed to initialize LCD!");
    return;
  }
  // set the pixel buffer to be a full screen buffer
  static constexpr size_t pixel_buffer_size = board.lcd_width() * 50;
  // initialize the LVGL display for the WsS3Lcd147
  if (!board.initialize_display(pixel_buffer_size)) {
    logger.error("Failed to initialize display!");
    return;
  }

  // initialize the uSD card
  using SdCardConfig = Bsp::SdCardConfig;
  SdCardConfig sdcard_config{};
  if (!board.initialize_sdcard(sdcard_config)) {
    logger.warn("Failed to initialize SD card, continuing without it.");
  }

  // initialize the LED
  if (!board.initialize_led()) {
    logger.error("Failed to initialize LED!");
    return;
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
      gui.next_rotation();
    }
  };
  board.initialize_button(on_button_pressed);

  // set the display brightness to be 75%
  board.brightness(75.0f);

  // make a task to cycle the color of the onboard LED
  espp::Task led_task({.callback = [](std::mutex &m, std::condition_variable &cv) -> bool {
                         static auto &board = Bsp::get();
                         auto start = std::chrono::steady_clock::now();
                         // set the LED color
                         static float hue = 0;
                         hue = std::fmod(hue + 1.0f, 360.0f);
                         board.led(espp::Hsv(hue, 100.0f, 100.0f));
                         // wait for 100ms
                         std::unique_lock<std::mutex> lock(m);
                         cv.wait_until(lock, start + 10ms);
                         return false;
                       },
                       .task_config = {
                           .name = "led_task",
                           .core_id = 1,
                       }});
  led_task.start();

  while (true) {
    auto start = esp_timer_get_time();
    // if the maximum number of circles are on the screen, clear them
    if (gui.visible_circle_count() >= Gui::max_circles()) {
      gui.clear_circles();
    } else {
      // draw a circle of circles on the screen (just draw the next circle)
      int middle_x = board.rotated_display_width() / 2;
      int middle_y = board.rotated_display_height() / 2;
      static constexpr int radius = 50;
      float angle = gui.visible_circle_count() * 2.0f * M_PI / Gui::max_circles();
      int x = middle_x + radius * cos(angle);
      int y = middle_y + radius * sin(angle);
      gui.draw_circle(x, y, 10);
    }
    auto end = esp_timer_get_time();
    auto elapsed = end - start;
    std::this_thread::sleep_for(100ms - std::chrono::microseconds(elapsed));
  }
  //! [ws-s3-lcd-1-47 example]
}

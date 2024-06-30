#include <chrono>
#include <vector>

#include "esp-box.hpp"

using namespace std::chrono_literals;

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

  while (true) {
    std::this_thread::sleep_for(100ms);
    if (box.update_touch()) {
      logger.info("Touch: {}", box.touchpad_data());
    }
  }
  //! [esp box example]
}

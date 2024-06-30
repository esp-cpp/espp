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
  box.initialize_touch();
  box.initialize_sound();

  while (true) {
    std::this_thread::sleep_for(100ms);
    if (box.update_touch()) {
      logger.info("Touch: {}", box.touchpad_data());
    }
  }
  //! [esp box example]
}

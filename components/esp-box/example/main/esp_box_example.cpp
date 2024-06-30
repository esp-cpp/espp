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
  //! [esp box example]
}

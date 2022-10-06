#include <chrono>
#include <functional>
#include <thread>

#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // we get access to fmt since we include logger.hpp
  fmt::print("Stating logger example!\n");
  auto logger_fn = []() {
    // create logger
    //! [Logger example]
    auto logger = espp::Logger({
        .tag = "Thread 2",
        .level = espp::Logger::Level::INFO
      });
    size_t loop_iteration{0};
    while (true) {
      // log - note: debug shouldn't be shown!
      logger.debug("some debug info: {}", loop_iteration);
      logger.info("some info: {}", loop_iteration);
      logger.warn("some warning: {}", loop_iteration);
      logger.error("some error: {}", loop_iteration);
      // update loop variables
      loop_iteration++;
      // sleep
      std::this_thread::sleep_for(300ms);
    }
    //! [Logger example]
  };
  auto logger_thread = std::thread(logger_fn);

  // create another logger
  auto logger = espp::Logger({
      .tag = "Thread 1",
      .level = espp::Logger::Level::DEBUG
    });
  size_t loop_iteration{0};
  while (true) {
    // log
    logger.debug("some debug info: {}", loop_iteration);
    logger.info("some info: {}", loop_iteration);
    logger.warn("some warning: {}", loop_iteration);
    logger.error("some error: {}", loop_iteration);
    // update loop variables
    loop_iteration++;
    // sleep
    std::this_thread::sleep_for(1s);
  }
}

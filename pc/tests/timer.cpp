#include "timer.hpp"

using namespace std::chrono_literals;

int main() {

  static auto start = std::chrono::system_clock::now();
  static auto elapsed = []() -> float {
    return std::chrono::duration<float>(std::chrono::system_clock::now() - start).count();
  };

  espp::Logger logger({.tag = "Timer Test", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting timer test");

  espp::Timer timer({.name = "Timer",
                     .period = 0.100s,
                     .delay = 0.0s,
                     .callback = [&]() -> bool {
                       static int iterations = 0;
                       logger.info("[{:.3f}] timer fired {}", elapsed(), iterations++);
                       // don't want to stop the timer
                       return false;
                     },
                     .log_level = espp::Logger::Verbosity::DEBUG});

  std::this_thread::sleep_for(10s);

  logger.info("Stopping timer test");

  return 0;
}

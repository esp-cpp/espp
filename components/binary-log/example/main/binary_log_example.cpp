#include <chrono>
#include <functional>
#include <thread>

#include "binary-log.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    //! [Binary Log example]
    float num_seconds_to_run = 10.0f;
    // create loggers
    binary_log::binary_log log("log.out");
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;

      BINARY_LOG(log, "elapsed: {:.3f}s", elapsed);
      BINARY_LOG(log, "remaining: {:.3f}s", remaining);

      if (remaining < 0) {
        BINARY_LOG(log, "You overstayed your welcome by {:.03}s!", -remaining);
      }
      std::this_thread::sleep_for(500ms);
    }
    //! [Logger example]
  }
}

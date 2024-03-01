#include <sdkconfig.h>

#include <chrono>
#include <vector>

#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "logger.hpp"
#include "timer.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Timer example", .level = espp::Logger::Verbosity::DEBUG});
  size_t num_seconds_to_run = 3;
  static auto start = std::chrono::high_resolution_clock::now();

  static auto elapsed = []() {
    auto now = std::chrono::high_resolution_clock::now();
    return std::chrono::duration<float>(now - start).count();
  };

#if CONFIG_PM_ENABLE
  logger.info("Enabling power management...");
  // Configure dynamic frequency scaling:
  // maximum and minimum frequencies are set in sdkconfig,
  // automatic light sleep is enabled if tickless idle support is enabled.
  esp_pm_config_t pm_config = {.max_freq_mhz = 240, .min_freq_mhz = 40, .light_sleep_enable = true};
  // if we have BT enabled, then the power mode won't work well....
  ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
#endif

  // basic timer example
  {
    logger.info("[{:.3f}] Starting basic timer example", elapsed());
    //! [timer example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 500ms,
                              .callback = timer_fn,
                              .log_level = espp::Logger::Verbosity::DEBUG});
    //! [timer example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // timer with delay example
  {
    logger.info("[{:.3f}] Starting timer with delay example", elapsed());
    //! [timer delay example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer =
        espp::Timer({.name = "Timer 1",
                     .period = 500ms,
                     .delay = 500ms,
                     .callback = timer_fn,
                     .auto_start = false, // don't start the timer automatically, we'll call start()
                     .log_level = espp::Logger::Verbosity::DEBUG});
    timer.start();
    std::this_thread::sleep_for(2s);
    logger.info("[{:.3f}] Cancelling timer for 2 seconds", elapsed());
    timer.cancel();
    std::this_thread::sleep_for(2s);
    timer.start();
    std::this_thread::sleep_for(2s);
    logger.info("[{:.3f}] Cancelling timer for 2 seconds", elapsed());
    timer.cancel();
    std::this_thread::sleep_for(2s);
    timer.start(1s);
    //! [timer delay example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // oneshot timer example
  {
    logger.info("[{:.3f}] Starting oneshot timer example", elapsed());
    //! [timer oneshot example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 0ms, // one shot timer
                              .delay = 500ms,
                              .callback = timer_fn,
                              .log_level = espp::Logger::Verbosity::DEBUG});
    //! [timer oneshot example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // timer cancel itself example
  {
    logger.info("[{:.3f}] Starting timer cancel itself example", elapsed());
    //! [timer cancel itself example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // cancel the timer after 3 iterations
      if (iterations == 3) {
        fmt::print("[{:.3f}] auto-cancelling timer\n", elapsed());
        return true;
      }
      return false;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 500ms,
                              .callback = timer_fn,
                              .stack_size_bytes = 6192,
                              .log_level = espp::Logger::Verbosity::DEBUG});
    //! [timer cancel itself example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // oneshot timer example cancel itself then start it again with delay
  {
    logger.info("[{:.3f}] Starting oneshot timer cancel itself then restart example", elapsed());
    //! [timer oneshot restart example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we want to stop, so return true
      return true;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 0ms, // one shot timer
                              .delay = 500ms,
                              .callback = timer_fn,
                              .stack_size_bytes = 4096,
                              .log_level = espp::Logger::Verbosity::DEBUG});
    std::this_thread::sleep_for(2s);
    timer.cancel();  // it will have already been cancelled by here, but this should be harmless
    timer.start(1s); // restart the timer with a 1 second delay
    //! [timer oneshot restart example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // timer example update period while running
  {
    logger.info("[{:.3f}] Starting timer update period while running example", elapsed());
    //! [timer update period example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 500ms,
                              .callback = timer_fn,
                              .stack_size_bytes = 4096,
                              .log_level = espp::Logger::Verbosity::DEBUG});
    std::this_thread::sleep_for(2s);
    logger.info("[{:.3f}] Updating period to 100ms", elapsed());
    timer.set_period(100ms);
    //! [timer update period example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  logger.info("Test ran for {:.03f} seconds", elapsed());

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

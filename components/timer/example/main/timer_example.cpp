#include <sdkconfig.h>

#include <chrono>
#include <vector>

#include <esp_pm.h>
#include <esp_sleep.h>
#include <esp_system.h>

#include "high_resolution_timer.hpp"
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
    logger.info("Starting basic timer example");
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

  // timer watchdog example
  {
    logger.info("Starting timer watchdog example");
    //! [timer watchdog example]
    static constexpr bool panic_on_watchdog_timeout = false;
    espp::Task::configure_task_watchdog(300ms, panic_on_watchdog_timeout);
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
    timer.start_watchdog(); // start the watchdog timer for this timer
    std::this_thread::sleep_for(500ms);
    std::error_code ec;
    std::string watchdog_info = espp::Task::get_watchdog_info(ec);
    if (ec) {
      fmt::print("Error getting watchdog info: {}\n", ec.message());
    } else if (!watchdog_info.empty()) {
      fmt::print("Watchdog info: {}\n", watchdog_info);
    } else {
      fmt::print("No watchdog info available\n");
    }
    // NOTE: the timer and the watchdog will both automatically get stopped when
    // the task goes out of scope and is destroyed.
    //! [timer watchdog example]
  }

  // timer with delay example
  {
    logger.info("Starting timer with delay example");
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
    logger.info("Cancelling timer for 2 seconds");
    timer.cancel();
    std::this_thread::sleep_for(2s);
    timer.start();
    std::this_thread::sleep_for(2s);
    logger.info("Cancelling timer for 2 seconds");
    timer.cancel();
    std::this_thread::sleep_for(2s);
    timer.start(1s);
    //! [timer delay example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // oneshot timer example
  {
    logger.info("Starting oneshot timer example");
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
    logger.info("Starting timer cancel itself example");
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
    logger.info("Starting oneshot timer cancel itself then restart example");
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
    logger.info("Starting timer update period while running example");
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
    logger.info("Updating period to 100ms");
    timer.set_period(100ms);
    //! [timer update period example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // timer example using advanced config
  {
    logger.info("Starting timer example using advanced config");
    //! [timer advanced config example]
    auto timer_fn = []() {
      static size_t iterations{0};
      fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer = espp::Timer({.period = 500ms,
                              .callback = timer_fn,
                              .task_config =
                                  {
                                      .name = "Advanced Config Timer",
                                      .stack_size_bytes = 4096,
                                      .priority = 10,
                                      .core_id = 1,
                                  },
                              .log_level = espp::Logger::Verbosity::DEBUG});
    //! [timer advanced config example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // high resolution timer example
  {
    logger.info("Starting high resolution timer example");
    //! [high resolution timer example]
    logger.set_rate_limit(100ms);
    auto timer_fn = [&]() {
      static size_t iterations{0};
      iterations++;
      logger.info_rate_limited("High resolution timer callback: {}", iterations);
      // we don't want to stop, so return false
      return false;
    };
    auto high_resolution_timer =
        espp::HighResolutionTimer({.name = "High Resolution Timer",
                                   .callback = timer_fn,
                                   .log_level = espp::Logger::Verbosity::DEBUG});
    uint64_t period_us = 100;
    bool started = high_resolution_timer.start(period_us);
    logger.info("High resolution timer started: {}", started);

    std::this_thread::sleep_for(500ms);
    logger.info("Updating period to 100ms");
    period_us = 1000 * 100;
    high_resolution_timer.set_period(period_us);
    logger.info("Periodic timer period: {}us", high_resolution_timer.get_period());

    // NOTE: only if CONFIG_ESP_TIMER_PROFILING is enabled will this show more
    //       than address, period and alarm.
    esp_timer_dump(stdout); // dump timer stats

    std::this_thread::sleep_for(500ms);
    logger.info("High resolution timer is running: {}", high_resolution_timer.is_running());
    logger.info("Stopping timer");
    high_resolution_timer.stop();

    std::this_thread::sleep_for(500ms);
    logger.info("Starting oneshot to expire in 100ms");
    started = high_resolution_timer.oneshot(period_us);
    logger.info("Oneshot timer started: {}", started);
    logger.info("Oneshot timer expiry: {}us", high_resolution_timer.get_period());

    //! [high resolution timer example]

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }

  // high resolution timer watchdog example
  {
    logger.info("Starting high resolution timer watchdog example");
    //! [high resolution timer watchdog example]
    logger.set_rate_limit(100ms);
    auto timer_fn = [&]() {
      static size_t iterations{0};
      iterations++;
      logger.info_rate_limited("High resolution timer callback: {}", iterations);
      // we don't want to stop, so return false
      return false;
    };
    auto high_resolution_timer =
        espp::HighResolutionTimer({.name = "High Resolution Timer 1",
                                   .callback = timer_fn,
                                   .log_level = espp::Logger::Verbosity::DEBUG});
    uint64_t period_us = 100;
    bool started = high_resolution_timer.start(period_us);
    logger.info("High resolution timer 1 started: {}", started);

    // make another HighResolutionTimer
    auto timer2_fn = [&]() {
      // sleep here to ensure watchdog triggers
      std::this_thread::sleep_for(350ms);
      // we don't want to stop, so return false
      return false;
    };
    auto high_resolution_timer2 =
        espp::HighResolutionTimer({.name = "High Resolution Timer 2",
                                   .callback = timer2_fn,
                                   .log_level = espp::Logger::Verbosity::DEBUG});

    // configure the task watchdog
    static constexpr bool panic_on_watchdog_timeout = false;
    espp::Task::configure_task_watchdog(300ms, panic_on_watchdog_timeout);

    // start the watchdog timer for this timer
    high_resolution_timer2.start_watchdog();

    // ensure we can run the watchdog on a oneshot timer which is started after
    // we start the watchdog
    period_us = 1000 * 100;
    started = high_resolution_timer2.oneshot(period_us);
    logger.info("High resolution timer 2 started: {}", started);

    std::this_thread::sleep_for(400ms);

    std::error_code ec;
    std::string watchdog_info = espp::Task::get_watchdog_info(ec);
    if (ec) {
      fmt::print("Error getting watchdog info: {}\n", ec.message());
    } else if (!watchdog_info.empty()) {
      fmt::print("Watchdog info: {}\n", watchdog_info);
    } else {
      fmt::print("No watchdog info available\n");
    }

    // now stop the watchdog timer
    high_resolution_timer2.stop_watchdog();

    // delay some more so we can see the watchdog timer has stopped
    std::this_thread::sleep_for(500ms);

    //! [high resolution timer watchdog example]
  }

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

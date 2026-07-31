#include <sdkconfig.h>

#include <atomic>
#include <chrono>
#include <string>
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

  // timer periodicity testing, with different durations
  {
    logger.info("Starting timer periodicity testing example");
    size_t iterations{0};
    auto timer_fn = [&iterations]() {
      if (iterations % 50 == 0) {
        fmt::print("[{:.3f}] #iterations = {}\n", elapsed(), iterations);
        std::this_thread::sleep_for(12ms); // simulate a long callback
      }
      iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto timer = espp::Timer({.name = "Timer 1",
                              .period = 10ms,
                              .callback = timer_fn,
                              .log_level = espp::Logger::Verbosity::WARN});
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
                              .log_level = espp::Logger::Verbosity::INFO});
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
                     .log_level = espp::Logger::Verbosity::INFO});
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
                              .log_level = espp::Logger::Verbosity::INFO});
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
                              .log_level = espp::Logger::Verbosity::INFO});
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
                              .log_level = espp::Logger::Verbosity::INFO});
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
                              .log_level = espp::Logger::Verbosity::INFO});
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
                                      .stack_size_bytes = 4 * 1024,
                                      .priority = 10,
                                      .core_id = 1,
                                  },
                              .log_level = espp::Logger::Verbosity::INFO});
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
                                   .log_level = espp::Logger::Verbosity::INFO});
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
                                   .log_level = espp::Logger::Verbosity::INFO});
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
                                   .log_level = espp::Logger::Verbosity::INFO});

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

  // ===========================================================================
  // Timer test suite
  //
  // A set of self-checking tests that run at the end of the example and print
  // PASS/FAIL for each, then an overall PASS/FAIL summary for the suite. These
  // exercise the timer's periodicity (fixed-rate scheduling / no drift), delay,
  // one-shot behavior, start()/is_running()/cancel(), self-cancel, and input
  // validation. Timings use generous tolerances so the suite is not flaky.
  // ===========================================================================
  {
    using namespace std::chrono;
    logger.info("");
    logger.info("======== Running Timer test suite ========");
    int passed = 0;
    int failed = 0;
    auto check = [&](const std::string &name, bool condition) {
      fmt::print("  [{}] {}\n", condition ? "PASS" : "FAIL", name);
      if (condition) {
        ++passed;
      } else {
        ++failed;
      }
    };

    // 1) Periodicity: a 20 ms periodic timer should fire ~50x/s and stay on a
    //    fixed schedule (the k-th callback lands near k*period, i.e. no
    //    cumulative drift).
    {
      std::atomic<int> count{0};
      std::atomic<float> last_fire{0.0f};
      auto t0 = steady_clock::now();
      auto timer = espp::Timer({.name = "test-periodicity",
                                .period = 20ms,
                                .callback =
                                    [&count, &last_fire, t0]() {
                                      last_fire = duration<float>(steady_clock::now() - t0).count();
                                      ++count;
                                      return false;
                                    },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(1s);
      timer.cancel();
      const int n = count.load();
      // first fires immediately, then every 20 ms -> ~51 in 1 s
      check("periodic timer fires ~50 times in 1 s (20 ms period)", n >= 45 && n <= 56);
      // the last callback should land near (n-1)*20 ms if there is no drift
      const float expected_last = (n - 1) * 0.020f;
      const float actual_last = last_fire.load();
      check("periodic timer stays on schedule (no cumulative drift)",
            n >= 2 && actual_last > expected_last - 0.030f && actual_last < expected_last + 0.030f);
    }

    // 2) A callback that overruns the period should run back-to-back (bounded by
    //    the callback duration), not stall or spiral.
    {
      std::atomic<int> count{0};
      auto timer = espp::Timer({.name = "test-overrun",
                                .period = 20ms,
                                .callback =
                                    [&count]() {
                                      ++count;
                                      std::this_thread::sleep_for(30ms); // longer than the period
                                      return false;
                                    },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(1s);
      timer.cancel();
      const int n = count.load();
      // ~30 ms per callback -> ~33 in 1 s
      auto msg = fmt::format(
          "Timer with a long (overrunning) callback runs continuously 27 <= {} <= 40", n);
      check(msg, n >= 27 && n <= 40);
    }

    // 3) Delay: the first callback fires at ~the configured delay, not before.
    {
      std::atomic<int> count{0};
      std::atomic<float> first_fire{-1.0f};
      auto t0 = steady_clock::now();
      auto timer = espp::Timer({.name = "test-delay",
                                .period = 50ms,
                                .delay = 300ms,
                                .callback =
                                    [&count, &first_fire, t0]() {
                                      if (count.fetch_add(1) == 0) {
                                        first_fire =
                                            duration<float>(steady_clock::now() - t0).count();
                                      }
                                      return false;
                                    },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(200ms); // still within the 300 ms delay
      const bool none_before_delay = (count.load() == 0);
      std::this_thread::sleep_for(400ms); // total 600 ms, past the delay
      timer.cancel();
      const float ff = first_fire.load();
      check("delayed timer does not fire before the delay", none_before_delay);
      check("delayed timer first fires at ~the delay", ff > 0.25f && ff < 0.40f);
    }

    // 4) One-shot (period 0) fires exactly once.
    {
      std::atomic<int> count{0};
      auto timer = espp::Timer({.name = "test-oneshot",
                                .period = 0ms,
                                .delay = 100ms,
                                .callback =
                                    [&count]() {
                                      ++count;
                                      return false;
                                    },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(400ms);
      check("one-shot timer (period 0) fires exactly once", count.load() == 1);
    }

    // 5) start() / is_running() / cancel().
    {
      std::atomic<int> count{0};
      auto timer = espp::Timer({.name = "test-startstop",
                                .period = 50ms,
                                .callback =
                                    [&count]() {
                                      ++count;
                                      return false;
                                    },
                                .auto_start = false,
                                .log_level = espp::Logger::Verbosity::WARN});
      check("timer is not running before start()", !timer.is_running());
      const bool started = timer.start();
      check("start() returns true and the timer is running", started && timer.is_running());
      check("start() on an already-running timer returns true", timer.start());
      std::this_thread::sleep_for(200ms);
      timer.cancel();
      const int after_cancel = count.load();
      std::this_thread::sleep_for(150ms);
      check("cancel() stops the timer (no more callbacks)",
            !timer.is_running() && count.load() == after_cancel);
    }

    // 6) A callback that returns true stops the timer.
    {
      std::atomic<int> count{0};
      auto timer = espp::Timer({.name = "test-selfstop",
                                .period = 50ms,
                                .callback = [&count]() { return ++count >= 3; },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(400ms);
      check("callback returning true stops the timer", count.load() == 3 && !timer.is_running());
    }

    // 7) Input validation: a negative period is clamped (behaves as one-shot).
    {
      std::atomic<int> count{0};
      auto timer = espp::Timer({.name = "test-negative-period",
                                .period = -50ms,
                                .callback =
                                    [&count]() {
                                      ++count;
                                      return false;
                                    },
                                .log_level = espp::Logger::Verbosity::WARN});
      std::this_thread::sleep_for(300ms);
      timer.cancel();
      check("negative period is clamped (runs once, not repeatedly)", count.load() == 1);
    }

    // ---- summary ----
    fmt::print("\n");
    logger.info("======== Timer test suite: {}/{} passed ========", passed, passed + failed);
    if (failed == 0) {
      logger.info("TIMER TESTS RESULT: PASS ({} tests)", passed);
    } else {
      logger.error("TIMER TESTS RESULT: FAIL ({} passed, {} failed)", passed, failed);
    }
  }

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

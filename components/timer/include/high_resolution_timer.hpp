#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <esp_timer.h>

#include "base_component.hpp"
#include "task.hpp"

namespace espp {
/// High resolution timer
/// This class provides a high resolution timer using the ESP-IDF esp_timer
/// API. The timer can be started in oneshot or periodic mode. The timer
/// period can be set and queried. The timer can be started, stopped, and
/// restarted. The timer can be queried to check if it is running.
///
/// @note Since this uses the esp-timer API, you cannot set different stack
/// sizes for differnt timers like you can with espp::Timer and espp::Task.
/// Instead, you may need to adjust the stack size via the menuconfig
/// `CONFIG_ESP_TIMER_TASK_STACK_SIZE`.
///
/// \section high_resolution_timer_ex1 High Resolution Timer Example
/// \snippet timer_example.cpp high resolution timer example
/// \section high_resolution_timer_ex2 High Resolution Timer Watchdog Example
/// \snippet timer_example.cpp high resolution timer watchdog example
class HighResolutionTimer : public espp::BaseComponent {
public:
  /// Callback type for the timer
  typedef std::function<void()> Callback;

  /// Configuration of the timer
  struct Config {
    std::string name;                  ///< Name of the timer
    Callback callback{nullptr};        ///< Callback to be called when the timer expires
    bool skip_unhandled_events{false}; ///< Skip unhandled events. If true, will skip unhandled
                                       ///< events in light sleep for periodic timers.
    esp_timer_dispatch_t dispatch_method =
        ESP_TIMER_TASK; ///< Dispatch method, TIMER_TASK or TIMER_ISR.
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level
  };

  /// Constructor
  /// @param config Configuration of the timer
  explicit HighResolutionTimer(const Config &config);

  /// Destructor
  ~HighResolutionTimer();

  /// Start the timer
  /// @param period_us Period of the timer in microseconds, or timeout if
  ///        oneshot is true
  /// @param oneshot True if the timer should be oneshot, false if periodic
  /// @return True if the timer was started successfully, false otherwise
  bool start(uint64_t period_us = 0, bool oneshot = false);

  /// Start the timer in oneshot mode
  /// @param timeout_us Timeout of the timer in microseconds
  /// @return True if the timer was started successfully, false otherwise
  bool oneshot(uint64_t timeout_us = 0);

  /// Start the timer in periodic mode
  /// @param period_us Period of the timer in microseconds
  /// @return True if the timer was started successfully, false otherwise
  bool periodic(uint64_t period_us = 0);

  /// Stop the timer
  void stop();

  /// Start the watchdog timer
  /// @return True if the watchdog timer was started successfully, false
  ///         otherwise
  /// @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
  ///       enabled in the menuconfig. Default is y (enabled).
  bool start_watchdog();

  /// Stop the watchdog timer
  /// @return True if the watchdog timer was stopped successfully, false
  ///         otherwise
  /// @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
  ///       enabled in the menuconfig. Default is y (enabled).
  bool stop_watchdog();

  /// Check if the timer is running
  /// @return True if the timer is running, false otherwise
  bool is_running() const;

  /// Is the timer oneshot?
  /// @return True if the timer is a oneshot timer, false if it is perioic.
  bool is_oneshot() const;

  /// Is the timer periodic?
  /// @return True if the timer is a periodic timer, false if it is oneshot.
  bool is_periodic() const;

  /// Set the period of the timer in microseconds
  /// @param period_us Period of the timer in microseconds
  /// @note This function will restart the timer if it is running
  void set_period(uint64_t period_us);

  /// Get the period of the timer in microseconds
  /// @return Period of the timer in microseconds
  /// @note This function will return 0 if the timer is not running
  /// @note This function will return the period of the timer, not the
  ///       remaining time
  uint64_t get_period();

protected:
  static void timer_callback(void *arg);

  void handle_timer_callback();

  esp_task_wdt_user_handle_t wdt_handle_{nullptr};
  bool skip_unhandled_events_{false};
  esp_timer_dispatch_t dispatch_method_{ESP_TIMER_TASK};
  esp_timer_handle_t timer_handle_{nullptr};
  std::atomic<bool> oneshot_{false};
  Callback callback_{nullptr};
};
} // namespace espp

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
class HighResolutionTimer : public espp::BaseComponent {
public:
  /// Callback type for the timer
  typedef std::function<void()> Callback;

  /// Configuration of the timer
  struct Config {
    std::string name;           ///< Name of the timer
    Callback callback{nullptr}; ///< Callback to be called when the timer expires
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Log level
  };

  /// Constructor
  /// @param config Configuration of the timer
  explicit HighResolutionTimer(const Config &config)
      : BaseComponent(config.name, config.log_level)
      , callback_(config.callback) {
    using namespace std::chrono_literals;
    // set a default logger rate limit (can always be set later by caller)
    set_log_rate_limit(100ms);
  }

  /// Destructor
  ~HighResolutionTimer() { stop(); }

  /// Start the timer
  /// @param period_us Period of the timer in microseconds, or timeout if
  ///        oneshot is true
  /// @param oneshot True if the timer should be oneshot, false if periodic
  /// @return True if the timer was started successfully, false otherwise
  bool start(uint64_t period_us = 0, bool oneshot = false) {
    if (is_running()) {
      stop();
    }

    // store whether the timer is oneshot or periodic
    oneshot_ = oneshot;

    esp_timer_create_args_t timer_args;
    timer_args.callback = timer_callback;
    timer_args.arg = this;
    timer_args.dispatch_method = ESP_TIMER_TASK; // TIMER_TASK or TIMER_ISR
    timer_args.name = get_name().c_str();

    esp_err_t err = ESP_OK;

    err = esp_timer_create(&timer_args, &timer_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to create timer: {}", esp_err_to_name(err));
      return false;
    }

    if (oneshot) {
      err = esp_timer_start_once(timer_handle_, period_us);
    } else {
      err = esp_timer_start_periodic(timer_handle_, period_us);
    }

    if (err != ESP_OK) {
      logger_.error("Failed to start timer: {}", esp_err_to_name(err));
      return false;
    }
    return true;
  }

  /// Start the timer in oneshot mode
  /// @param timeout_us Timeout of the timer in microseconds
  /// @return True if the timer was started successfully, false otherwise
  bool oneshot(uint64_t timeout_us = 0) { return start(timeout_us, true); }

  /// Start the timer in periodic mode
  /// @param period_us Period of the timer in microseconds
  /// @return True if the timer was started successfully, false otherwise
  bool periodic(uint64_t period_us = 0) { return start(period_us, false); }

  /// Stop the timer
  void stop() {
    if (is_running()) {
      esp_timer_stop(timer_handle_);
    }
    if (timer_handle_) {
      esp_timer_delete(timer_handle_);
      timer_handle_ = nullptr;
    }
  }

  /// Check if the timer is running
  /// @return True if the timer is running, false otherwise
  bool is_running() const { return timer_handle_ && esp_timer_is_active(timer_handle_); }

  /// Is the timer oneshot?
  /// @return True if the timer is a oneshot timer, false if it is perioic.
  bool is_oneshot() const { return oneshot_; };

  /// Is the timer periodic?
  /// @return True if the timer is a periodic timer, false if it is oneshot.
  bool is_periodic() const { return !oneshot_; };

  /// Set the period of the timer in microseconds
  /// @param period_us Period of the timer in microseconds
  /// @note This function will restart the timer if it is running
  void set_period(uint64_t period_us) {
    esp_err_t err = ESP_OK;
    if (is_running()) {
      err = esp_timer_restart(timer_handle_, period_us);
    } else if (timer_handle_) {
      if (oneshot_) {
        err = esp_timer_start_once(timer_handle_, period_us);
      } else {
        err = esp_timer_start_periodic(timer_handle_, period_us);
      }
    } else {
      logger_.error("Cannot set period for timer, start() has not been called!");
    }
    if (err != ESP_OK) {
      logger_.error("Failed to set timer period: {}", esp_err_to_name(err));
    }
  }

  /// Get the period of the timer in microseconds
  /// @return Period of the timer in microseconds
  /// @note This function will return 0 if the timer is not running
  /// @note This function will return the period of the timer, not the
  ///       remaining time
  uint64_t get_period() {
    uint64_t period_us = 0;
    esp_err_t err = ESP_OK;
    if (oneshot_) {
      err = esp_timer_get_expiry_time(timer_handle_, &period_us);
    } else {
      err = esp_timer_get_period(timer_handle_, &period_us);
    }
    if (err != ESP_OK) {
      logger_.error("Failed to get timer period: {}", esp_err_to_name(err));
      return 0;
    }
    logger_.debug("Timer period for {} timer: {} us", oneshot_.load() ? "oneshot" : "periodic",
                  period_us);
    return period_us;
  }

protected:
  static void timer_callback(void *arg) {
    auto timer = static_cast<HighResolutionTimer *>(arg);
    if (!timer) {
      return;
    }
    timer->handle_timer_callback();
  }

  void handle_timer_callback() {
    logger_.debug_rate_limited("Timer expired, calling callback");
    if (callback_) {
      callback_();
    }
  }

  esp_timer_handle_t timer_handle_{nullptr};
  std::atomic<bool> oneshot_{false};
  Callback callback_{nullptr};
};
} // namespace espp

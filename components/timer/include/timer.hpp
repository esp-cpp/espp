#pragma once

#include <chrono>
#include <functional>
#include <string>

#include "base_component.hpp"
#include "task.hpp"

namespace espp {
/// @brief A timer that can be used to schedule tasks to run at a later time.
/// @details A timer can be used to schedule a task to run at a later time.
///          The timer will run in the background and will call the task when
///          the time is up. The timer can be canceled at any time. A timer
///          can be configured to run once or to repeat.
///
///          The timer uses a task to run in the background. The task will
///          sleep until the timer is ready to run. When the timer is ready to
///          run, the task will call the callback function. The callback
///          function can return true to cancel the timer or false to keep the
///          timer running. If the timer is configured to repeat, then the
///          callback function will be called again after the period has
///          elapsed. If the timer is configured to run once, then the
///          callback function will only be called once.
///
///          The timer can be configured to start automatically when it is
///          constructed. If the timer is not configured to start
///          automatically, then the timer can be started by calling start().
///          The timer can be canceled at any time by calling cancel().
///
/// @note The timer uses a task to run in the background, so the timer
///       callback function will be called in the context of the task. The
///       timer callback function should not block for a long time because it
///       will block the task. If the timer callback function blocks for a
///       long time, then the timer will not be able to keep up with the
///       period.
///
/// \section timer_ex1 Timer Example 1
/// \snippet timer_example.cpp timer example
/// \section timer_ex2 Timer Delay Example
/// \snippet timer_example.cpp timer delay example
/// \section timer_ex3 Oneshot Timer Example
/// \snippet timer_example.cpp timer oneshot example
/// \section timer_ex4 Timer Cancel Itself Example
/// \snippet timer_example.cpp timer cancel itself example
/// \section timer_ex5 Oneshot Timer Cancel Itself Then Start again with Delay Example
/// \snippet timer_example.cpp timer oneshot restart example
/// \section timer_ex6 Timer Update Period Example
/// \snippet timer_example.cpp timer update period example
class Timer : public BaseComponent {
public:
  typedef std::function<bool()>
      callback_fn; ///< The callback function type. Return true to cancel the timer.

  /// @brief The configuration for the timer.
  struct Config {
    std::string_view name; ///< The name of the timer.
    std::chrono::duration<float>
        period; ///< The period of the timer. If 0, the timer callback will only be called once.
    std::chrono::duration<float> delay{
        0}; ///< The delay before the first execution of the timer callback after start() is called.
    callback_fn callback;  ///< The callback function to call when the timer expires.
    bool auto_start{true}; ///< If true, the timer will start automatically when constructed.
    size_t stack_size_bytes{4096}; ///< The stack size of the task that runs the timer.
    size_t priority{0}; ///< Priority of the timer, 0 is lowest priority on ESP / FreeRTOS.
    int core_id{-1};    ///< Core ID of the timer, -1 means it is not pinned to any core.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the timer.
  };

  /// @brief Construct a new Timer object
  /// @param config The configuration for the timer.
  explicit Timer(const Config &config)
      : BaseComponent(config.name, config.log_level)
      , period_(std::chrono::duration_cast<std::chrono::microseconds>(config.period))
      , delay_(std::chrono::duration_cast<std::chrono::microseconds>(config.delay))
      , callback_(config.callback) {
    // set the logger rate limit
    logger_.set_rate_limit(std::chrono::milliseconds(100));
    // make the task
    task_ = espp::Task::make_unique({
        .name = std::string(config.name) + "_task",
        .callback = std::bind(&Timer::timer_callback_fn, this, std::placeholders::_1,
                              std::placeholders::_2),
        .stack_size_bytes = config.stack_size_bytes,
        .priority = config.priority,
        .core_id = config.core_id,
        .log_level = config.log_level,
    });
    period_float = std::chrono::duration<float>(period_).count();
    delay_float = std::chrono::duration<float>(delay_).count();
    if (config.auto_start) {
      start();
    }
  }

  /// @brief Destroy the Timer object
  /// @details Cancels the timer if it is running.
  ~Timer() { cancel(); }

  /// @brief Start the timer.
  /// @details Starts the timer. Does nothing if the timer is already running.
  void start() {
    logger_.info("starting with period {:.3f} s and delay {:.3f} s", period_float, delay_float);
    running_ = true;
    // start the task
    task_->start();
  }

  /// @brief Start the timer with a delay.
  /// @details Starts the timer with a delay. If the timer is already running,
  ///          this will cancel the timer and start it again with the new
  ///          delay. If the timer is not running, this will start the timer
  ///          with the delay. Overwrites any previous delay that might have
  ///          been set.
  /// @param delay The delay before the first execution of the timer callback.
  void start(std::chrono::duration<float> delay) {
    if (delay.count() < 0) {
      logger_.warn("delay cannot be negative, not starting");
      return;
    }
    if (is_running()) {
      logger_.info("restarting with delay {:.3f} s", delay.count());
      cancel();
    }
    delay_ = std::chrono::duration_cast<std::chrono::microseconds>(delay);
    delay_float = std::chrono::duration<float>(delay_).count();
    start();
  }

  /// @brief Stop the timer, same as cancel().
  /// @details Stops the timer, same as cancel().
  void stop() { cancel(); }

  /// @brief Cancel the timer.
  /// @details Cancels the timer.
  void cancel() {
    logger_.info("canceling");
    running_ = false;
    // cancel the task
    task_->stop();
  }

  /// @brief Set the period of the timer.
  /// @details Sets the period of the timer.
  /// @param period The period of the timer.
  /// @note If the period is 0, the timer will run once.
  /// @note If the period is negative, the period will not be set / updated.
  /// @note If the timer is running, the period will be updated after the
  ///       current period has elapsed.
  void set_period(std::chrono::duration<float> period) {
    if (period.count() < 0) {
      logger_.warn("period cannot be negative, not setting");
      return;
    }
    period_ = std::chrono::duration_cast<std::chrono::microseconds>(period);
    period_float = std::chrono::duration<float>(period_).count();
    logger_.info("setting period to {:.3f} s", period_float);
  }

  /// @brief Check if the timer is running.
  /// @details Checks if the timer is running.
  /// @return true if the timer is running, false otherwise.
  bool is_running() const { return running_ && task_->is_running(); }

protected:
  bool timer_callback_fn(std::mutex &m, std::condition_variable &cv) {
    logger_.debug("callback entered");
    if (!running_) {
      // stop the timer, the timer was canceled
      logger_.debug("timer was canceled, stopping");
      return true;
    }
    if (!callback_) {
      // stop the timer, the callback is null
      logger_.debug("callback is null, stopping");
      running_ = false;
      return true;
    }
    // initial delay, if any - this is only used the first time the timer
    // runs
    if (delay_float > 0) {
      auto start_time = std::chrono::steady_clock::now();
      logger_.debug("waiting for delay {:.3f} s", delay_float);
      std::unique_lock<std::mutex> lock(m);
      cv.wait_until(lock, start_time + delay_);
      if (!running_) {
        logger_.debug("delay canceled, stopping");
        return true;
      }
      // now set the delay to 0
      delay_ = std::chrono::microseconds(0);
      delay_float = 0;
    }
    // now run the callback
    auto start_time = std::chrono::steady_clock::now();
    logger_.debug("running callback");
    bool requested_stop = callback_();
    if (requested_stop || period_float <= 0) {
      // stop the timer if requested or if the period is <= 0
      logger_.debug("callback requested stop or period is <= 0, stopping");
      running_ = false;
      return true;
    }
    auto end = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(end - start_time).count();
    if (elapsed > period_float) {
      // if the callback took longer than the period, then we should just
      // return and run the callback again immediately
      logger_.warn_rate_limited("callback took longer ({:.3f} s) than period ({:.3f} s)", elapsed,
                                period_float);
      return false;
    }
    // now wait for the period (taking into account the time it took to run
    // the callback)
    {
      std::unique_lock<std::mutex> lock(m);
      cv.wait_until(lock, start_time + period_);
      // Note: we don't care about cv_retval here because we are going to
      // return from the function anyway. If the timer was canceled, then
      // the task will be stopped and the callback will not be called again.
    }
    // keep the timer running
    return false;
  }

  std::chrono::microseconds period_{0}; ///< The period of the timer. If 0, the timer will run once.
  std::chrono::microseconds delay_{0};  ///< The delay before the timer starts.
  std::atomic<bool> running_{false};    ///< True if the timer is running, false otherwise.
  float period_float;
  float delay_float;
  callback_fn callback_;             ///< The callback function to call when the timer expires.
  std::unique_ptr<espp::Task> task_; ///< The task that runs the timer.
};
} // namespace espp

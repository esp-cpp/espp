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
/// \section timer_ex2 Timer Watchdog Example
/// \snippet timer_example.cpp timer watchdog example
/// \section timer_ex3 Timer Delay Example
/// \snippet timer_example.cpp timer delay example
/// \section timer_ex4 Oneshot Timer Example
/// \snippet timer_example.cpp timer oneshot example
/// \section timer_ex5 Timer Cancel Itself Example
/// \snippet timer_example.cpp timer cancel itself example
/// \section timer_ex6 Oneshot Timer Cancel Itself Then Start again with Delay Example
/// \snippet timer_example.cpp timer oneshot restart example
/// \section timer_ex7 Timer Update Period Example
/// \snippet timer_example.cpp timer update period example
/// \section timer_ex8 Timer AdvancedConfig Example
/// \snippet timer_example.cpp timer advanced config example
class Timer : public BaseComponent {
public:
  typedef std::function<bool()>
      callback_fn; ///< The callback function type. Return true to cancel the timer.

  /// @brief The configuration for the timer.
  struct Config {
    std::string_view name; ///< The name of the timer.
    std::chrono::duration<float>
        period; ///< The period of the timer. If 0, the timer callback will only be called once.
    std::chrono::duration<float> delay = std::chrono::duration<float>(
        0); ///< The delay before the first execution of the timer callback after start() is called.
    espp::Timer::callback_fn callback; ///< The callback function to call when the timer expires.
    bool auto_start{true}; ///< If true, the timer will start automatically when constructed.
    size_t stack_size_bytes{4096}; ///< The stack size of the task that runs the timer.
    size_t priority{0}; ///< Priority of the timer, 0 is lowest priority on ESP / FreeRTOS.
    int core_id{-1};    ///< Core ID of the timer, -1 means it is not pinned to any core.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the timer.
  };

  /// @brief Advanced configuration for the timer.
  struct AdvancedConfig {
    std::chrono::duration<float>
        period; ///< The period of the timer. If 0, the timer callback will only be called once.
    std::chrono::duration<float> delay = std::chrono::duration<float>(
        0); ///< The delay before the first execution of the timer callback after start() is called.
    espp::Timer::callback_fn callback; ///< The callback function to call when the timer expires.
    bool auto_start{true}; ///< If true, the timer will start automatically when constructed.
    espp::Task::BaseConfig task_config; ///< The task configuration for the timer.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the timer.
  };

  /// @brief Construct a new Timer object
  /// @param config The configuration for the timer.
  explicit Timer(const Config &config);

  /// @brief Construct a new Timer object
  /// @param config The configuration for the timer.
  explicit Timer(const AdvancedConfig &config);

  /// @brief Destroy the Timer object
  /// @details Cancels the timer if it is running.
  ~Timer();

  /// @brief Start the timer.
  /// @details Starts the timer. Does nothing if the timer is already running.
  void start();

  /// @brief Start the timer with a delay.
  /// @details Starts the timer with a delay. If the timer is already running,
  ///          this will cancel the timer and start it again with the new
  ///          delay. If the timer is not running, this will start the timer
  ///          with the delay. Overwrites any previous delay that might have
  ///          been set.
  /// @param delay The delay before the first execution of the timer callback.
  void start(const std::chrono::duration<float> &delay);

  /// @brief Stop the timer, same as cancel().
  /// @details Stops the timer, same as cancel().
  void stop();

  /// @brief Cancel the timer.
  /// @details Cancels the timer.
  void cancel();

#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
  /// @brief Start the task watchdog for the timer.
  /// @return true if the watchdog was started, false otherwise.
  /// @note This function is only available on ESP
  /// @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
  ///       enabled in the menuconfig. Default is y (enabled).
  /// @see stop_watchdog()
  /// @see Task::start_watchdog()
  /// @see Task::stop_watchdog()
  bool start_watchdog();

  /// @brief Stop the task watchdog for the timer.
  /// @return true if the watchdog was stopped, false otherwise.
  /// @note This function is only available on ESP
  /// @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
  ///       enabled in the menuconfig. Default is y (enabled).
  /// @see start_watchdog()
  /// @see Task::start_watchdog()
  /// @see Task::stop_watchdog()
  bool stop_watchdog();
#endif // ESP_PLATFORM || _DOXYGEN_

  /// @brief Set the period of the timer.
  /// @details Sets the period of the timer.
  /// @param period The period of the timer.
  /// @note If the period is 0, the timer will run once.
  /// @note If the period is negative, the period will not be set / updated.
  /// @note If the timer is running, the period will be updated after the
  ///       current period has elapsed.
  void set_period(const std::chrono::duration<float> &period);

  /// @brief Check if the timer is running.
  /// @details Checks if the timer is running.
  /// @return true if the timer is running, false otherwise.
  bool is_running() const;

protected:
  bool timer_callback_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  std::chrono::microseconds period_{0}; ///< The period of the timer. If 0, the timer will run once.
  std::chrono::microseconds delay_{0};  ///< The delay before the timer starts.
  std::atomic<bool> running_{false};    ///< True if the timer is running, false otherwise.
  float period_float;
  float delay_float;
  callback_fn callback_;             ///< The callback function to call when the timer expires.
  std::unique_ptr<espp::Task> task_; ///< The task that runs the timer.
};
} // namespace espp

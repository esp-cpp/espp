#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

#if defined(ESP_PLATFORM)
#include <esp_pthread.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif

#include "base_component.hpp"

namespace espp {

/**
 * @brief Task provides an abstraction over std::thread which optionally
 * includes memory / priority configuration on ESP systems. It allows users to
 * easily stop the task, and will automatically stop itself if destroyed.
 *
 * There is also a utility function which can be used to get the info for the
 * task of the current context, or for a provided Task object.
 *
 * There is also a helper function to run a lambda on a specific core, which can
 * be used to run a specific function on a specific core, as you might want to
 * do when registering an interrupt driver on a specific core.
 *
 * \section task_ex1 Basic Task Example
 * \snippet task_example.cpp Task example
 * \section task_ex2 Task Watchdog Example
 * \snippet task_example.cpp task watchdog example
 * \section task_ex3 Many Task Example
 * \snippet task_example.cpp ManyTask example
 * \section task_ex4 Long Running Task Example
 * \snippet task_example.cpp LongRunningTask example
 * \section task_ex5 Task Info Example
 * \snippet task_example.cpp Task Info example
 * \section task_ex6 Task Request Stop Example
 * \snippet task_example.cpp Task Request Stop example
 *
 * \section run_on_core_ex1 Run on Core Example
 * \snippet task_example.cpp run on core example
 */
class Task : public espp::BaseComponent {
public:
#if defined(ESP_PLATFORM)
  typedef void *task_id_t;
#else
  typedef std::thread::id task_id_t;
#endif

  /**
   * @brief Task callback function signature.
   *
   * @note The callback is run repeatedly within the Task, therefore it MUST
   *      return, and also SHOULD have a sleep to give the processor over to
   *      other tasks. For this reason, the callback is provided a
   *      std::condition_variable (and associated mutex) which the callback
   *      can use when they need to wait. If the cv.wait_for / cv.wait_until
   *      return <a href="https://en.cppreference.com/w/cpp/thread/cv_status">
   *      std::cv_status::timeout</a>, no action is necessary, but if they
   *      return <a href="https://en.cppreference.com/w/cpp/thread/cv_status">
   *      std::cv_status::no_timeout</a>, then the function should return
   *      immediately since the task is being stopped (optionally performing
   *      any task-specific tear-down).
   *
   * @param m mutex associated with the condition variable (should be locked
   *          before calling cv.wait_for() or cv.wait_until())
   *
   * @param cv condition variable the callback can use to perform an
   *           interruptible wait.
   *
   * @return Whether or not the callback's thread / task should stop - True to
   *         stop, false to continue running.
   */
  typedef std::function<bool(std::mutex &m, std::condition_variable &cv)> callback_fn;

  /**
   * @brief Simple callback function signature.
   *
   * @ note The callback is run repeatedly within the Task, therefore it MUST
   *        return, and also SHOULD have a sleep to give the processor over to
   *        other tasks.
   * @return True to stop the task, false to continue running.
   */
  typedef std::function<bool()> simple_callback_fn;

  /**
   * @brief Base configuration struct for the Task.
   * @note This is designed to be used as a configuration struct in other classes
   *       that may have a Task as a member.
   */
  struct BaseConfig {
    std::string name;              /**< Name of the task */
    size_t stack_size_bytes{4096}; /**< Stack Size (B) allocated to the task. */
    size_t priority{0}; /**< Priority of the task, 0 is lowest priority on ESP / FreeRTOS.  */
    int core_id{-1};    /**< Core ID of the task, -1 means it is not pinned to any core.  */
  };

  /**
   * @brief Configuration struct for the Task.
   * @note This is the recommended way to configure the Task, and allows you to
   *       use the condition variable and mutex from the task to wait_for and
   *       wait_until.
   * @note This is an older configuration struct, and is kept for backwards
   *       compatibility. It is recommended to use the AdvancedConfig struct
   *       instead.
   */
  struct Config {
    std::string name;                 /**< Name of the task */
    espp::Task::callback_fn callback; /**< Callback function  */
    size_t stack_size_bytes{4096};    /**< Stack Size (B) allocated to the task. */
    size_t priority{0}; /**< Priority of the task, 0 is lowest priority on ESP / FreeRTOS.  */
    int core_id{-1};    /**< Core ID of the task, -1 means it is not pinned to any core.  */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Simple configuration struct for the Task.
   * @note This is useful for when you don't need to use the condition variable
   *       or mutex in the callback.
   */
  struct SimpleConfig {
    espp::Task::simple_callback_fn callback; /**< Callback function  */
    espp::Task::BaseConfig task_config;      /**< Base configuration for the task. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Advanced configuration struct for the Task.
   * @note This is the recommended way to configure the Task, and allows you to
   *       use the condition variable and mutex from the task to wait_for and
   *       wait_until.
   */
  struct AdvancedConfig {
    espp::Task::callback_fn callback;   /**< Callback function  */
    espp::Task::BaseConfig task_config; /**< Base configuration for the task. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Construct a new Task object using the Config struct.
   * @param config Config struct to initialize the Task with.
   */
  explicit Task(const espp::Task::Config &config);

  /**
   *  @brief Construct a new Task object using the SimpleConfig struct.
   *  @param config SimpleConfig struct to initialize the Task with.
   */
  explicit Task(const espp::Task::SimpleConfig &config);

  /**
   *  @brief Construct a new Task object using the AdvancedConfig struct.
   *  @param config AdvancedConfig struct to initialize the Task with.
   */
  explicit Task(const espp::Task::AdvancedConfig &config);

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config Config struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const espp::Task::Config &config);

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config SimpleConfig struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const espp::Task::SimpleConfig &config);

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config AdvancedConfig struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const espp::Task::AdvancedConfig &config);

  /**
   * @brief Destroy the task, stopping it if it was started.
   */
  ~Task();

  /**
   * @brief Start executing the task.
   *
   * @return true if the task started, false if it was already started.
   */
  bool start();

  /**
   * @brief Stop the task execution.
   * @details This will request the task to stop, notify the condition variable,
   *          and (if this calling context is not the task context) join the
   *          thread.
   * @return true if the task stopped, false if it was not started / already
   *         stopped.
   */
  bool stop();

  /**
   * @brief Has the task been started or not?
   *
   * @return true if the task is started / running, false otherwise.
   */
  bool is_started() const;

  /**
   * @brief Is the task running?
   *
   * @return true if the task is running, false otherwise.
   */
  bool is_running() const;

#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
  /**
   * @brief Start the task watchdog for this task.
   * @return true if the watchdog was started, false otherwise.
   */
  bool start_watchdog();

  /**
   * @brief Stop the task watchdog for this task.
   * @return true if the watchdog was stopped, false otherwise.
   */
  bool stop_watchdog();

  /**
   * @brief Initialize/Configure the task watchdog for the current task.
   * @param timeout_ms Timeout in milliseconds for the watchdog.
   * @param panic_on_timeout Whether or not to panic on timeout.
   * @return true if the watchdog was initialized, false otherwise.
   * @note This function will not monitor the idle tasks.
   * @note If the watchdog has not been configured, then this function will call
   *       `esp_task_wdt_init`, otherwise it will then call
   *       `esp_task_wdt_reconfigure`.
   */
  static bool configure_task_watchdog(uint32_t timeout_ms, bool panic_on_timeout = true);

  /**
   * @brief Initialize/Configure the task watchdog for the current task.
   * @param timeout The timeout for the watchdog.
   * @param panic_on_timeout Whether or not to panic on timeout.
   * @return true if the watchdog was initialized, false otherwise.
   * @note This function will not monitor the idle tasks.
   * @note If the watchdog has not been configured, then this function will call
   *       `esp_task_wdt_init`, otherwise it will then call
   *       `esp_task_wdt_reconfigure`.
   */
  static bool configure_task_watchdog(const std::chrono::milliseconds &timeout,
                                      bool panic_on_timeout = true);

  /**
   * @brief Retrieve the info about tasks / users which triggered the task
   *        watchdog timeout.
   * @param ec Error code to set if there was an error retrieving the info.
   * @return std::string containing the task watchdog info, or an empty string
   *         if there was no timeout or there was an error retrieving the info.
   * @note This function will only return info for tasks which are still
   *       registered with the watchdog. If you call this after you have called
   *       stop_watchdog() for a task, then even if the task triggered the
   *       watchdog timeout you will not get that information.
   */
  static std::string get_watchdog_info(std::error_code &ec);

  /**
   * @brief Get the info (as a string) for the task of the current context.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
   * @note This function is only available on ESP32
   */
  static std::string get_info();

  /**
   * @brief Get the info (as a string) for the provided \p task.
   * @param task Reference to the task for which you want the information.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
   * @note This function is only available on ESP32
   */
  static std::string get_info(const Task &task);
#endif

  /**
   * @brief Get the ID for this Task's thread / task context.
   * @return ID for this Task's thread / task context.
   */
  task_id_t get_id() const {
#if defined(ESP_PLATFORM)
    return task_handle_;
#else
    return thread_.get_id();
#endif
  }

  /**
   * @brief Get the ID for the current thread / task context.
   * @return ID for the current thread / task context.
   */
  static task_id_t get_current_id() {
#if defined(ESP_PLATFORM)
    return static_cast<task_id_t>(xTaskGetCurrentTaskHandle());
#else
    return std::this_thread::get_id();
#endif
  }

protected:
  /**
   * @brief Function that is run in the task thread.
   * @details Will call the callback function repeatedly until the task is
   *          stopped or until the callback function returns true, indicating
   *          that the task should stop.
   */
  void thread_function();

  /**
   * @brief Notify the task to stop and join the thread.
   */
  void notify_and_join();

  std::string name_;     ///< Name of the task, used in logs and task monitoring.
  callback_fn callback_; ///< Callback function for the task. Called within Task::thread_function().
  simple_callback_fn simple_callback_; ///< Simple callback function for the task. Called within
                                       ///< Task::thread_function().
  BaseConfig config_;                  ///< Configuration for the task.

  std::atomic<bool> started_{false};
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::mutex thread_mutex_;
  std::thread thread_;
#if defined(ESP_PLATFORM)
  std::atomic<bool> watchdog_started_{false};
  task_id_t task_handle_;
#endif
};
} // namespace espp

#include "task_formatters.hpp"

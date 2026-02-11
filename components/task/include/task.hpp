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
 * \section task_ex5 Long Running Task Notified Example (Recommended)
 * \snippet task_example.cpp LongRunningTaskNotified example
 * \section task_ex6 Task Info Example
 * \snippet task_example.cpp Task Info example
 * \section task_ex7 Task Request Stop Example
 * \snippet task_example.cpp Task Request Stop example
 *
 * \section run_on_core_ex1 Run on Core Example
 * \snippet task_example.cpp run on core example
 * \section run_on_core_ex2 Run on Core (Non-Blocking) Example
 * \snippet task_example.cpp run on core nonblocking example
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
   *      std::condition_variable (and associated mutex) which the callback can
   *      use when they need to wait. If the cv.wait_for / cv.wait_until return
   *      false, no action is necessary (as the timeout properly occurred / the
   *      task was not notified). If they return true, then the callback
   *      function should return immediately since the task is being stopped
   *      (optionally performing any task-specific tear-down).
   *
   * @param m mutex associated with the condition variable (should be locked
   *          before calling cv.wait_for() or cv.wait_until())
   *
   * @param cv condition variable the callback can use to perform an
   *           interruptible wait. Use the notified parameter as the predicate
   *           for the wait to ensure spurious wake-ups are properly ignored.
   *
   * @param notified Whether or not the task has been notified to stop. This is
   *        a reference to the task's notified member, and is intended to be
   *        used with the condition variable parameter cv when waiting to avoid
   *        spurious wakeups. The task function is responsible for clearing this
   *        flag after each wait. Do not release the lock on the mutex m until
   *        the notified flag has been checked and cleared.
   *
   * @return Whether or not the callback's thread / task should stop - True to
   *         stop, false to continue running.
   */
  typedef std::function<bool(std::mutex &m, std::condition_variable &cv, bool &notified)>
      callback_m_cv_notified_fn;

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
   * @warning This is an older callback function signature, and is kept for
   *       backwards compatibility. It is recommended to use the newer callback
   *       signature (callback_m_cv_notified_fn) which includes the notified
   *       parameter, enabling the task callback function to wait on the
   *       condition variable and ignore spurious wakeups.
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
  typedef std::function<bool(std::mutex &m, std::condition_variable &cv)> callback_m_cv_fn;

  /**
   * @brief Simple callback function signature.
   *
   * @note The callback is run repeatedly within the Task, therefore it MUST
   *       return, and also SHOULD have a sleep to give the processor over to
   *       other tasks.
   *
   * @return True to stop the task, false to continue running.
   */
  typedef std::function<bool()> callback_no_params_fn;

  /**
   * @brief Variant of the callback function for the task.
   * @note This is a std::variant of the different callback function signatures
   *      that can be used with the Task. This allows the Task to be configured
   *      with a callback function that takes no parameters, a callback function
   *      that takes a mutex and condition variable, or a callback function that
   *      takes a mutex, condition variable, and a bool reference to the notified
   *      flag.
   *
   *      This is primarily used to enable simpler API upgrades in the future
   *      and maximize backwards compatibility.
   */
  typedef std::variant<callback_m_cv_notified_fn, callback_m_cv_fn, callback_no_params_fn>
      callback_variant;

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
   *        Can be initialized with any of the supported callback function
   *        signatures.
   */
  struct Config {
    espp::Task::callback_variant callback; /**< Callback function  */
    espp::Task::BaseConfig task_config;    /**< Base configuration for the task. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Construct a new Task object using the Config struct.
   * @param config Config struct to initialize the Task with.
   */
  explicit Task(const espp::Task::Config &config);

  /**
   * @brief Construct a new Task object using the callback function and
   *        BaseConfig struct.
   * @details This is a convenience constructor that allows you to create a Task
   *          with a callback function and a BaseConfig struct, without having
   *          to create a Config struct.
   *
   * @param callback Callback function for the task.
   * @param config BaseConfig struct to initialize the Task with.
   * @param log_level Log verbosity for the task.
   */
  explicit Task(const espp::Task::callback_no_params_fn &callback,
                const espp::Task::BaseConfig &config,
                espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : Task(Config{callback, config, log_level}) {}

  /**
   * @brief Construct a new Task object using the callback function and
   *        BaseConfig struct.
   * @details This is a convenience constructor that allows you to create a Task
   *          with a callback function that takes a mutex and condition variable,
   *          and a BaseConfig struct, without having to create a Config struct.
   *
   * @param callback Callback function for the task.
   * @param config BaseConfig struct to initialize the Task with.
   * @param log_level Log verbosity for the task.
   */
  explicit Task(const espp::Task::callback_m_cv_fn &callback, const espp::Task::BaseConfig &config,
                espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : Task(Config{callback, config, log_level}) {}

  /**
   * @brief Construct a new Task object using the callback function and
   *        BaseConfig struct.
   * @details This is a convenience constructor that allows you to create a Task
   *          with a callback function that takes a mutex, condition variable,
   *          and a bool reference to the notified flag, and a BaseConfig struct,
   *          without having to create a Config struct.
   *
   * @param callback Callback function for the task.
   * @param config BaseConfig struct to initialize the Task with.
   * @param log_level Log verbosity for the task.
   */
  explicit Task(const espp::Task::callback_m_cv_notified_fn &callback,
                const espp::Task::BaseConfig &config,
                espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN)
      : Task(Config{callback, config, log_level}) {}

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config Config struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const espp::Task::Config &config);

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
   * @note This function is only available on ESP
   * @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
   *       enabled in the menuconfig. Default is y (enabled).
   */
  bool start_watchdog();

  /**
   * @brief Stop the task watchdog for this task.
   * @return true if the watchdog was stopped, false otherwise.
   * @note This function is only available on ESP
   * @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
   *       enabled in the menuconfig. Default is y (enabled).
   */
  bool stop_watchdog();

  /**
   * @brief Initialize/Configure the task watchdog for the current task.
   * @param timeout_ms Timeout in milliseconds for the watchdog.
   * @param panic_on_timeout Whether or not to panic on timeout.
   * @return true if the watchdog was initialized, false otherwise.
   * @note This function is only available on ESP
   * @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
   *       enabled in the menuconfig. Default is y (enabled).
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
   * @note This function is only available on ESP
   * @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
   *       enabled in the menuconfig. Default is y (enabled).
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
   * @note This function is only available on ESP
   * @note This function will do nothing unless CONFIG_ESP_TASK_WDT_EN is
   *       enabled in the menuconfig. Default is y (enabled).
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
   * @note This function is only available on ESP
   */
  static std::string get_info();

  /**
   * @brief Get the info (as a string) for the provided \p task.
   * @param task Reference to the task for which you want the information.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
   * @note This function is only available on ESP
   */
  static std::string get_info(const Task &task);

  /**
   * @brief Get the FreeRTOS task handle for the task of the current context.
   * @return TaskHandle_t FreeRTOS task handle
   * @note This function is only available on ESP
   * @note This function is designed to be used in conjunction with FreeRTOS
   *       APIs which require a TaskHandle_t.
   */
  static TaskHandle_t get_freertos_handle() { return xTaskGetCurrentTaskHandle(); }

  /**
   * @brief Get the FreeRTOS task handle for the provided task.
   * @param task Reference to the task for which you want the FreeRTOS handle.
   * @return TaskHandle_t FreeRTOS task handle
   * @warning This will only return a valid handle if the task is started.
   * @note This function is only available on ESP
   * @note This function is designed to be used in conjunction with FreeRTOS
   *       APIs which require a TaskHandle_t. If you're wanting to print or use
   *       the pointer in other ways, It's recommended to get the void* pointer
   *       from get_id() instead, which can be directly printed without needing
   *       fmt::ptr.
   */
  static TaskHandle_t get_freertos_handle(const Task &task) {
    return static_cast<TaskHandle_t>(task.get_id());
  }

  /**
   * @brief Get the priority for the task of the current context.
   * @return int Priority of the task
   * @note This function is only available on ESP
   */
  static int get_priority() { return uxTaskPriorityGet(nullptr); }

  /**
   * @brief Get the priority for the provided task.
   * @param task Reference to the task for which you want the priority.
   * @return int Priority of the task
   * @note This function is only available on ESP
   */
  static int get_priority(const Task &task) { return uxTaskPriorityGet(get_freertos_handle(task)); }

  /**
   * @brief Get the stack high water mark (minimum free stack space) for the
   *        task of the current context.
   * @return size_t Stack high water mark (bytes). This is the minimum number of
   *         bytes that have remained unallocated on the stack since the task
   *         started. Higher values indicate more free stack space, lower values
   *         indicate less free stack space.
   * @note This function is only available on ESP
   */
  static size_t get_high_water_mark() {
    return uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
  }

  /**
   * @brief Get the stack high water mark (minimum free stack space) for the
   *        provided task.
   * @param task Reference to the task for which you want the high water mark.
   * @return size_t Stack high water mark (bytes). This is the minimum number of
   *         bytes that have remained unallocated on the stack since the task
   *         started. Higher values indicate more free stack space, lower values
   *         indicate less free stack space.
   * @note This function is only available on ESP
   */
  static size_t get_high_water_mark(const Task &task) {
    return uxTaskGetStackHighWaterMark(get_freertos_handle(task)) * sizeof(StackType_t);
  }

  /**
   * @brief Get the core ID for the task of the current context.
   * @return int Core ID of the task. -1 if the task is not pinned to any
   *                core.
   * @note This function is only available on ESP
   */
  static int get_core_id() {
    auto core_id = xPortGetCoreID();
    if (core_id == tskNO_AFFINITY) {
      return -1;
    }
    return core_id;
  }

  /**
   * @brief Get the core ID for the provided task.
   * @param task Reference to the task for which you want the core ID.
   * @return int Core ID of the task. -1 if the task is not pinned to any
   *                core.
   * @note This function is only available on ESP
   */
  static int get_core_id(const Task &task) {
    auto core_id = xTaskGetCoreID(get_freertos_handle(task));
    if (core_id == tskNO_AFFINITY) {
      return -1;
    }
    return core_id;
  }

#endif // ESP_PLATFORM

  /**
   * @brief Get the ID for this Task's thread / task context.
   * @return ID for this Task's thread / task context.
   * @warning This will only return a valid id if the task is started.
   */
  task_id_t get_id() const {
#if defined(ESP_PLATFORM)
    return task_handle_;
#else
    return thread_.get_id();
#endif
  }

  /**
   * @brief Get the ID for the Task's thread / task context.
   * @param task Reference to the task for which you want the ID.
   * @return ID for this Task's thread / task context.
   * @warning This will only return a valid id if the task is started.
   */
  static task_id_t get_id(const Task &task) { return task.get_id(); }

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

  std::string name_;          ///< Name of the task, used in logs and task monitoring.
  callback_variant callback_; ///< Variant of the callback function for the task.
  BaseConfig config_;         ///< Configuration for the task.

  std::atomic<bool> started_{false};
  std::condition_variable cv_;
  bool notified_{false};
  std::mutex cv_m_;
  std::mutex thread_mutex_;
  std::thread thread_;
#if defined(ESP_PLATFORM)
  std::atomic<bool> watchdog_started_{false};
  task_id_t task_handle_{nullptr};
#endif
};
} // namespace espp

#include "task_formatters.hpp"

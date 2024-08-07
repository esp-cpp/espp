#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

#if defined(ESP_PLATFORM)
#include <esp_pthread.h>
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
 * \section task_ex2 Many Task Example
 * \snippet task_example.cpp ManyTask example
 * \section task_ex3 Long Running Task Example
 * \snippet task_example.cpp LongRunningTask example
 * \section task_ex4 Task Info Example
 * \snippet task_example.cpp Task Info example
 * \section task_ex5 Task Request Stop Example
 * \snippet task_example.cpp Task Request Stop example
 *
 * \section run_on_core_ex1 Run on Core Example
 * \snippet task_example.cpp run on core example
 */
class Task : public BaseComponent {
public:
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
    std::string name;                  /**< Name of the task */
    size_t stack_size_bytes{4 * 1024}; /**< Stack Size (B) allocated to the task. */
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
    std::string name;                  /**< Name of the task */
    callback_fn callback;              /**< Callback function  */
    size_t stack_size_bytes{4 * 1024}; /**< Stack Size (B) allocated to the task. */
    size_t priority{0}; /**< Priority of the task, 0 is lowest priority on ESP / FreeRTOS.  */
    int core_id{-1};    /**< Core ID of the task, -1 means it is not pinned to any core.  */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Simple configuration struct for the Task.
   * @note This is useful for when you don't need to use the condition variable
   *       or mutex in the callback.
   */
  struct SimpleConfig {
    simple_callback_fn callback;                          /**< Callback function  */
    BaseConfig task_config;                               /**< Base configuration for the task. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Advanced configuration struct for the Task.
   * @note This is the recommended way to configure the Task, and allows you to
   *       use the condition variable and mutex from the task to wait_for and
   *       wait_until.
   */
  struct AdvancedConfig {
    callback_fn callback;                                 /**< Callback function  */
    BaseConfig task_config;                               /**< Base configuration for the task. */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  /**
   * @brief Construct a new Task object using the Config struct.
   * @param config Config struct to initialize the Task with.
   */
  explicit Task(const Config &config)
      : BaseComponent(config.name, config.log_level)
      , name_(config.name)
      , callback_(config.callback)
      , config_({config.name, config.stack_size_bytes, config.priority, config.core_id}) {}

  /**
   *  @brief Construct a new Task object using the SimpleConfig struct.
   *  @param config SimpleConfig struct to initialize the Task with.
   */
  explicit Task(const SimpleConfig &config)
      : BaseComponent(config.task_config.name, config.log_level)
      , name_(config.task_config.name)
      , simple_callback_(config.callback)
      , config_(config.task_config) {}

  /**
   *  @brief Construct a new Task object using the AdvancedConfig struct.
   *  @param config AdvancedConfig struct to initialize the Task with.
   */
  explicit Task(const AdvancedConfig &config)
      : BaseComponent(config.task_config.name, config.log_level)
      , name_(config.task_config.name)
      , callback_(config.callback)
      , config_(config.task_config) {}

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config Config struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const Config &config) {
    return std::make_unique<Task>(config);
  }

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config SimpleConfig struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const SimpleConfig &config) {
    return std::make_unique<Task>(config);
  }

  /**
   * @brief Get a unique pointer to a new task created with \p config.
   *        Useful to not have to use templated std::make_unique (less typing).
   * @param config AdvancedConfig struct to initialize the Task with.
   * @return std::unique_ptr<Task> pointer to the newly created task.
   */
  static std::unique_ptr<Task> make_unique(const AdvancedConfig &config) {
    return std::make_unique<Task>(config);
  }

  /**
   * @brief Destroy the task, stopping it if it was started.
   */
  ~Task() {
    logger_.debug("Destroying task");
    // stop the task if it was started
    if (started_) {
      stop();
    }
    // ensure we stop the thread if it's still around
    if (thread_.joinable()) {
      thread_.join();
    }
    logger_.debug("Task destroyed");
  }

  /**
   * @brief Start executing the task.
   *
   * @return true if the task started, false if it was already started.
   */
  bool start() {
    logger_.debug("Starting task");
    if (started_) {
      logger_.warn("Task already started!");
      return false;
    }

#if defined(ESP_PLATFORM)
    auto thread_config = esp_pthread_get_default_config();
    thread_config.thread_name = name_.c_str();
    auto core_id = config_.core_id;
    if (core_id >= 0)
      thread_config.pin_to_core = core_id;
    if (core_id >= portNUM_PROCESSORS) {
      logger_.error("core_id ({}) is larger than portNUM_PROCESSORS ({}), cannot create Task '{}'",
                    core_id, portNUM_PROCESSORS, name_);
      return false;
    }
    thread_config.stack_size = config_.stack_size_bytes;
    thread_config.prio = config_.priority;
    // this will set the config for the next created thread
    auto err = esp_pthread_set_cfg(&thread_config);
    if (err == ESP_ERR_NO_MEM) {
      logger_.error("Out of memory, cannot create Task '{}'", name_);
      return false;
    }
    if (err == ESP_ERR_INVALID_ARG) {
      // see
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/pthread.html?highlight=esp_pthread_set_cfg#_CPPv419esp_pthread_set_cfgPK17esp_pthread_cfg_t
      logger_.error(
          "Configured stack size ({}) is less than PTHREAD_STACK_MIN ({}), cannot create Task '{}'",
          config_.stack_size_bytes, PTHREAD_STACK_MIN, name_);
      return false;
    }
#endif

    if (thread_.joinable()) {
      thread_.join();
    }

    // set the atomic so that when the thread starts it won't immediately
    // exit.
    started_ = true;
    // create and start the std::thread
    thread_ = std::thread(&Task::thread_function, this);

    logger_.debug("Task started");
    return true;
  }

  /**
   * @brief Stop the task execution, blocking until it stops.
   *
   * @return true if the task stopped, false if it was not started / already
   * stopped.
   */
  bool stop() {
    logger_.debug("Stopping task");
    if (started_) {
      started_ = false;
      cv_.notify_all();
      if (thread_.joinable()) {
        thread_.join();
      }
    }
    logger_.debug("Task stopped");
    return true;
  }

  /**
   * @brief Has the task been started or not?
   *
   * @return true if the task is started / running, false otherwise.
   */
  bool is_started() const { return started_; }

  /**
   * @brief Is the task running?
   *
   * @return true if the task is running, false otherwise.
   */
  bool is_running() const { return is_started(); }

#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
  /**
   * @brief Get the info (as a string) for the task of the current context.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
   * @note This function is only available on ESP32
   */
  static std::string get_info() {
    return fmt::format("[T] '{}',{},{},{}\n", pcTaskGetName(nullptr), xPortGetCoreID(),
                       uxTaskPriorityGet(nullptr), uxTaskGetStackHighWaterMark(nullptr));
  }

  /**
   * @brief Get the info (as a string) for the provided \p task.
   * @param task Reference to the task for which you want the information.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
   * @note This function is only available on ESP32
   */
  static std::string get_info(const Task &task) {
    TaskHandle_t freertos_handle = xTaskGetHandle(task.name_.c_str());
    return fmt::format("[T] '{}',{},{},{}\n", pcTaskGetName(freertos_handle), xPortGetCoreID(),
                       uxTaskPriorityGet(freertos_handle),
                       uxTaskGetStackHighWaterMark(freertos_handle));
  }

  /// Run the given function on the specific core, then return the result (if any)
  /// @details This function will run the given function on the specified core,
  ///         then return the result (if any). If the provided core is the same
  ///         as the current core, the function will run directly. If the
  ///         provided core is different, the function will be run on the
  ///         specified core and the result will be returned to the calling
  ///         thread. Note that this function will block the calling thread until
  ///         the function has completed, regardless of the core it is run on.
  /// @param f The function to run
  /// @param core_id The core to run the function on
  /// @param stack_size_bytes The stack size to allocate for the function
  /// @param priority The priority of the task
  /// @note This function is only available on ESP32
  /// @note If you provide a core_id < 0, the function will run on the current
  ///       core (same core as the caller)
  /// @note If you provide a core_id >= configNUM_CORES, the function will run on
  ///       the last core
  static auto run_on_core(const auto &f, int core_id, size_t stack_size_bytes = 2048,
                          size_t priority = 5) {
    if (core_id < 0 || core_id == xPortGetCoreID()) {
      // If no core id specified or we are already executing on the desired core,
      // run the function directly
      return f();
    } else {
      // Otherwise run the function on the desired core
      if (core_id > configNUM_CORES - 1) {
        // If the core id is larger than the number of cores, run on the last core
        core_id = configNUM_CORES - 1;
      }
      std::mutex mutex;
      std::unique_lock lock(mutex); // cppcheck-suppress localMutex
      std::condition_variable cv;   ///< Signal for when the task is done / function is run
      if constexpr (!std::is_void_v<decltype(f())>) {
        // the function returns something
        decltype(f()) ret_val;
        auto f_task = espp::Task::make_unique(espp::Task::Config{
            .name = "run_on_core_task",
            .callback = [&mutex, &cv, &f, &ret_val](auto &cb_m, auto &cb_cv) -> bool {
              // synchronize with the main thread - block here until the main thread
              // waits on the condition variable (cv), then run the function
              std::unique_lock lock(mutex);
              // run the function
              ret_val = f();
              // signal that the task is done
              cv.notify_all();
              return true; // stop the task
            },
            .stack_size_bytes = stack_size_bytes,
            .priority = priority,
            .core_id = core_id,
        });
        f_task->start();
        cv.wait(lock);
        return ret_val;
      } else {
        // the function returns void
        auto f_task = espp::Task::make_unique(espp::Task::Config{
            .name = "run_on_core_task",
            .callback = [&mutex, &cv, &f](auto &cb_m, auto &cb_cv) -> bool {
              // synchronize with the main thread - block here until the main thread
              // waits on the condition variable (cv), then run the function
              std::unique_lock lock(mutex);
              // run the function
              f();
              // signal that the task is done
              cv.notify_all();
              return true; // stop the task
            },
            .stack_size_bytes = stack_size_bytes,
            .priority = priority,
            .core_id = core_id,
        });
        f_task->start();
        cv.wait(lock);
      }
    }
  }
#endif

protected:
  void thread_function() {
    while (started_) {
      if (callback_) {
        bool should_stop = callback_(cv_m_, cv_);
        if (should_stop) {
          // callback returned true, so stop running the thread function
          logger_.debug("Callback requested stop, thread_function exiting");
          started_ = false;
          break;
        }
      } else if (simple_callback_) {
        bool should_stop = simple_callback_();
        if (should_stop) {
          // callback returned true, so stop running the thread function
          logger_.debug("Callback requested stop, thread_function exiting");
          started_ = false;
          break;
        }
      } else {
        started_ = false;
        break;
      }
    }
  }

  /**
   * @brief Name of the task (used in logs and taks monitoring).
   */
  std::string name_;

  /**
   * @brief Callback function called within Task::thread_function() when
   * started.
   */
  callback_fn callback_;

  /**
   * @brief Simple callback function called within Task::thread_function() when
   * started.
   */
  simple_callback_fn simple_callback_;

  /**
   * @brief Configuration for the task.
   */
  BaseConfig config_;

  std::atomic<bool> started_{false};
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::thread thread_;
};
} // namespace espp

// for printing of BaseConfig using libfmt
template <> struct fmt::formatter<espp::Task::BaseConfig> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Task::BaseConfig &config, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(),
        "Task::BaseConfig{{name: '{}', stack_size_bytes: {}, priority: {}, core_id: {}}}",
        config.name, config.stack_size_bytes, config.priority, config.core_id);
  }
};

// for printing of Task::Config using libfmt
template <> struct fmt::formatter<espp::Task::Config> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const espp::Task::Config &config, FormatContext &ctx) const {
    return fmt::format_to(
        ctx.out(), "Task::Config{{name: '{}', stack_size_bytes: {}, priority: {}, core_id: {}}}",
        config.name, config.stack_size_bytes, config.priority, config.core_id);
  }
};

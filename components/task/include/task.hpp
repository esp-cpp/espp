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

#include "logger.hpp"

namespace espp {

/**
 * @brief Task provides an abstraction over std::thread which optionally
 * includes memory / priority configuration on ESP systems. It allows users to
 * easily stop the task, and will automatically stop itself if destroyed.
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
 */
class Task {
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

  struct Config {
    std::string_view name;             /**< Name of the task */
    callback_fn callback;              /**< Callback function  */
    size_t stack_size_bytes{4 * 1024}; /**< Stack Size (B) allocated to the task. */
    size_t priority{0}; /**< Priority of the task, 0 is lowest priority on ESP / FreeRTOS.  */
    int core_id{-1};    /**< Core ID of the task, -1 means it is not pinned to any core.  */
    Logger::Verbosity log_level{Logger::Verbosity::WARN}; /**< Log verbosity for the task.  */
  };

  Task(const Config &config)
      : name_(config.name), callback_(config.callback), stack_size_bytes_(config.stack_size_bytes),
        priority_(config.priority), core_id_(config.core_id),
        logger_({.tag = name_, .level = config.log_level}) {}

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
    if (core_id_ >= 0)
      thread_config.pin_to_core = core_id_;
    if (core_id_ >= portNUM_PROCESSORS) {
      logger_.error("core_id ({}) is larger than portNUM_PROCESSORS ({}), cannot create Task '{}'",
                    core_id_, portNUM_PROCESSORS, name_);
      return false;
    }
    thread_config.stack_size = stack_size_bytes_;
    thread_config.prio = priority_;
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
          stack_size_bytes_, PTHREAD_STACK_MIN, name_);
      return false;
    }
#endif

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
    started_ = false;
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
    logger_.debug("Task stopped");
    return true;
  }

  /**
   * @brief Has the task been started or not?
   *
   * @return true if the task is started / running, false otherwise.
   */
  bool is_started() { return started_; }

  /**
   * @brief Is the task running?
   *
   * @return true if the task is running, false otherwise.
   */
  bool is_running() { return is_started(); }

#if defined(ESP_PLATFORM)
  /**
   * @brief Get the info (as a string) for the task of the current context.
   * @return std::string containing name, core ID, priority, and stack high
   *         water mark (B)
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
   */
  static std::string get_info(const Task &task) {
    TaskHandle_t freertos_handle = xTaskGetHandle(task.name_.c_str());
    return fmt::format("[T] '{}',{},{},{}\n", pcTaskGetName(freertos_handle), xPortGetCoreID(),
                       uxTaskPriorityGet(freertos_handle),
                       uxTaskGetStackHighWaterMark(freertos_handle));
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

  // NOTE: the below parameters are only used on ESP / FreeRTOS platform
  /**
   * @brief On ESP platform, the amount of bytes allocated to the Task stack.
   */
  size_t stack_size_bytes_;

  /**
   * @brief On ESP platform, the priority of the task, with 0 being lowest
   * priority.
   */
  size_t priority_;

  /**
   * @brief On ESP platform, the core id that the task is pinned to. If -1,
   * then the task is not pinned to a core.
   */
  int core_id_;

  Logger logger_;

  std::atomic<bool> started_{false};
  std::condition_variable cv_;
  std::mutex cv_m_;
  std::thread thread_;
};
} // namespace espp

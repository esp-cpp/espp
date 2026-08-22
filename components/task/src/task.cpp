#include "task.hpp"

#if !defined(ESP_PLATFORM)
#if defined(_WIN32)
#include <windows.h>
#else
#include <algorithm>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#endif
#endif // !ESP_PLATFORM

using namespace espp;

#if !defined(ESP_PLATFORM)
namespace {
#if !defined(_WIN32)
// espp follows the FreeRTOS convention for priorities: 0 is the lowest and
// ~25 (a configMAX_PRIORITIES-like ceiling) is the highest useful priority.
// Host-side scheduling maps espp priorities into the native range using this
// ceiling.
constexpr size_t ESPP_PRIORITY_CEILING = 25;
#endif
// Warn only once per process when real-time scheduling is unavailable
// (e.g. unprivileged Linux without CAP_SYS_NICE / RLIMIT_RTPRIO).
std::atomic<bool> rt_unavailable_warned{false};
} // namespace

bool Task::apply_thread_priority(std::thread &thread, size_t priority) {
  if (!thread.joinable()) {
    return false;
  }
  return apply_thread_priority_to_handle(thread.native_handle(), priority);
}

bool Task::apply_thread_priority_to_handle(std::thread::native_handle_type handle,
                                           size_t priority) {
#if defined(__linux__) || defined(__APPLE__)
  struct sched_param param = {};
  if (priority == 0) {
    // espp priority 0 = default (non-realtime) scheduling. SCHED_OTHER only
    // accepts the static priority range [min, max] of that policy (a single
    // value, 0, on Linux; the default is the middle of the range on macOS).
    const int other_min = sched_get_priority_min(SCHED_OTHER);
    const int other_max = sched_get_priority_max(SCHED_OTHER);
    if (other_min < 0 || other_max < other_min) {
      // sched_get_priority_min/max return -1 on failure; don't feed a bogus
      // (negative) priority to pthread_setschedparam.
      return false;
    }
    param.sched_priority = (other_min + other_max) / 2;
    const int err = pthread_setschedparam(handle, SCHED_OTHER, &param);
    if (err != 0) {
      logger_.debug("Could not reset task '{}' to default scheduling: {}", config_.name,
                    strerror(err));
      return false;
    }
    return true;
  }
  // espp priority >= 1: map linearly onto the SCHED_FIFO real-time priority
  // range. Priority 1 -> the minimum RT priority, ESPP_PRIORITY_CEILING (or
  // above) -> the maximum.
  const int fifo_min = sched_get_priority_min(SCHED_FIFO);
  const int fifo_max = sched_get_priority_max(SCHED_FIFO);
  if (fifo_min < 0 || fifo_max < fifo_min) {
    return false;
  }
  const size_t clamped = std::min(priority, ESPP_PRIORITY_CEILING);
  const int span = fifo_max - fifo_min;
  param.sched_priority = fifo_min + static_cast<int>((clamped - 1) * static_cast<size_t>(span) /
                                                     (ESPP_PRIORITY_CEILING - 1));
  const int err = pthread_setschedparam(handle, SCHED_FIFO, &param);
  if (err != 0) {
    // Most commonly EPERM on Linux: real-time scheduling needs CAP_SYS_NICE or
    // an RLIMIT_RTPRIO allowance. This must never fail the task - fall back to
    // default scheduling and warn once per process. The fallback is an
    // explicit SCHED_OTHER reset: if a previous application succeeded (and
    // e.g. a privilege / RLIMIT_RTPRIO change made this one fail), the thread
    // may currently be running SCHED_FIFO, and leaving it there would
    // contradict the documented fallback and keep a potentially starving RT
    // thread alive.
    const int other_min = sched_get_priority_min(SCHED_OTHER);
    const int other_max = sched_get_priority_max(SCHED_OTHER);
    struct sched_param other_param = {};
    other_param.sched_priority =
        (other_min >= 0 && other_max >= other_min) ? (other_min + other_max) / 2 : 0;
    const int reset_err = pthread_setschedparam(handle, SCHED_OTHER, &other_param);
    if (reset_err != 0) {
      logger_.warn("Could not apply SCHED_FIFO priority {} to task '{}' ({}), and resetting to "
                   "default scheduling also failed ({}); the thread keeps its previous scheduling "
                   "policy",
                   param.sched_priority, config_.name, strerror(err), strerror(reset_err));
    } else if (!rt_unavailable_warned.exchange(true)) {
      logger_.warn("Could not apply SCHED_FIFO priority {} to task '{}' ({}); running without "
                   "realtime priority; grant CAP_SYS_NICE or configure RLIMIT_RTPRIO for RT "
                   "scheduling (e.g. PREEMPT_RT)",
                   param.sched_priority, config_.name, strerror(err));
    }
    return false;
  }
  logger_.debug("Applied SCHED_FIFO priority {} to task '{}'", param.sched_priority, config_.name);
  return true;
#elif defined(_WIN32)
  // Best-effort mapping of the espp priority range onto the Windows thread
  // priority classes.
  int win_priority = THREAD_PRIORITY_NORMAL;
  if (priority >= 17) {
    win_priority = THREAD_PRIORITY_TIME_CRITICAL;
  } else if (priority >= 9) {
    win_priority = THREAD_PRIORITY_HIGHEST;
  } else if (priority >= 1) {
    win_priority = THREAD_PRIORITY_ABOVE_NORMAL;
  }
  if (!SetThreadPriority(static_cast<HANDLE>(handle), win_priority)) {
    if (!rt_unavailable_warned.exchange(true)) {
      logger_.warn("Could not apply thread priority {} to task '{}'; running without elevated "
                   "priority",
                   win_priority, config_.name);
    }
    return false;
  }
  return true;
#else
  // Unknown host platform: priorities are stored but not applied.
  (void)handle;
  (void)priority;
  return false;
#endif
}
#endif // !ESP_PLATFORM

Task::Task(const Task::Config &config)
    : BaseComponent(config.task_config.name, config.log_level)
    , callback_(config.callback)
    , config_(config.task_config)
    , priority_(config.task_config.priority) {}

std::unique_ptr<Task> Task::make_unique(const Task::Config &config) {
  return std::make_unique<Task>(config);
}

Task::~Task() {
  logger_.debug("Destroying task");
  stop();
  notify_and_join();
  logger_.debug("Task destroyed");
}

bool Task::start() {
  logger_.debug("Starting task");
  if (started_) {
    logger_.warn("Task already started!");
    return false;
  }

#if defined(ESP_PLATFORM)
  auto thread_config = esp_pthread_get_default_config();
  thread_config.thread_name = config_.name.c_str();
  auto core_id = config_.core_id;
  if (core_id >= 0)
    thread_config.pin_to_core = core_id;
  if (core_id >= portNUM_PROCESSORS) {
    logger_.error("core_id ({}) is larger than portNUM_PROCESSORS ({}), cannot create Task '{}'",
                  core_id, portNUM_PROCESSORS, config_.name);
    return false;
  }
  thread_config.stack_size = config_.stack_size_bytes;
  // clamp to the valid FreeRTOS priority range, exactly like set_priority():
  // an out-of-range configured value must not make startup fail
  size_t start_priority = priority_.load();
  if (start_priority >= configMAX_PRIORITIES) {
    logger_.warn("Configured priority ({}) >= configMAX_PRIORITIES ({}), clamping", start_priority,
                 configMAX_PRIORITIES);
    start_priority = configMAX_PRIORITIES - 1;
    priority_ = start_priority;
  }
  thread_config.prio = start_priority;
  // this will set the config for the next created thread
  auto err = esp_pthread_set_cfg(&thread_config);
  if (err == ESP_ERR_NO_MEM) {
    logger_.error("Out of memory, cannot create Task '{}'", config_.name);
    return false;
  }
  if (err == ESP_ERR_INVALID_ARG) {
    // see
    // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/pthread.html?highlight=esp_pthread_set_cfg#_CPPv419esp_pthread_set_cfgPK17esp_pthread_cfg_t
    logger_.error(
        "Configured stack size ({}) is less than PTHREAD_STACK_MIN ({}), cannot create Task '{}'",
        config_.stack_size_bytes, PTHREAD_STACK_MIN, config_.name);
    return false;
  }
#endif

  // ensure the thread is not running
  notify_and_join();

  // ensure the notification flag is reset
  {
    std::lock_guard<std::mutex> lock(cv_m_);
    notified_ = false;
  }

  // set the atomic so that when the thread starts it won't immediately
  // exit.
  started_ = true;
  {
    std::lock_guard<std::mutex> lock(thread_mutex_);
    // create and start the std::thread
    thread_ = std::thread(&Task::thread_function, this);
    // On ESP the priority was applied via esp_pthread above; on host platforms
    // (when host_realtime is opted in) the new thread applies the priority to
    // itself at the top of thread_function(), BEFORE the first callback
    // invocation - applying it from here would race the thread's startup and a
    // short callback could run entirely (or even exit) at the default
    // priority.
  }
  logger_.debug("Task started");
  return true;
}

bool Task::stop() {
  if (started_) {
    started_ = false;
    logger_.debug("Stopping task");
#if defined(ESP_PLATFORM)
    stop_watchdog();
#endif
    notify_and_join();
    logger_.debug("Task stopped");
    return true;
  } else {
    logger_.debug("Task already stopped");
    return false;
  }
}

#if defined(ESP_PLATFORM)
bool Task::start_watchdog() {
#if !CONFIG_ESP_TASK_WDT_EN
  logger_.debug("Watchdog not enabled in the configuration, cannot start watchdog!");
  return false;
#else
  if (watchdog_started_) {
    logger_.debug("Watchdog already started!");
    return false;
  }
  logger_.debug("Starting watchdog for task '{}'", config_.name);
  // subscribe to the watchdog
  auto task_handle = static_cast<TaskHandle_t>(get_id());
  if (task_handle == nullptr) {
    logger_.error("Failed to get task handle for task '{}'", config_.name);
    return false;
  }
  auto err = esp_task_wdt_add(task_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to start watchdog for task '{}'", config_.name);
    return false;
  }
  // everything is good, set the flag
  watchdog_started_ = true;
  return true;
#endif // CONFIG_ESP_TASK_WDT_EN
}

bool Task::stop_watchdog() {
#if !CONFIG_ESP_TASK_WDT_EN
  logger_.debug("Watchdog not enabled in the configuration, cannot stop watchdog!");
  return false;
#else
  if (!watchdog_started_) {
    logger_.debug("Watchdog already stopped!");
    return false;
  }
  logger_.debug("Stopping watchdog for task '{}'", config_.name);
  // update the flag
  watchdog_started_ = false;
  // unsubscribe from the watchdog
  auto task_handle = static_cast<TaskHandle_t>(get_id());
  if (task_handle == nullptr) {
    logger_.error("Failed to get task handle for task '{}'", config_.name);
    return false;
  }
  auto err = esp_task_wdt_delete(task_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to stop watchdog for task '{}'", config_.name);
  }
  return err == ESP_OK;
#endif // CONFIG_ESP_TASK_WDT_EN
}

bool Task::configure_task_watchdog(uint32_t timeout_ms, bool panic_on_timeout) {
#if !CONFIG_ESP_TASK_WDT_EN
  return false;
#else
  esp_task_wdt_config_t config;
  memset(&config, 0, sizeof(config));
  config.timeout_ms = timeout_ms;
  config.trigger_panic = panic_on_timeout;
  auto err = esp_task_wdt_status(nullptr);
  if (err == ESP_ERR_INVALID_STATE) {
    // the watchdog was not initialized yet, so initialize it
    err = esp_task_wdt_init(&config);
  } else if (err == ESP_OK || err == ESP_ERR_NOT_FOUND) {
    // the watchdog is already initialized, so reconfigure it
    err = esp_task_wdt_reconfigure(&config);
  } else {
    // some other error occurred
    return false;
  }
  return err == ESP_OK;
#endif // CONFIG_ESP_TASK_WDT_EN
}

bool Task::configure_task_watchdog(const std::chrono::milliseconds &timeout,
                                   bool panic_on_timeout) {
  return configure_task_watchdog(timeout.count(), panic_on_timeout);
}

std::string Task::get_watchdog_info(std::error_code &ec) {
#if !CONFIG_ESP_TASK_WDT_EN
  ec = std::make_error_code(std::errc::operation_not_supported);
  return "";
#else
  std::string info = "";
  auto err = esp_task_wdt_print_triggered_tasks(
      [](void *arg, const char *msg) {
        std::string *info = static_cast<std::string *>(arg);
        *info += msg;
      },
      &info, nullptr);
  if (err == ESP_FAIL) {
    // no triggered tasks were found, no information was printed
  } else if (err != ESP_OK) {
    ec = std::make_error_code(std::errc::io_error);
  }
  return info;
#endif // CONFIG_ESP_TASK_WDT_EN
}
#endif // ESP_PLATFORM

void Task::notify_and_join() {
  {
    std::lock_guard<std::mutex> lock(cv_m_);
    notified_ = true;
  }
  cv_.notify_all();
  auto thread_id = get_id();
  auto current_id = get_current_id();
  logger_.debug("Thread id: {}, current id: {}", thread_id, current_id);
  // check to ensure we're not the same thread
  std::lock_guard<std::mutex> lock(thread_mutex_);
  if (thread_.joinable() && current_id != thread_id) {
    thread_.join();
#if defined(ESP_PLATFORM)
    task_handle_ = nullptr;
#endif
  }
}

bool Task::is_started() const { return started_; }

bool Task::is_running() const { return is_started(); }

bool Task::set_priority(size_t priority) {
#if defined(ESP_PLATFORM)
  // clamp to the valid FreeRTOS priority range
  if (priority >= configMAX_PRIORITIES) {
    logger_.warn("Requested priority ({}) >= configMAX_PRIORITIES ({}), clamping", priority,
                 configMAX_PRIORITIES);
    priority = configMAX_PRIORITIES - 1;
  }
#endif
  // always store the new priority so it is used on the next start() (atomic:
  // the worker thread's startup priority application and
  // get_configured_priority() read it concurrently)
  priority_ = priority;
  logger_.debug("Set priority to {} for task '{}'", priority, config_.name);
#if defined(ESP_PLATFORM)
  // if the task is running, apply the change to the live task as well
  auto handle = static_cast<TaskHandle_t>(get_id());
  if (started_ && handle != nullptr) {
    vTaskPrioritySet(handle, static_cast<UBaseType_t>(priority));
    return true;
  }
#else
  // if the task is running and host real-time scheduling was opted in, apply
  // the change to the live thread as well (best-effort; see
  // BaseConfig::host_realtime for the per-platform semantics)
  if (started_ && config_.host_realtime) {
    std::lock_guard<std::mutex> lock(thread_mutex_);
    return apply_thread_priority(thread_, priority);
  }
#endif
  return false;
}

bool Task::set_core_id(int core_id) {
  // always store the new core id so it is used on the next start()
  config_.core_id = core_id;
  logger_.debug("Set core id to {} for task '{}'", core_id, config_.name);
#if defined(ESP_PLATFORM) && defined(configUSE_CORE_AFFINITY) && (configUSE_CORE_AFFINITY == 1) && \
    (configNUMBER_OF_CORES > 1)
  // this FreeRTOS build supports changing a live task's core affinity
  auto handle = static_cast<TaskHandle_t>(get_id());
  if (started_ && handle != nullptr) {
    UBaseType_t affinity_mask;
    if (core_id < 0) {
      // not pinned: allow the task to run on any core
      affinity_mask = (1u << configNUMBER_OF_CORES) - 1u;
    } else {
      affinity_mask = 1u << core_id;
    }
    vTaskCoreAffinitySet(handle, affinity_mask);
    return true;
  }
  return false;
#else
  // the default ESP-IDF FreeRTOS port fixes core affinity at task creation, so
  // the new core id only takes effect the next time the task is started
  if (started_) {
    logger_.warn("Cannot change core affinity of running task '{}'; new core id ({}) will take "
                 "effect on next start()",
                 config_.name, core_id);
  }
  return false;
#endif
}

#if defined(ESP_PLATFORM) || defined(_DOXYGEN_)
std::string Task::get_info() {
  return fmt::format("[T] '{}',{},{},{}", pcTaskGetName(nullptr), xPortGetCoreID(),
                     uxTaskPriorityGet(nullptr), uxTaskGetStackHighWaterMark(nullptr));
}

std::string Task::get_info(const Task &task) {
  TaskHandle_t freertos_handle = static_cast<TaskHandle_t>(task.get_id());
  return fmt::format("[T] '{}',{},{},{}", pcTaskGetName(freertos_handle), get_core_id(task),
                     uxTaskPriorityGet(freertos_handle),
                     uxTaskGetStackHighWaterMark(freertos_handle));
}
#endif

void Task::thread_function() {
#if defined(ESP_PLATFORM)
  task_handle_ = get_current_id();
#else
  // Apply the (opted-in) host scheduling policy to ourselves before the first
  // callback invocation, so the priority reliably covers the task's entire
  // execution (best-effort: an unprivileged failure falls back to default
  // scheduling with a one-time warning).
  if (config_.host_realtime) {
#if defined(_WIN32)
    apply_thread_priority_to_handle(GetCurrentThread(), priority_.load());
#elif defined(__linux__) || defined(__APPLE__)
    apply_thread_priority_to_handle(pthread_self(), priority_.load());
#endif
  }
#endif // ESP_PLATFORM
  while (started_) {
    bool should_stop = false;
    if (std::holds_alternative<callback_m_cv_notified_fn>(callback_)) {
      auto cb = std::get<callback_m_cv_notified_fn>(callback_);
      should_stop = cb(cv_m_, cv_, notified_);
    } else if (std::holds_alternative<callback_m_cv_fn>(callback_)) {
      auto cb = std::get<callback_m_cv_fn>(callback_);
      should_stop = cb(cv_m_, cv_);
    } else if (std::holds_alternative<callback_no_params_fn>(callback_)) {
      auto cb = std::get<callback_no_params_fn>(callback_);
      should_stop = cb();
    } else {
      started_ = false;
      break;
    }
    if (should_stop) {
      // callback returned true, so stop running the thread function
      logger_.debug("Callback requested stop, thread_function exiting");
      started_ = false;
      break;
    }

#if defined(ESP_PLATFORM) && CONFIG_ESP_TASK_WDT_EN
    // check if the watchdog is enabled
    if (watchdog_started_) {
      auto err = esp_task_wdt_reset();
      if (err != ESP_OK) {
        logger_.error("Watchdog reset failed for task '{}'", config_.name);
      }
    }
#endif // ESP_PLATFORM
  }    // while (started_)
}

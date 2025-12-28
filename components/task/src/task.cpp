#include "task.hpp"

using namespace espp;

Task::Task(const Task::Config &config)
    : BaseComponent(config.task_config.name, config.log_level)
    , name_(config.task_config.name)
    , callback_(config.callback)
    , config_(config.task_config) {}

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
  logger_.debug("Starting watchdog for task '{}'", name_);
  // subscribe to the watchdog
  auto task_handle = static_cast<TaskHandle_t>(get_id());
  if (task_handle == nullptr) {
    logger_.error("Failed to get task handle for task '{}'", name_);
    return false;
  }
  auto err = esp_task_wdt_add(task_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to start watchdog for task '{}'", name_);
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
  logger_.debug("Stopping watchdog for task '{}'", name_);
  // update the flag
  watchdog_started_ = false;
  // unsubscribe from the watchdog
  auto task_handle = static_cast<TaskHandle_t>(get_id());
  if (task_handle == nullptr) {
    logger_.error("Failed to get task handle for task '{}'", name_);
    return false;
  }
  auto err = esp_task_wdt_delete(task_handle);
  if (err != ESP_OK) {
    logger_.error("Failed to stop watchdog for task '{}'", name_);
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
        logger_.error("Watchdog reset failed for task '{}'", name_);
      }
    }
#endif // ESP_PLATFORM
  }    // while (started_)
}

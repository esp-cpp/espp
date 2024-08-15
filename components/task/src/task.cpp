#include "task.hpp"

using namespace espp;

Task::Task(const Task::Config &config)
    : BaseComponent(config.name, config.log_level)
    , name_(config.name)
    , callback_(config.callback)
    , config_({config.name, config.stack_size_bytes, config.priority, config.core_id}) {}

Task::Task(const Task::SimpleConfig &config)
    : BaseComponent(config.task_config.name, config.log_level)
    , name_(config.task_config.name)
    , simple_callback_(config.callback)
    , config_(config.task_config) {}

Task::Task(const Task::AdvancedConfig &config)
    : BaseComponent(config.task_config.name, config.log_level)
    , name_(config.task_config.name)
    , callback_(config.callback)
    , config_(config.task_config) {}

std::unique_ptr<Task> Task::make_unique(const Task::Config &config) {
  return std::make_unique<Task>(config);
}

std::unique_ptr<Task> Task::make_unique(const Task::SimpleConfig &config) {
  return std::make_unique<Task>(config);
}

std::unique_ptr<Task> Task::make_unique(const Task::AdvancedConfig &config) {
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
    notify_and_join();
    logger_.debug("Task stopped");
    return true;
  } else {
    logger_.debug("Task already stopped");
    return false;
  }
}

void Task::notify_and_join() {
  {
    std::lock_guard<std::mutex> lock(cv_m_);
    cv_.notify_all();
  }
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
  return fmt::format("[T] '{}',{},{},{}\n", pcTaskGetName(nullptr), xPortGetCoreID(),
                     uxTaskPriorityGet(nullptr), uxTaskGetStackHighWaterMark(nullptr));
}

std::string Task::get_info(const Task &task) {
  TaskHandle_t freertos_handle = static_cast<TaskHandle_t>(task.get_id());
  return fmt::format("[T] '{}',{},{},{}\n", pcTaskGetName(freertos_handle), xPortGetCoreID(),
                     uxTaskPriorityGet(freertos_handle),
                     uxTaskGetStackHighWaterMark(freertos_handle));
}
#endif

void Task::thread_function() {
#if defined(ESP_PLATFORM)
  task_handle_ = get_current_id();
#endif
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

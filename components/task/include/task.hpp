#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <thread>

#if defined(ESP_PLATFORM)
#include <esp_pthread.h>
#endif

#include "logger.hpp"

namespace espp {
  class Task {
  public:

    typedef std::function<void(std::mutex& m, std::condition_variable& cv)> task_fn;

    // NOTE: the callback is run repeatedly within the Task, therefore it MUST
    // return, and also SHOULD have a sleep to give the processor over to other
    // tasks. For this reason, the callback is provided a
    // std::condition_variable (and associated mutex) which the callback can use
    // when they need to wait. If the cv.wait_for / cv.wait_until return
    // std::cv_status::timeout, no action is necessary, but if they return
    // std::cv_status::no_timeout, then the function should return immediately
    // since the task is being stopped (optionally performing any task-specific
    // tear-down).
    struct Config {
      std::string_view name;
      task_fn callback;
      size_t stack_size_bytes{4*1024};
      size_t priority{0};
      int core_id{-1};
      Logger::Level log_level{Logger::Level::WARN};
    };

    Task(const Config& config) : name_(config.name),
                                 callback_(config.callback),
                                 stack_size_bytes_(config.stack_size_bytes),
                                 priority_(config.priority),
                                 core_id_(config.core_id),
                                 logger_({.tag = config.name, .level = config.log_level}) {}

    static std::unique_ptr<Task> make_unique(const Config& config) { return std::make_unique<Task>(config); }

    ~Task() {
      logger_.debug("Destroying task");
      // stop the task if it was started
      if (started_) {
        stop();
      }
      logger_.debug("Task destroyed");
    }

    bool start() {
      logger_.debug("Starting task");
      if (started_) {
        logger_.warn("Task already started!");
        return false;
      }
      started_ = true;

#if defined(ESP_PLATFORM)
      auto thread_config = esp_pthread_get_default_config();
      thread_config.thread_name = name_.c_str();
      if (core_id_ > 0)
        thread_config.pin_to_core = core_id_;
      thread_config.stack_size = stack_size_bytes_;
      thread_config.prio = priority_;
      esp_pthread_set_cfg(&thread_config);
#endif

      // create and start the std::thread
      thread_ = std::thread(&Task::thread_function, this);

      logger_.debug("Task started");
      return true;
    }

    bool stop() {
      logger_.debug("Stopping task");
      if (!started_) {
        logger_.warn("Task is not running!");
        return false;
      }
      started_ = false;
      cv_.notify_all();
      if (thread_.joinable()) {
        thread_.join();
      }
      logger_.debug("Task stopped");
      return true;
    }

    bool is_started() { return started_; }

  protected:
    void thread_function() {
      while (started_) {
        if (callback_) {
          callback_(cv_m_, cv_);
        } else {
          break;
        }
      }
    }

    std::string name_;
    task_fn callback_;

    // NOTE: the below parameters are only used on ESP / FreeRTOS platform
    size_t stack_size_bytes_;
    size_t priority_;
    int core_id_;

    Logger logger_;

    std::atomic<bool> started_{false};
    std::condition_variable cv_;
    std::mutex cv_m_;
    std::thread thread_;
  };
}

#pragma once

#include <atomic>
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

    // NOTE: the callback is run repeatedly within the Task, therefore it MUST
    // return, and also SHOULD have a sleep to give the processor over to other
    // tasks
    struct Config {
      std::string_view name;
      std::function<void()> callback;
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
      // stop the task if it was started
      stop();
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
          callback_();
        } else {
          break;
        }
      }
    }

    std::string name_;
    std::function<void()> callback_;

    // NOTE: the below parameters are only used on ESP / FreeRTOS platform
    size_t stack_size_bytes_;
    size_t priority_;
    int core_id_;

    Logger logger_;

    std::atomic<bool> started_{false};
    std::thread thread_;
  };
}

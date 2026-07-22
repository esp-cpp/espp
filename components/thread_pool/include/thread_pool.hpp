#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "base_component.hpp"
#include "task.hpp"

namespace espp {

class ThreadPool : public espp::BaseComponent {
public:
  using Job = std::function<void(void)>;

  struct Stats {
    std::uint64_t submitted = 0;
    std::uint64_t executed = 0;
    std::uint64_t rejected = 0;
  };

  struct Config {
    std::size_t worker_count = 1;
    std::size_t max_queue_size = 0; // 0 means unbounded
    bool auto_start = true;
    bool block_on_submit_when_full = false;
    espp::Task::BaseConfig worker_task_config = {
        .name = "thread_pool_worker",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = -1,
    };
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN;
  };

  explicit ThreadPool(const Config &config);
  ~ThreadPool();

  void start();
  void stop();

  bool is_running() const;

  bool submit(const Job &job);
  bool try_submit(const Job &job);

  std::size_t queue_size() const;
  std::size_t worker_count() const;

  Stats stats() const;

private:
  bool worker_task_fn(std::mutex &task_mutex, std::condition_variable &task_cv,
                      bool &task_notified);

  bool submit_impl(const Job &job, bool allow_blocking_when_full);

  Config config_;

  mutable std::mutex queue_mutex_;
  std::condition_variable queue_has_work_cv_;
  std::condition_variable queue_has_space_cv_;
  std::deque<Job> queue_;

  std::vector<std::unique_ptr<espp::Task>> workers_;
  std::atomic<bool> running_{false};
  bool stopping_ = false;

  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> executed_{0};
  std::atomic<std::uint64_t> rejected_{0};
};

} // namespace espp

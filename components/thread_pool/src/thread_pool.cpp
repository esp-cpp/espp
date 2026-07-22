#include "thread_pool.hpp"

#include <utility>

using namespace espp;

ThreadPool::ThreadPool(const Config &config)
    : BaseComponent("ThreadPool", config.log_level), config_(config) {
  if (config_.worker_count == 0) {
    config_.worker_count = 1;
  }

  workers_.reserve(config_.worker_count);
  for (std::size_t i = 0; i < config_.worker_count; ++i) {
    auto worker_config = config_.worker_task_config;
    worker_config.name = config_.worker_task_config.name + "_" + std::to_string(i);
    using namespace std::placeholders;
    workers_.emplace_back(espp::Task::make_unique({
        .callback = std::bind(&ThreadPool::worker_task_fn, this, _1, _2, _3),
        .task_config = worker_config,
        .log_level = config_.log_level,
    }));
  }

  if (config_.auto_start) {
    start();
  }
}

ThreadPool::~ThreadPool() { stop(); }

void ThreadPool::start() {
  if (running_.exchange(true)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = false;
  }

  for (std::size_t i = 0; i < workers_.size(); ++i) {
    auto &worker = workers_[i];
    if (!worker->start()) {
      // Handle the error if needed, e.g., log it
      logger_.warn("Failed to start worker thread {}. could be already started or not enough memory", i);
    }

  }
}

void ThreadPool::stop() {
  if (!running_.exchange(false)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = true;
  }

  queue_has_work_cv_.notify_all();
  queue_has_space_cv_.notify_all();

  for (auto &worker : workers_) {
    worker->stop();
  }
}

bool ThreadPool::is_running() const { return running_.load(); }

bool ThreadPool::submit(const Job &job) {
  return submit_impl(job, config_.block_on_submit_when_full);
}

bool ThreadPool::try_submit(const Job &job) { return submit_impl(job, false); }

bool ThreadPool::submit_impl(const Job &job, bool allow_blocking_when_full) {
  if (!job) {
    rejected_++;
    return false;
  }

  std::unique_lock<std::mutex> lock(queue_mutex_);
  if (!running_.load() || stopping_) {
    rejected_++;
    return false;
  }

  if (config_.max_queue_size > 0) {
    if (allow_blocking_when_full) {
      queue_has_space_cv_.wait(lock, [&]() {
        return stopping_ || queue_.size() < config_.max_queue_size;
      });
      if (stopping_) {
        rejected_++;
        return false;
      }
    } else if (queue_.size() >= config_.max_queue_size) {
      rejected_++;
      return false;
    }
  }

  queue_.push_back(job);
  submitted_++;
  lock.unlock();
  queue_has_work_cv_.notify_one();
  return true;
}

std::size_t ThreadPool::queue_size() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return queue_.size();
}

std::size_t ThreadPool::worker_count() const { return workers_.size(); }

ThreadPool::Stats ThreadPool::stats() const {
  return {
      .submitted = submitted_.load(),
      .executed = executed_.load(),
      .rejected = rejected_.load(),
  };
}

bool ThreadPool::worker_task_fn(std::mutex &task_mutex,
                                std::condition_variable &task_cv,
                                bool &task_notified) {
  (void)task_cv;

  Job job;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_has_work_cv_.wait(lock, [&]() { return stopping_ || !queue_.empty(); });

    if (queue_.empty()) {
      std::unique_lock<std::mutex> task_lock(task_mutex);
      return stopping_ || task_notified;
    }

    job = std::move(queue_.front());
    queue_.pop_front();

    if (config_.max_queue_size > 0) {
      queue_has_space_cv_.notify_one();
    }
  }

  job();
  executed_++;

  std::unique_lock<std::mutex> task_lock(task_mutex);
  return task_notified;
}

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
    workers_.emplace_back(espp::Task::make_unique({
        .callback = [this]() { return worker_task_fn(); },
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
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (running_.load()) {
      return;
    }
    stopping_ = false;
    running_.store(true);
  }

  std::size_t started_count = 0;
  for (std::size_t i = 0; i < workers_.size(); ++i) {
    if (workers_[i]->start()) {
      ++started_count;
    } else {
      logger_.warn("Failed to start worker {} (already started or insufficient memory)", i);
    }
  }

  if (started_count == 0) {
    logger_.error("No workers started; rolling back pool start");
    for (auto &worker : workers_) {
      worker->stop();
    }
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      running_.store(false);
      stopping_ = true;
      rejected_ += static_cast<std::uint64_t>(queue_.size());
      queue_.clear();
    }
    queue_has_space_cv_.notify_all();
  }
}

void ThreadPool::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);
  if (!running_.exchange(false)) {
    return;
  }

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    stopping_ = true;
    rejected_ += static_cast<std::uint64_t>(queue_.size());
    queue_.clear();
  }

  queue_has_work_cv_.notify_all();
  queue_has_space_cv_.notify_all();

  for (auto &worker : workers_) {
    worker->stop();
  }
}

bool ThreadPool::is_running() const { return running_.load(); }

bool ThreadPool::submit(Job job) {
  return submit_impl(std::move(job), config_.block_on_submit_when_full);
}

bool ThreadPool::try_submit(Job job) { return submit_impl(std::move(job), false); }

bool ThreadPool::submit_impl(Job job, bool allow_blocking_when_full) {
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

  queue_.push_back(std::move(job));
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

bool ThreadPool::worker_task_fn() {
  Job job;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_has_work_cv_.wait(lock, [&]() { return stopping_ || !queue_.empty(); });

    if (stopping_) {
      return true;
    }

    job = std::move(queue_.front());
    queue_.pop_front();
  }

  if (config_.max_queue_size > 0) {
    queue_has_space_cv_.notify_one();
  }

  job();
  executed_++;

  return false;
}

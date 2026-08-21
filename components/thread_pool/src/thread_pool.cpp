#include "thread_pool.hpp"

#include <algorithm>
#include <utility>

using namespace espp;

ThreadPool::ThreadPool(const Config &config)
    : BaseComponent("ThreadPool", config.log_level)
    , config_(config) {
  per_band_workers_ =
      std::any_of(config_.band_worker_counts.begin(), config_.band_worker_counts.end(),
                  [](std::size_t count) { return count > 0; });

  if (per_band_workers_) {
    // Per-band workers: band k gets band_worker_counts[k] workers at
    // band_task_priorities[k], each servicing bands 0..k (its own band and
    // every more urgent band).
    for (std::size_t band = 0; band < kNumBands; ++band) {
      for (std::size_t i = 0; i < config_.band_worker_counts[band]; ++i) {
        auto worker_config = config_.worker_task_config;
        worker_config.name =
            config_.worker_task_config.name + "_b" + std::to_string(band) + "_" + std::to_string(i);
        worker_config.priority = config_.band_task_priorities[band];
        workers_.emplace_back(espp::Task::make_unique({
            .callback = [this, band]() { return worker_task_fn(band); },
            .task_config = worker_config,
            .log_level = config_.log_level,
        }));
      }
    }
  } else {
    if (config_.worker_count == 0) {
      config_.worker_count = 1;
    }
    workers_.reserve(config_.worker_count);
    for (std::size_t i = 0; i < config_.worker_count; ++i) {
      auto worker_config = config_.worker_task_config;
      worker_config.name = config_.worker_task_config.name + "_" + std::to_string(i);
      workers_.emplace_back(espp::Task::make_unique({
          .callback = [this]() { return worker_task_fn(kNumBands - 1); },
          .task_config = worker_config,
          .log_level = config_.log_level,
      }));
    }
  }

  if (config_.auto_start) {
    start();
  }
}

ThreadPool::~ThreadPool() { stop(); }

bool ThreadPool::start() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  if (running_.load()) {
    return true;
  }
  stopping_ = false;

  bool all_workers_started = true;
  for (std::size_t i = 0; i < workers_.size(); ++i) {
    if ((!workers_[i]->start()) && (!workers_[i]->is_running())) {
      logger_.warn("Failed to start worker {} (already started or insufficient memory)", i);
      all_workers_started = false;
      break;
    }
  }

  if (!all_workers_started) {
    logger_.error("Not all workers started; rolling back pool start");
    stopping_ = true;
    queue_has_work_cv_.notify_all();
    queue_has_space_cv_.notify_all();
    for (auto &worker : workers_) {
      worker->stop();
    }
    return false;
  }

  running_.store(true);
  return true;
}

void ThreadPool::stop() {
  std::lock_guard<std::mutex> lifecycle_lock(lifecycle_mutex_);

  if (!running_.exchange(false)) {
    return;
  }
  stopping_ = true;

  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    rejected_ += static_cast<std::uint64_t>(total_queued_);
    for (auto &queue : queues_) {
      queue.clear();
    }
    total_queued_ = 0;
  }

  queue_has_work_cv_.notify_all();
  queue_has_space_cv_.notify_all();

  for (auto &worker : workers_) {
    worker->stop();
  }
}

bool ThreadPool::is_running() const { return running_.load(); }

bool ThreadPool::submit(Job &&job) {
  return submit_impl(std::move(job), QosBand::Normal, config_.block_on_submit_when_full);
}

bool ThreadPool::submit(Job &&job, QosBand band) {
  return submit_impl(std::move(job), band, config_.block_on_submit_when_full);
}

bool ThreadPool::try_submit(Job &&job) {
  return submit_impl(std::move(job), QosBand::Normal, false);
}

bool ThreadPool::try_submit(Job &&job, QosBand band) {
  return submit_impl(std::move(job), band, false);
}

bool ThreadPool::submit_impl(Job &&job, QosBand band, bool allow_blocking_when_full) {
  if (!job) {
    rejected_++;
    return false;
  }

  auto band_index = static_cast<std::size_t>(band);
  if (band_index >= kNumBands) {
    logger_.warn("Invalid band {}, clamping to Low", band_index);
    band_index = static_cast<std::size_t>(QosBand::Low);
  }

  std::unique_lock<std::mutex> lock(queue_mutex_);
  if (!running_.load() || stopping_) {
    rejected_++;
    return false;
  }

  if (config_.max_queue_size > 0) {
    if (allow_blocking_when_full) {
      queue_has_space_cv_.wait(
          lock, [&]() { return stopping_ || total_queued_ < config_.max_queue_size; });
      if (stopping_) {
        rejected_++;
        return false;
      }
    } else if (total_queued_ >= config_.max_queue_size) {
      rejected_++;
      return false;
    }
  }

  queues_[band_index].push_back(Entry{std::move(job), std::chrono::steady_clock::now()});
  ++total_queued_;
  submitted_++;
  band_submitted_[band_index]++;
  lock.unlock();
  if (per_band_workers_) {
    // Workers are heterogeneous (each sees only bands 0..k); notify_one could
    // wake a worker that cannot service this band, so wake them all.
    queue_has_work_cv_.notify_all();
  } else {
    queue_has_work_cv_.notify_one();
  }
  return true;
}

std::size_t ThreadPool::queue_size() const {
  std::lock_guard<std::mutex> lock(queue_mutex_);
  return total_queued_;
}

std::size_t ThreadPool::worker_count() const { return workers_.size(); }

ThreadPool::Stats ThreadPool::stats() const {
  Stats s{
      .submitted = submitted_.load(),
      .executed = executed_.load(),
      .rejected = rejected_.load(),
  };
  for (std::size_t band = 0; band < kNumBands; ++band) {
    s.band_submitted[band] = band_submitted_[band].load();
    s.band_executed[band] = band_executed_[band].load();
    s.band_aged[band] = band_aged_[band].load();
  }
  return s;
}

bool ThreadPool::age_bands_locked() {
  bool promoted = false;
  const auto now = std::chrono::steady_clock::now();
  for (std::size_t band = 1; band < kNumBands; ++band) {
    if (queues_[band].empty()) {
      continue;
    }
    // Bands are FIFO, so the front is always the longest-waiting entry of its
    // band - checking only the front is sufficient (and O(bands) per pop).
    if (now - queues_[band].front().enqueued_at < config_.aging_threshold) {
      continue;
    }
    Entry entry = std::move(queues_[band].front());
    queues_[band].pop_front();
    // Restart the aging clock: an entry ages up at most one band per
    // aging_threshold interval. It enters the more urgent band at the back,
    // ahead of all of that band's later arrivals (FIFO), which is what
    // guarantees progress under sustained load.
    entry.enqueued_at = now;
    queues_[band - 1].push_back(std::move(entry));
    band_aged_[band]++;
    promoted = true;
  }
  return promoted;
}

bool ThreadPool::worker_task_fn(std::size_t max_band) {
  Job job;
  std::size_t band = 0;
  bool promoted = false;
  const bool aging_enabled = config_.aging_threshold.count() > 0;
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    auto have_visible_work = [&]() {
      const auto visible_end = queues_.begin() + static_cast<std::ptrdiff_t>(max_band) + 1;
      return std::any_of(queues_.begin(), visible_end,
                         [](const auto &queue) { return !queue.empty(); });
    };
    auto pred = [&]() { return stopping_.load() || have_visible_work(); };
    if (aging_enabled && per_band_workers_) {
      // A band-restricted worker cannot see the less urgent bands, but it is
      // still responsible for aging them (promotion is what eventually makes a
      // starving entry visible to some worker); wake periodically to do so.
      queue_has_work_cv_.wait_for(lock, config_.aging_threshold, pred);
    } else {
      queue_has_work_cv_.wait(lock, pred);
    }

    if (stopping_) {
      return true;
    }

    if (aging_enabled) {
      promoted = age_bands_locked();
    }

    // Take from the most urgent (lowest index) non-empty band this worker
    // services.
    std::size_t found_band = kNumBands;
    for (std::size_t b = 0; b <= max_band; ++b) {
      if (!queues_[b].empty()) {
        found_band = b;
        break;
      }
    }
    if (found_band == kNumBands) {
      // Nothing visible (e.g. a timed wakeup purely to age other bands).
      if (promoted && per_band_workers_) {
        lock.unlock();
        queue_has_work_cv_.notify_all();
      }
      return false;
    }
    band = found_band;
    job = std::move(queues_[band].front().job);
    queues_[band].pop_front();
    --total_queued_;
  }

  if (promoted && per_band_workers_) {
    // A promotion may have made work visible to a sleeping band-restricted
    // worker that submit() targeted at a band it cannot see.
    queue_has_work_cv_.notify_all();
  }

  if (config_.max_queue_size > 0) {
    queue_has_space_cv_.notify_one();
  }

  // Count the job as executed BEFORE invoking it: anything the job makes
  // observable (e.g. signalling a waiter that then reads stats()) must already
  // see this job accounted for. Counting after the call would race with such
  // observers (the worker can be descheduled between the job body finishing
  // and the increment - especially when workers run at real-time priority).
  executed_++;
  band_executed_[band]++;
  job();

  return false;
}

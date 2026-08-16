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

/**
 * @brief A thread pool that dispatches submitted jobs to a fixed set of worker threads.
 *
 * Workers are implemented as espp::Task instances. Jobs are queued and
 * consumed in FIFO order. The queue can be optionally bounded; when full,
 * new submissions are either rejected immediately or blocked until space
 * becomes available, depending on the configuration.
 *
 * \section thread_pool_ex1 Lifecycle: start / stop / is_running / worker_count
 * \snippet thread_pool_example.cpp lifecycle example
 * \section thread_pool_ex2 Submit Jobs
 * \snippet thread_pool_example.cpp submit example
 * \section thread_pool_ex3 try_submit - Non-Blocking Rejection When Full
 * \snippet thread_pool_example.cpp try_submit example
 * \section thread_pool_ex4 Blocking Submit When Full
 * \snippet thread_pool_example.cpp blocking submit example
 * \section thread_pool_ex5 Submit Rejected After stop()
 * \snippet thread_pool_example.cpp submit after stop example
 * \section thread_pool_ex6 Concurrent start / stop
 * \snippet thread_pool_example.cpp concurrent lifecycle example
 * \section thread_pool_ex7 Concurrent submit and try_submit
 * \snippet thread_pool_example.cpp concurrent submit example
 * \section thread_pool_ex8 Chained Pools
 * \snippet thread_pool_example.cpp chained pools example
 * \section thread_pool_ex9 Self-Submit
 * \snippet thread_pool_example.cpp self-submit example
 */
class ThreadPool : public espp::BaseComponent {
public:
  /// @brief A callable job that can be submitted to the pool.
  using Job = std::function<void(void)>;

  /// @brief Snapshot of pool activity counters.
  struct Stats {
    std::uint64_t submitted = 0; ///< Total jobs accepted into the queue.
    std::uint64_t executed = 0;  ///< Total jobs successfully executed.
    std::uint64_t rejected = 0;  ///< Total jobs rejected (invalid job, stopped/stopping, or queue
                                ///< full) or dropped (due to stop, the enqueued jobs were dropped).
  };

  /// @brief Configuration parameters for constructing a ThreadPool.
  struct Config {
    std::size_t worker_count = 1;   ///< Number of worker threads to spawn.
    std::size_t max_queue_size = 0; ///< Maximum pending jobs (0 = unbounded).
    bool auto_start = true;         ///< Start workers immediately on construction.
    bool block_on_submit_when_full =
        false; ///< If true, submit() blocks when the queue is full instead of rejecting.
    espp::Task::BaseConfig worker_task_config = {
        ///< Base configuration applied to every worker task.
        .name = "thread_pool_worker",
        .stack_size_bytes = 4096,
        .priority = 5,
        .core_id = -1,
    };
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; ///< Logger verbosity level.
  };

  /// @brief Construct the pool with the given configuration.
  /// @param config Pool configuration.
  explicit ThreadPool(const Config &config);

  /// @brief Destroy the pool, stopping all workers gracefully.
  ~ThreadPool();

  /// @brief Start all worker threads.
  /// @return True if all workers were successfully started, false otherwise.
  /// @note No-op if the pool is already running and return true immediately.
  /// @note If any workers could not be started, the pool will roll back to the stopped state.
  bool start();

  /// @brief Stop all worker threads and reject further submissions.
  /// @note Blocks until every worker has exited; queued jobs may not be executed.
  void stop();

  /// @brief Query whether the pool is currently running.
  /// @return true if workers are active, false otherwise.
  bool is_running() const;

  /// @brief Submit a job, optionally blocking when the queue is full.
  ///
  /// Blocks if Config::block_on_submit_when_full is true and the queue has
  /// reached its capacity limit. Otherwise behaves identically to try_submit().
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @return true if the job was accepted, false if it was rejected.
  bool submit(Job &&job);

  /// @brief Attempt to submit a job without blocking.
  ///
  /// Returns immediately with false when the queue is full.
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @return true if the job was accepted, false if it was rejected.
  bool try_submit(Job &&job);

  /// @brief Return the number of jobs currently waiting in the queue.
  /// @return Pending job count.
  std::size_t queue_size() const;

  /// @brief Return the number of worker threads in the pool.
  /// @return Worker thread count.
  std::size_t worker_count() const;

  /// @brief Return a snapshot of the pool's activity counters.
  /// @return Stats struct with submitted, executed, and rejected counts.
  Stats stats() const;

private:
  bool worker_task_fn();

  bool submit_impl(Job &&job, bool allow_blocking_when_full);

  Config config_;

  std::mutex lifecycle_mutex_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_has_work_cv_;
  std::condition_variable queue_has_space_cv_;
  std::deque<Job> queue_;

  std::vector<std::unique_ptr<espp::Task>> workers_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stopping_{false};

  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> executed_{0};
  std::atomic<std::uint64_t> rejected_{0};
};

} // namespace espp

#include "thread_pool_format_helpers.hpp"

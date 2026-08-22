#pragma once

#include <array>
#include <atomic>
#include <chrono>
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
#include "qos_band.hpp"
#include "task.hpp"

namespace espp {

/**
 * @brief A thread pool that dispatches submitted jobs to a fixed set of worker
 *        threads, with bucketed priority bands (espp::QosBand).
 *
 * Workers are implemented as espp::Task instances. Jobs are queued into one of
 * four priority bands (QosBand::Critical/High/Normal/Low) and consumed in FIFO
 * order within a band, with more urgent (lower-index) bands always drained
 * first. Submitting without a band uses QosBand::Normal, which preserves the
 * original single-FIFO-queue behavior exactly. The queue can be optionally
 * bounded (Config::max_queue_size bounds the TOTAL across all bands); when
 * full, new submissions are either rejected immediately or blocked until space
 * becomes available, depending on the configuration.
 *
 * **Aging (starvation guard).** With strict band priority a continuous stream
 * of urgent jobs could starve less urgent bands forever. To prevent this, when
 * a worker looks for work it first promotes the front entry of any band whose
 * wait time exceeds Config::aging_threshold up one band (to the back of the
 * next more-urgent band, with its aging clock restarted). Aging is
 * deliberately approximate: only band fronts are examined (O(bands) per pop,
 * no full-queue scans), and because bands are FIFO this is sufficient - the
 * front is always the longest-waiting entry of its band. aging_threshold is
 * an ELIGIBILITY interval, not an at-most wait bound: promotion is evaluated
 * only when a worker next dequeues work, so a hop can happen arbitrarily
 * later than the threshold if every worker is stuck in a long in-flight job,
 * and after promotion the entry still waits behind the destination band's
 * backlog at that moment. What IS guaranteed: since promoted entries enter
 * ahead of all later arrivals, every queued entry makes progress toward
 * Critical under any sustained load, so nothing starves. Set aging_threshold
 * to 0 for strict (starvation-permitting) band priority.
 *
 * **Worker bands (true OS preemption, opt-in).** By default all
 * Config::worker_count workers are identical and service every band. Setting
 * any element of Config::band_worker_counts non-zero switches to per-band
 * workers instead: band_worker_counts[k] workers are created for band k, each
 * running at the espp::Task priority Config::band_task_priorities[k] (a
 * FreeRTOS priority on ESP; mapped to a SCHED_FIFO real-time priority on
 * Linux/macOS - see espp::Task::BaseConfig::priority). A band-k worker
 * services bands 0..k, i.e. its own band and every MORE urgent band. This
 * means a Critical job can be picked up by any worker, while a band-k worker
 * never runs work less urgent than band k - with one exception: the deepest
 * (least urgent) configured band's workers service EVERY band, so that no
 * band is unreachable when the configuration has no Low-band worker (see
 * Config::band_worker_counts). The latency guarantee this buys:
 * because every worker drains band 0 first and the band-0 workers run at the
 * highest OS priority, a newly arrived Critical job - WHEN the Critical queue
 * is otherwise empty - waits at most the remaining duration of one
 * already-running job before a high-OS-priority worker picks it up (and on a
 * preemptive OS - e.g. FreeRTOS or Linux PREEMPT_RT with granted RT
 * scheduling - that worker preempts lower-priority ones the moment it becomes
 * runnable). With a Critical backlog the new job additionally waits behind
 * the earlier Critical jobs, which drain FIFO across all workers first.
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
 * \section thread_pool_ex10 Priority Bands
 * \snippet thread_pool_example.cpp priority bands example
 */
class ThreadPool : public espp::BaseComponent {
public:
  /// @brief A callable job that can be submitted to the pool.
  using Job = std::function<void(void)>;

  /// @brief Number of priority bands (see espp::QosBand).
  static constexpr std::size_t kNumBands = kNumQosBands;

  /// @brief Snapshot of pool activity counters.
  struct Stats {
    std::uint64_t submitted = 0; ///< Total jobs accepted into the queue.
    std::uint64_t executed = 0;  ///< Total jobs executed (counted when a worker begins executing
                                 ///< the job, so a job's own side effects always observe it).
    std::uint64_t rejected = 0;  ///< Total jobs rejected (invalid job, stopped/stopping, or queue
                                ///< full) or dropped (due to stop, the enqueued jobs were dropped).
    std::array<std::uint64_t, kNumBands> band_submitted{}; ///< Jobs accepted per band (by the band
                                                           ///< they were submitted to).
    std::array<std::uint64_t, kNumBands> band_executed{};  ///< Jobs executed per band (by the band
                                                           ///< they were popped from, i.e. after
                                                           ///< any aging promotions).
    std::array<std::uint64_t, kNumBands> band_aged{}; ///< Aging promotions OUT of each band (an
                                                      ///< entry moved from band i to band i-1).
  };

  /// @brief Configuration parameters for constructing a ThreadPool.
  struct Config {
    std::size_t worker_count = 1;   ///< Number of worker threads to spawn (ignored when
                                    ///< band_worker_counts is set - see below).
    std::size_t max_queue_size = 0; ///< Maximum pending jobs TOTAL across all bands (0 =
                                    ///< unbounded).
    bool auto_start = true;         ///< Start workers immediately on construction.
    bool block_on_submit_when_full =
        false; ///< If true, submit() blocks when the queue is full instead of rejecting.
    std::chrono::milliseconds aging_threshold{
        100}; ///< Starvation guard: a queued job whose wait exceeds this is promoted up one band
              ///< (approximate, front-of-band only - see the class docs). 0 disables aging
              ///< (strict band priority).
    std::array<std::size_t, kNumBands> band_worker_counts{}; ///< Opt-in per-band worker counts
                                                             ///< (index = QosBand). All zero (the
                                                             ///< default) = disabled: worker_count
                                                             ///< identical workers service all
                                                             ///< bands. When any element is
                                                             ///< non-zero, band k gets
                                                             ///< band_worker_counts[k] workers at
                                                             ///< band_task_priorities[k], each
                                                             ///< servicing bands 0..k. The
                                                             ///< deepest configured band's
                                                             ///< workers service ALL bands (with
                                                             ///< a warning if that band is not
                                                             ///< Low), so every band is always
                                                             ///< reachable even with
                                                             ///< aging_threshold == 0.
    std::array<std::size_t, kNumBands> band_task_priorities{
        10, 7, 5, 1}; ///< espp::Task priorities for per-band workers (only used when
                      ///< band_worker_counts is set). Defaults descend from Critical to Low. On
                      ///< ESP these are FreeRTOS priorities and are always applied; on host
                      ///< platforms (Linux/macOS) they map to SCHED_FIFO real-time priorities
                      ///< (see espp::Task::BaseConfig::priority) and are only applied when
                      ///< band_workers_realtime is set.
    bool band_workers_realtime =
        false; ///< Opt-in for OS real-time scheduling of per-band workers on HOST platforms
               ///< (sets espp::Task::BaseConfig::host_realtime on each worker). When false (the
               ///< default), host per-band workers run at default (non-realtime)
               ///< scheduling: band ordering is still honored at the queue level (workers pop
               ///< the most urgent band first), but the OS scheduler does not preempt in favor
               ///< of the more urgent bands' workers. When true, band_task_priorities are
               ///< applied as SCHED_FIFO real-time priorities.
               ///< @warning SCHED_FIFO workers can starve the rest of the system if a job spins;
               ///< on Linux this additionally requires CAP_SYS_NICE or an RLIMIT_RTPRIO
               ///< allowance (e.g. under PREEMPT_RT), otherwise the Task falls back to default
               ///< scheduling with a one-time warning. Ignored on ESP, where FreeRTOS
               ///< priorities are always applied.
    espp::Task::BaseConfig worker_task_config = {
        ///< Base configuration applied to every worker task. (For per-band workers the priority
        ///< field is overridden by band_task_priorities.)
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

  /// @brief Submit a job at QosBand::Normal, optionally blocking when the queue is full.
  ///
  /// Blocks if Config::block_on_submit_when_full is true and the queue has
  /// reached its capacity limit. Otherwise behaves identically to try_submit().
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @return true if the job was accepted, false if it was rejected.
  bool submit(Job &&job);

  /// @brief Submit a job at the given priority band, optionally blocking when the queue is full.
  ///
  /// Blocks if Config::block_on_submit_when_full is true and the queue has
  /// reached its capacity limit. Otherwise behaves identically to try_submit().
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @param band Priority band to enqueue the job at (see espp::QosBand).
  /// @return true if the job was accepted, false if it was rejected.
  bool submit(Job &&job, QosBand band);

  /// @brief Attempt to submit a job at QosBand::Normal without blocking.
  ///
  /// Returns immediately with false when the queue is full.
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @return true if the job was accepted, false if it was rejected.
  bool try_submit(Job &&job);

  /// @brief Attempt to submit a job at the given priority band without blocking.
  ///
  /// Returns immediately with false when the queue is full.
  /// @param job Callable to enqueue; moved into the queue on acceptance.
  /// @param band Priority band to enqueue the job at (see espp::QosBand).
  /// @return true if the job was accepted, false if it was rejected.
  bool try_submit(Job &&job, QosBand band);

  /// @brief Return the number of jobs currently waiting in the queue (all bands).
  /// @return Pending job count.
  std::size_t queue_size() const;

  /// @brief Return the number of worker threads in the pool.
  /// @return Worker thread count.
  std::size_t worker_count() const;

  /// @brief Return a snapshot of the pool's activity counters.
  /// @return Stats struct with total and per-band submitted / executed / rejected / aged counts.
  Stats stats() const;

private:
  /// @brief An enqueued job plus its enqueue timestamp (for aging).
  struct Entry {
    Job job;
    std::chrono::steady_clock::time_point enqueued_at{};
  };

  bool worker_task_fn(std::size_t max_band);

  bool submit_impl(Job &&job, QosBand band, bool allow_blocking_when_full);

  /// Promote the front entry of any band whose wait exceeds the aging
  /// threshold up one band. Must be called with queue_mutex_ held.
  /// @return true if any entry was promoted.
  bool age_bands_locked();

  Config config_;
  bool per_band_workers_{false}; ///< True when Config::band_worker_counts is in use.

  std::mutex lifecycle_mutex_;
  mutable std::mutex queue_mutex_;
  std::condition_variable queue_has_work_cv_;
  std::condition_variable queue_has_space_cv_;
  std::array<std::deque<Entry>, kNumBands> queues_; ///< One FIFO per band; index = QosBand.
  std::size_t total_queued_{0};                     ///< Sum of all band queue sizes.

  std::vector<std::unique_ptr<espp::Task>> workers_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stopping_{false};

  std::atomic<std::uint64_t> submitted_{0};
  std::atomic<std::uint64_t> executed_{0};
  std::atomic<std::uint64_t> rejected_{0};
  std::array<std::atomic<std::uint64_t>, kNumBands> band_submitted_{};
  std::array<std::atomic<std::uint64_t>, kNumBands> band_executed_{};
  std::array<std::atomic<std::uint64_t>, kNumBands> band_aged_{};
};

} // namespace espp

#include "thread_pool_format_helpers.hpp"

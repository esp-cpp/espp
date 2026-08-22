#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "task.hpp"
#include "thread_pool.hpp"

using namespace std::chrono_literals;

namespace {
// Poll `pred` until it returns true or `timeout` elapses; returns the final
// value of pred(). Used for bounded, non-flaky waits on background progress.
template <typename Predicate>
bool wait_until(Predicate &&pred, std::chrono::milliseconds timeout,
                std::chrono::milliseconds interval = std::chrono::milliseconds(1)) {
  auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (pred()) {
      return true;
    }
    std::this_thread::sleep_for(interval);
  }
  return pred();
}
} // namespace

int main() {
  espp::Logger logger({.tag = "ThreadPool Test", .level = espp::Logger::Verbosity::INFO});

  int total_passed = 0;
  int total_tests = 0;

  auto check = [&](bool condition, const std::string &desc) -> bool {
    ++total_tests;
    if (condition) {
      ++total_passed;
      logger.info("  PASS: {}", desc);
    } else {
      logger.error("  FAIL: {}", desc);
    }
    return condition;
  };

  // ---------------------------------------------------------------------------
  // 1. Lifecycle: start / stop / is_running / worker_count
  // ---------------------------------------------------------------------------
  logger.info("--- lifecycle ---");
  {
    espp::ThreadPool pool({
        .worker_count = 2,
        .auto_start = false,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });
    check(!pool.is_running(), "not running before start()");
    check(pool.worker_count() == 2, "worker_count() == 2");
    check(pool.start(), "start() returns true");
    check(pool.is_running(), "is_running() after start()");
    check(pool.start(), "duplicate start() is no-op, returns true");
    pool.stop();
    check(!pool.is_running(), "not running after stop()");
  }

  // ---------------------------------------------------------------------------
  // 2. submit() + queue_size() + stats()
  // ---------------------------------------------------------------------------
  logger.info("--- submit + stats ---");
  {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<int> done{0};
    constexpr int N = 6;

    espp::ThreadPool pool({
        .worker_count = 2,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    for (int i = 0; i < N; ++i) {
      pool.submit(espp::ThreadPool::Job([&]() {
        std::this_thread::sleep_for(20ms);
        ++done;
        cv.notify_one();
      }));
    }
    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&] { return done.load() >= N; });
    }

    auto s = pool.stats();
    logger.info("  stats: submitted={} executed={} rejected={}", s.submitted, s.executed,
                s.rejected);
    check(s.submitted == N, "submitted == N");
    check(s.executed == N, "executed == N");
    check(s.rejected == 0, "rejected == 0");
    check(pool.queue_size() == 0, "queue empty after all jobs finish");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 3. try_submit() — deterministic rejection when queue is full
  // ---------------------------------------------------------------------------
  logger.info("--- try_submit bounded queue ---");
  {
    std::mutex barrier_mtx;
    std::condition_variable barrier_cv;
    bool release = false;
    std::mutex started_mtx;
    std::condition_variable started_cv;
    std::atomic<int> started{0};

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 2,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    // Submit first job and wait until it is executing (off the queue)
    pool.try_submit(espp::ThreadPool::Job([&]() {
      {
        std::lock_guard<std::mutex> lk(started_mtx);
        ++started;
      }
      started_cv.notify_one();
      std::unique_lock<std::mutex> lk(barrier_mtx);
      barrier_cv.wait(lk, [&] { return release; });
    }));
    {
      std::unique_lock<std::mutex> lk(started_mtx);
      started_cv.wait(lk, [&] { return started.load() >= 1; });
    }

    // Fill the 2-slot queue
    for (int i = 0; i < 2; ++i) {
      pool.try_submit(espp::ThreadPool::Job([&]() {
        std::unique_lock<std::mutex> lk(barrier_mtx);
        barrier_cv.wait(lk, [&] { return release; });
      }));
    }

    // Queue now provably full — every further try_submit must be rejected
    int rejected = 0;
    for (int i = 0; i < 3; ++i) {
      if (!pool.try_submit(espp::ThreadPool::Job([] {}))) {
        ++rejected;
      }
    }
    check(rejected == 3, "3 try_submit calls rejected when queue full");
    check(pool.stats().rejected == 3, "stats.rejected == 3");

    {
      std::lock_guard<std::mutex> lk(barrier_mtx);
      release = true;
    }
    barrier_cv.notify_all();
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 4. submit() after stop() — rejected
  // ---------------------------------------------------------------------------
  logger.info("--- submit: rejected after stop() ---");
  {
    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });
    pool.stop();
    check(!pool.submit(espp::ThreadPool::Job([] {})), "submit after stop() returns false");
    check(pool.stats().rejected == 1, "stats.rejected == 1");
  }

  // ---------------------------------------------------------------------------
  // 5. Concurrent submit + try_submit from multiple producer threads
  // ---------------------------------------------------------------------------
  logger.info("--- concurrent: multi-thread submit and try_submit ---");
  {
    std::mutex mtx;
    std::condition_variable cv;
    constexpr int num_threads = 4;
    constexpr int jobs_per_thread = 10;
    constexpr int total = num_threads * jobs_per_thread;
    std::atomic<int> done{0};

    espp::ThreadPool pool({
        .worker_count = 3,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    std::vector<std::thread> producers;
    producers.reserve(num_threads);
    for (int t = 0; t < num_threads; ++t) {
      producers.emplace_back([&]() {
        for (int i = 0; i < jobs_per_thread; ++i) {
          pool.submit(espp::ThreadPool::Job([&]() {
            std::this_thread::sleep_for(5ms);
            ++done;
            cv.notify_one();
          }));
        }
      });
    }
    for (auto &p : producers)
      p.join();

    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&] { return done.load() >= total; });
    }

    auto s = pool.stats();
    logger.info("  stats: submitted={} executed={} rejected={}", s.submitted, s.executed,
                s.rejected);
    check(s.submitted == total, "all concurrent submissions accepted");
    check(s.executed == total, "all submitted jobs executed");
    check(s.rejected == 0, "no jobs rejected (unbounded queue)");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 6. Blocking submit when full (block_on_submit_when_full = true)
  // ---------------------------------------------------------------------------
  logger.info("--- submit: blocking when queue full ---");
  {
    std::mutex mtx;
    std::condition_variable cv;
    std::atomic<int> done{0};
    constexpr int total = 6;

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 2,
        .auto_start = true,
        .block_on_submit_when_full = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    int accepted = 0;
    for (int i = 0; i < total; ++i) {
      if (pool.submit(espp::ThreadPool::Job([&]() {
            std::this_thread::sleep_for(30ms);
            ++done;
            cv.notify_one();
          }))) {
        ++accepted;
      }
    }
    check(accepted == total, "all jobs accepted (blocking submit)");

    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&] { return done.load() >= total; });
    }

    auto s = pool.stats();
    logger.info("  stats: submitted={} executed={} rejected={}", s.submitted, s.executed,
                s.rejected);
    check(s.submitted == total, "submitted == total");
    check(s.executed == total, "executed == total");
    check(s.rejected == 0, "rejected == 0");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 7. Concurrent start/stop from multiple threads
  // ---------------------------------------------------------------------------
  logger.info("--- concurrent: start/stop from multiple threads ---");
  {
    espp::ThreadPool pool({
        .worker_count = 2,
        .auto_start = false,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    constexpr int num_threads = 4;
    constexpr int iterations = 10;
    std::vector<std::thread> threads;
    threads.reserve(num_threads);
    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&pool, t]() {
        for (int i = 0; i < iterations; ++i) {
          if ((t + i) % 2 == 0)
            pool.start();
          else
            pool.stop();
        }
      });
    }
    for (auto &t : threads)
      t.join();

    pool.stop();
    check(!pool.is_running(), "pool reaches a clean stopped state");
    auto s = pool.stats();
    logger.info("  stats: submitted={} executed={} rejected={}", s.submitted, s.executed,
                s.rejected);
    check(s.submitted == 0 && s.executed == 0 && s.rejected == 0,
          "stats all zero (no jobs submitted)");
  }

  // ---------------------------------------------------------------------------
  // 8. Chained pools: a job in pool_a submits work to pool_b
  // ---------------------------------------------------------------------------
  logger.info("--- chained: job in pool_a submits to pool_b ---");
  {
    std::mutex mtx;
    std::condition_variable cv;
    constexpr int num_a_jobs = 5;
    constexpr int b_jobs_per_a = 2;
    constexpr int total_b = num_a_jobs * b_jobs_per_a;
    std::atomic<int> done_b{0};

    espp::ThreadPool pool_b({
        .worker_count = 2,
        .auto_start = true,
        .worker_task_config =
            {.name = "pool_b_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });
    espp::ThreadPool pool_a({
        .worker_count = 2,
        .auto_start = true,
        .worker_task_config =
            {.name = "pool_a_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    for (int i = 0; i < num_a_jobs; ++i) {
      pool_a.submit(espp::ThreadPool::Job([&pool_b, &done_b, &cv]() {
        for (int j = 0; j < b_jobs_per_a; ++j) {
          pool_b.submit(espp::ThreadPool::Job([&done_b, &cv]() {
            std::this_thread::sleep_for(20ms);
            ++done_b;
            cv.notify_one();
          }));
        }
      }));
    }

    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&] { return done_b.load() >= total_b; });
    }

    auto sa = pool_a.stats();
    auto sb = pool_b.stats();
    logger.info("  pool_a stats: submitted={} executed={} rejected={}", sa.submitted, sa.executed,
                sa.rejected);
    logger.info("  pool_b stats: submitted={} executed={} rejected={}", sb.submitted, sb.executed,
                sb.rejected);
    check(sa.executed == num_a_jobs, "pool_a executes all A jobs");
    check(sb.executed == total_b, "pool_b executes all chained B jobs");
    check(sb.rejected == 0, "pool_b rejects no jobs");
    pool_a.stop();
    pool_b.stop();
  }

  // ---------------------------------------------------------------------------
  // 9. Self-submit: a job submits another job back to the same pool
  // ---------------------------------------------------------------------------
  logger.info("--- self-submit: job submits to its own pool ---");
  {
    std::mutex mtx;
    std::condition_variable cv;
    constexpr int num_initial = 4;
    constexpr int total = num_initial * 2; // each initial job spawns one follow-up
    std::atomic<int> done{0};

    espp::ThreadPool pool({
        .worker_count = 2,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    for (int i = 0; i < num_initial; ++i) {
      pool.submit(espp::ThreadPool::Job([&pool, &done, &cv]() {
        ++done;
        cv.notify_one();
        pool.submit(espp::ThreadPool::Job([&done, &cv]() {
          std::this_thread::sleep_for(10ms);
          ++done;
          cv.notify_one();
        }));
      }));
    }

    {
      std::unique_lock<std::mutex> lk(mtx);
      cv.wait(lk, [&] { return done.load() >= total; });
    }

    auto s = pool.stats();
    logger.info("  stats: submitted={} executed={} rejected={}", s.submitted, s.executed,
                s.rejected);
    check(s.submitted == total, "all initial + follow-up jobs submitted");
    check(s.executed == total, "all initial + follow-up jobs executed");
    check(s.rejected == 0, "no jobs rejected");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 10. Task host priority: start()/set_priority() succeed without privileges
  // ---------------------------------------------------------------------------
  logger.info("--- task: host priority application (graceful fallback) ---");
  {
    std::atomic<int> iterations{0};
    espp::Task task({.callback = [&]() -> bool {
                       ++iterations;
                       std::this_thread::sleep_for(1ms);
                       return false;
                     },
                     .task_config = {.name = "prio_task",
                                     .stack_size_bytes = 4096,
                                     .priority = 10,
                                     .core_id = -1,
                                     .host_realtime = true}});
    check(task.get_configured_priority() == 10, "configured priority round-trips from config");
    // Must succeed even when RT scheduling is not permitted (unprivileged CI):
    // the priority application falls back gracefully and never fails start().
    check(task.start(), "start() succeeds with an RT-range priority, unprivileged");
    check(wait_until([&] { return iterations.load() > 0; }, 2s), "task callback runs");
    // Live priority changes are best-effort (return value depends on platform
    // privileges); the stored value must always round-trip.
    task.set_priority(3);
    check(task.get_configured_priority() == 3, "set_priority(3) round-trips while running");
    task.set_priority(0); // demote back to default scheduling
    check(task.get_configured_priority() == 0, "set_priority(0) round-trips while running");
    check(task.stop(), "task stops cleanly");
  }

  // ---------------------------------------------------------------------------
  // 11. Priority ordering: Critical overtakes queued Low jobs (strict priority)
  // ---------------------------------------------------------------------------
  logger.info("--- priority: Critical overtakes queued Low ---");
  {
    std::mutex gate_mtx;
    std::condition_variable gate_cv;
    bool release = false;
    std::atomic<int> blocker_started{0};
    std::atomic<int> done{0};
    std::mutex order_mtx;
    std::vector<std::string> order;

    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        .aging_threshold = 0ms, // strict band priority for determinism
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    // Gate the single worker so submission order is fully deterministic.
    pool.submit(espp::ThreadPool::Job([&]() {
      ++blocker_started;
      std::unique_lock<std::mutex> lk(gate_mtx);
      gate_cv.wait(lk, [&] { return release; });
    }));
    check(wait_until([&] { return blocker_started.load() >= 1; }, 2s), "blocker job is executing");

    for (int i = 0; i < 3; ++i) {
      pool.submit(espp::ThreadPool::Job([&, i]() {
                    std::lock_guard<std::mutex> lk(order_mtx);
                    order.push_back("low" + std::to_string(i));
                    ++done;
                  }),
                  espp::QosBand::Low);
    }
    pool.submit(espp::ThreadPool::Job([&]() {
                  std::lock_guard<std::mutex> lk(order_mtx);
                  order.push_back("critical");
                  ++done;
                }),
                espp::QosBand::Critical);

    {
      std::lock_guard<std::mutex> lk(gate_mtx);
      release = true;
    }
    gate_cv.notify_all();
    check(wait_until([&] { return done.load() >= 4; }, 2s), "all banded jobs completed");
    {
      std::lock_guard<std::mutex> lk(order_mtx);
      check(order.size() == 4 && order[0] == "critical",
            "Critical job ran before the queued Low jobs");
      check(order.size() == 4 && order[1] == "low0" && order[2] == "low1" && order[3] == "low2",
            "Low jobs kept FIFO order within their band");
    }
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 12. Default-band equivalence: no-band submits behave FIFO exactly as before
  // ---------------------------------------------------------------------------
  logger.info("--- priority: default band preserves FIFO ---");
  {
    std::mutex gate_mtx;
    std::condition_variable gate_cv;
    bool release = false;
    std::atomic<int> blocker_started{0};
    std::atomic<int> done{0};
    std::mutex order_mtx;
    std::vector<int> order;
    constexpr int N = 10;

    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    pool.submit(espp::ThreadPool::Job([&]() {
      ++blocker_started;
      std::unique_lock<std::mutex> lk(gate_mtx);
      gate_cv.wait(lk, [&] { return release; });
    }));
    check(wait_until([&] { return blocker_started.load() >= 1; }, 2s), "blocker job is executing");

    for (int i = 0; i < N; ++i) {
      pool.submit(espp::ThreadPool::Job([&, i]() {
        std::lock_guard<std::mutex> lk(order_mtx);
        order.push_back(i);
        ++done;
      }));
    }
    {
      std::lock_guard<std::mutex> lk(gate_mtx);
      release = true;
    }
    gate_cv.notify_all();
    check(wait_until([&] { return done.load() >= N; }, 2s), "all no-band jobs completed");
    {
      std::lock_guard<std::mutex> lk(order_mtx);
      bool fifo = order.size() == N;
      for (int i = 0; fifo && i < N; ++i) {
        fifo = (order[i] == i);
      }
      check(fifo, "no-band submits execute in exact FIFO submission order");
    }
    auto s = pool.stats();
    check(s.band_submitted[static_cast<size_t>(espp::QosBand::Normal)] == N + 1,
          "no-band submits are accounted to the Normal band");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 13. Aging: a Low job runs under a continuous stream of Normal jobs
  // ---------------------------------------------------------------------------
  logger.info("--- priority: aging rescues a Low job under Normal load ---");
  {
    std::mutex gate_mtx;
    std::condition_variable gate_cv;
    bool release = false;
    std::atomic<int> blocker_started{0};
    std::atomic<bool> low_done{false};
    std::atomic<int> normals_done{0};
    std::atomic<bool> stop_stream{false};

    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        .aging_threshold = 20ms,
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    // Gate the worker so the Low job provably queues BEHIND a busy Normal band
    // (an idle worker would otherwise pick the Low job up immediately).
    pool.submit(espp::ThreadPool::Job([&]() {
      ++blocker_started;
      std::unique_lock<std::mutex> lk(gate_mtx);
      gate_cv.wait(lk, [&] { return release; });
    }));
    check(wait_until([&] { return blocker_started.load() >= 1; }, 2s), "blocker job is executing");

    // The Low job goes in first, plus a primed backlog of Normal jobs longer
    // than the aging threshold (5 x 5ms > 20ms)...
    pool.submit(espp::ThreadPool::Job([&]() { low_done = true; }), espp::QosBand::Low);
    auto normal_job = espp::ThreadPool::Job([&]() {
      std::this_thread::sleep_for(5ms);
      ++normals_done;
    });
    for (int i = 0; i < 5; ++i) {
      pool.submit(espp::ThreadPool::Job(normal_job));
    }
    // ...then more Normal jobs are produced faster than the single worker
    // consumes them, so the Normal band never empties. Without aging the Low
    // job would starve indefinitely; with aging it must run in bounded time.
    std::thread producer([&]() {
      while (!stop_stream.load()) {
        pool.submit(espp::ThreadPool::Job(normal_job));
        std::this_thread::sleep_for(2ms);
      }
    });
    {
      std::lock_guard<std::mutex> lk(gate_mtx);
      release = true;
    }
    gate_cv.notify_all();

    bool rescued = wait_until([&] { return low_done.load(); }, 5s, 5ms);
    stop_stream = true;
    producer.join();
    check(rescued, "aged Low job ran while the Normal stream was still active");
    check(normals_done.load() > 0, "Normal stream made progress concurrently");
    auto s = pool.stats();
    logger.info("  stats: {}", s);
    check(s.band_aged[static_cast<size_t>(espp::QosBand::Low)] >= 1,
          "stats recorded an aging promotion out of the Low band");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 14. Per-band stats
  // ---------------------------------------------------------------------------
  logger.info("--- priority: per-band stats ---");
  {
    std::mutex gate_mtx;
    std::condition_variable gate_cv;
    bool release = false;
    std::atomic<int> blocker_started{0};
    std::atomic<int> done{0};

    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        .aging_threshold = 0ms, // keep jobs in their submitted bands
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });

    pool.submit(espp::ThreadPool::Job([&]() { // Normal-band blocker
      ++blocker_started;
      std::unique_lock<std::mutex> lk(gate_mtx);
      gate_cv.wait(lk, [&] { return release; });
    }));
    check(wait_until([&] { return blocker_started.load() >= 1; }, 2s), "blocker job is executing");

    auto count_done = espp::ThreadPool::Job([&]() { ++done; });
    pool.submit(espp::ThreadPool::Job(count_done), espp::QosBand::Critical);
    pool.submit(espp::ThreadPool::Job(count_done), espp::QosBand::High);
    pool.submit(espp::ThreadPool::Job(count_done), espp::QosBand::Normal);
    pool.submit(espp::ThreadPool::Job(count_done), espp::QosBand::Low);

    {
      std::lock_guard<std::mutex> lk(gate_mtx);
      release = true;
    }
    gate_cv.notify_all();
    check(wait_until([&] { return done.load() >= 4; }, 2s), "one job per band completed");

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    check(s.band_submitted[0] == 1 && s.band_submitted[1] == 1 && s.band_submitted[2] == 2 &&
              s.band_submitted[3] == 1,
          "band_submitted counts each band (blocker is Normal)");
    check(s.band_executed == s.band_submitted, "band_executed matches band_submitted (no aging)");
    check(s.band_aged[0] == 0 && s.band_aged[1] == 0 && s.band_aged[2] == 0 && s.band_aged[3] == 0,
          "no aging promotions recorded with aging disabled");
    check(s.submitted == 5 && s.executed == 5 && s.rejected == 0, "totals consistent");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // 15. Worker bands: per-band workers, mixed load, nothing lost
  // ---------------------------------------------------------------------------
  logger.info("--- priority: per-band workers (worker bands) ---");
  {
    constexpr int num_critical = 20;
    constexpr int num_normal = 40;
    constexpr int num_low = 20;
    constexpr int total = num_critical + num_normal + num_low;
    std::atomic<int> done{0};
    std::mutex lat_mtx;
    std::vector<std::chrono::microseconds> critical_latencies;
    std::vector<std::chrono::microseconds> low_latencies;

    espp::ThreadPool pool({
        .auto_start = true,
        .band_worker_counts = {{1, 1, 2, 1}}, // 1 Critical, 1 High, 2 Normal, 1 Low worker
        .band_workers_realtime = true, // exercise the host SCHED_FIFO path (best-effort; falls
                                       // back gracefully without CAP_SYS_NICE/RLIMIT_RTPRIO)
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });
    check(pool.worker_count() == 5, "per-band worker counts create 1+1+2+1 workers");

    auto submit_timed = [&](espp::QosBand band, std::vector<std::chrono::microseconds> *lat) {
      auto t0 = std::chrono::steady_clock::now();
      pool.submit(espp::ThreadPool::Job([&, t0, lat]() {
                    auto waited = std::chrono::duration_cast<std::chrono::microseconds>(
                        std::chrono::steady_clock::now() - t0);
                    if (lat != nullptr) {
                      std::lock_guard<std::mutex> lk(lat_mtx);
                      lat->push_back(waited);
                    }
                    std::this_thread::sleep_for(2ms);
                    ++done;
                  }),
                  band);
    };

    // Interleave the load: mostly Normal with periodic Critical / Low.
    int c = 0, n = 0, l = 0;
    while (c < num_critical || n < num_normal || l < num_low) {
      if (n < num_normal) {
        submit_timed(espp::QosBand::Normal, nullptr);
        ++n;
      }
      if ((n % 2) == 0 && c < num_critical) {
        submit_timed(espp::QosBand::Critical, &critical_latencies);
        ++c;
      }
      if ((n % 2) == 1 && l < num_low) {
        submit_timed(espp::QosBand::Low, &low_latencies);
        ++l;
      }
    }

    check(wait_until([&] { return done.load() >= total; }, 10s, 5ms),
          "all jobs completed on per-band workers (none lost)");
    auto s = pool.stats();
    logger.info("  stats: {}", s);
    check(s.submitted == total && s.executed == total && s.rejected == 0,
          "stats: everything submitted was executed, nothing rejected");
    {
      std::lock_guard<std::mutex> lk(lat_mtx);
      auto max_of = [](const std::vector<std::chrono::microseconds> &v) {
        return v.empty() ? std::chrono::microseconds(0) : *std::max_element(v.begin(), v.end());
      };
      logger.info("  critical: n={} max wait={}us; low: n={} max wait={}us",
                  critical_latencies.size(), max_of(critical_latencies).count(),
                  low_latencies.size(), max_of(low_latencies).count());
      check(critical_latencies.size() == num_critical && low_latencies.size() == num_low,
            "latency recorded for every Critical and Low job");
    }
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // Per-band workers: every band stays reachable even in strict mode
  // ---------------------------------------------------------------------------
  logger.info("--- priority: deepest band worker covers all bands (strict, no aging) ---");
  {
    // Only a Critical worker configured, and aging disabled: without the
    // coverage fallback the Normal and Low submissions below would sit queued
    // forever. The deepest configured band's workers must service every band.
    std::atomic<int> done{0};
    espp::ThreadPool pool({
        .auto_start = true,
        .aging_threshold = std::chrono::milliseconds(0), // strict band priority
        .band_worker_counts = {{1, 0, 0, 0}},            // ONLY a Critical worker
        .worker_task_config =
            {.name = "tp_worker", .stack_size_bytes = 4096, .priority = 5, .core_id = -1},
    });
    check(pool.worker_count() == 1, "single Critical worker created");
    pool.submit(espp::ThreadPool::Job([&]() { ++done; }), espp::QosBand::Critical);
    pool.submit(espp::ThreadPool::Job([&]() { ++done; })); // Normal (default)
    pool.submit(espp::ThreadPool::Job([&]() { ++done; }), espp::QosBand::Low);
    check(wait_until([&] { return done.load() == 3; }, 5s),
          "Critical, Normal, and Low jobs all execute with only a Critical worker (no band "
          "unreachable)");
    pool.stop();
  }

  // ---------------------------------------------------------------------------
  // Summary
  // ---------------------------------------------------------------------------
  logger.info("==================== Results ====================");
  logger.info("{}/{} checks passed", total_passed, total_tests);
  if (total_passed == total_tests) {
    logger.info("All checks passed!");
  } else {
    logger.error("{} check(s) FAILED", total_tests - total_passed);
  }

  return (total_passed == total_tests) ? 0 : 1;
}

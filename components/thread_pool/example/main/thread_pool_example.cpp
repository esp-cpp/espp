#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "logger.hpp"
#include "thread_pool.hpp"

using namespace std::chrono_literals;

static void wait_for_jobs(std::condition_variable &cv, std::mutex &mtx, std::atomic<int> &completed,
                          int expected) {
  std::unique_lock<std::mutex> lock(mtx);
  cv.wait(lock, [&]() { return completed.load() >= expected; });
}

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ThreadPool Example", .level = espp::Logger::Verbosity::INFO});

  struct TestResult {
    std::string name;
    bool passed{false};
  };
  std::vector<TestResult> results;

  // Returns false and logs on failure; used to accumulate per-test pass/fail.
  auto check = [&](const std::string &test, bool condition, const std::string &desc) -> bool {
    if (condition) {
      logger.info("  PASS [{}]: {}", test, desc);
    } else {
      logger.error("  FAIL [{}]: {}", test, desc);
    }
    return condition;
  };

  // -------------------------------------------------------------------------
  // 1. Manual start/stop + is_running() + worker_count()
  // -------------------------------------------------------------------------
  {
    const std::string name = "lifecycle: start/stop/is_running/worker_count";
    //! [lifecycle example]
    logger.info("--- {} ---", name);
    bool passed = true;

    espp::ThreadPool pool({
        .worker_count = 3,
        .max_queue_size = 0,
        .auto_start = false,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    passed &= check(name, !pool.is_running(), "pool should not be running before start()");
    passed &= check(name, pool.worker_count() == 3, "worker_count() should be 3");

    passed &= check(name, pool.start(), "start() should return true on first call");
    passed &= check(name, pool.is_running(), "pool should be running after start()");

    passed &= check(name, pool.start(), "start() should return true when already running (no-op)");
    passed &=
        check(name, pool.is_running(), "pool should still be running after duplicate start()");

    pool.stop();
    passed &= check(name, !pool.is_running(), "pool should not be running after stop()");
    //! [lifecycle example]

    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 2. submit() + queue_size() + stats()
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: normal dispatch + queue_size + stats";
    //! [submit example]
    logger.info("--- {} ---", name);
    bool passed = true;

    std::mutex done_mutex;
    std::condition_variable done_cv;
    constexpr int total_jobs = 8;
    std::atomic<int> completed_jobs{0};

    espp::ThreadPool pool({
        .worker_count = 2,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    int accepted_count = 0;
    for (int i = 0; i < total_jobs; ++i) {
      if (pool.submit([&, i]() {
            std::this_thread::sleep_for(50ms);
            ++completed_jobs;
            done_cv.notify_one();
          })) {
        ++accepted_count;
      }
    }
    passed &=
        check(name, accepted_count == total_jobs, "all jobs should be accepted (unbounded queue)");

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted == total_jobs, "submitted count should equal total_jobs");
    passed &= check(name, s.executed == total_jobs, "executed count should equal total_jobs");
    passed &= check(name, s.rejected == 0, "rejected count should be 0");
    passed &= check(name, pool.queue_size() == 0, "queue should be empty after all jobs finish");

    pool.stop();
    //! [submit example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 3. try_submit() — non-blocking rejection when queue is full
  // -------------------------------------------------------------------------
  {
    const std::string name = "try_submit: rejection when queue full";
    //! [try_submit example]
    logger.info("--- {} ---", name);
    bool passed = true;

    // 1 worker, queue capacity 2: 1 executing + 2 queued = 3 total slots.
    //
    // Jobs are gated by an explicit barrier so workers cannot drain the queue
    // before we assert rejection, making the test fully deterministic.
    //
    // To guarantee the queue is provably full we must also ensure the worker
    // has dequeued (and is executing) the first job before we fill the two
    // remaining queue slots.  We use a separate "started" CV for this.
    std::mutex barrier_mutex;
    std::condition_variable barrier_cv;
    bool release_workers = false;

    std::mutex started_mutex;
    std::condition_variable started_cv;
    std::atomic<int> jobs_started{0};

    auto blocking_job = [&]() {
      // Signal that this job is now executing (off the queue)
      {
        std::lock_guard<std::mutex> lock(started_mutex);
        ++jobs_started;
      }
      started_cv.notify_one();
      // Block until the test releases the barrier
      std::unique_lock<std::mutex> lock(barrier_mutex);
      barrier_cv.wait(lock, [&]() { return release_workers; });
    };

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 2,
        .auto_start = true,
        .block_on_submit_when_full = false,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Step 1: submit the first job and wait until it is executing
    //         (it has been removed from the queue by the worker).
    int fill_accepted = 0;
    if (pool.try_submit(blocking_job)) {
      ++fill_accepted;
    }
    {
      std::unique_lock<std::mutex> lock(started_mutex);
      started_cv.wait(lock, [&]() { return jobs_started.load() >= 1; });
    }

    // Step 2: fill the 2 remaining queue slots.
    for (int i = 0; i < 2; ++i) {
      if (pool.try_submit(blocking_job)) {
        ++fill_accepted;
      }
    }
    passed &= check(name, fill_accepted == 3, "first 3 try_submit calls should be accepted");

    // Step 3: queue is now provably full — every additional try_submit must be rejected.
    int rejected_count = 0;
    for (int i = 0; i < 3; ++i) {
      if (!pool.try_submit([&]() {})) {
        ++rejected_count;
      }
    }
    passed &= check(name, rejected_count == 3, "try_submit when full should return false");

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.rejected == 3, "stats.rejected should be 3");

    // Release the barrier so workers can finish, then stop cleanly.
    {
      std::lock_guard<std::mutex> lock(barrier_mutex);
      release_workers = true;
    }
    barrier_cv.notify_all();
    pool.stop();
    //! [try_submit example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 4. submit() blocking when full (block_on_submit_when_full = true)
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: blocking when queue full";
    //! [blocking submit example]
    logger.info("--- {} ---", name);
    bool passed = true;

    std::mutex done_mutex;
    std::condition_variable done_cv;
    std::atomic<int> completed_jobs{0};
    constexpr int total_jobs = 6;

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 2,
        .auto_start = true,
        .block_on_submit_when_full = true,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    int accepted_count = 0;
    for (int i = 0; i < total_jobs; ++i) {
      if (pool.submit([&, i]() {
            std::this_thread::sleep_for(30ms);
            ++completed_jobs;
            done_cv.notify_one();
          })) {
        ++accepted_count;
      }
    }
    passed &=
        check(name, accepted_count == total_jobs, "all jobs should be accepted (blocking submit)");

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted == total_jobs, "submitted count should equal total_jobs");
    passed &= check(name, s.executed == total_jobs, "executed count should equal total_jobs");
    passed &= check(name, s.rejected == 0, "rejected count should be 0");

    pool.stop();
    //! [blocking submit example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 5. submit() after stop() — rejected via is_running() guard
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: rejected after stop()";
    //! [submit after stop example]
    logger.info("--- {} ---", name);
    bool passed = true;

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    pool.stop();
    bool accepted = pool.submit([]() {});
    passed &= check(name, !accepted, "submit() after stop() should return false");
    passed &= check(name, pool.stats().submitted == 0, "submitted count should be 0");
    passed &= check(name, pool.stats().rejected == 1, "rejected count should be 1");

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    //! [submit after stop example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 6. Concurrent start/stop from multiple threads
  // -------------------------------------------------------------------------
  {
    const std::string name = "concurrent: start/stop from multiple threads";
    //! [concurrent lifecycle example]
    logger.info("--- {} ---", name);
    bool passed = true;

    espp::ThreadPool pool({
        .worker_count = 2,
        .max_queue_size = 0,
        .auto_start = false,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    constexpr int num_threads = 4;
    constexpr int iterations = 10;
    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for (int t = 0; t < num_threads; ++t) {
      threads.emplace_back([&pool, t]() {
        for (int i = 0; i < iterations; ++i) {
          if ((t + i) % 2 == 0) {
            pool.start();
          } else {
            pool.stop();
          }
        }
      });
    }
    for (auto &t : threads) {
      t.join();
    }

    // Bring pool to a known stopped state and verify consistency
    pool.stop();
    passed &= check(name, !pool.is_running(), "pool should reach a clean stopped state");

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    // No jobs were submitted — all counters must be zero
    passed &= check(name, s.submitted == 0 && s.executed == 0 && s.rejected == 0,
                    "stats should all be zero (no jobs submitted)");
    //! [concurrent lifecycle example]

    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 7. Concurrent submit/try_submit from multiple producer threads
  // -------------------------------------------------------------------------
  {
    const std::string name = "concurrent: multi-thread submit and try_submit";
    //! [concurrent submit example]
    logger.info("--- {} ---", name);
    bool passed = true;

    std::mutex done_mutex;
    std::condition_variable done_cv;
    constexpr int num_submit_threads = 3;
    constexpr int num_try_submit_threads = 2;
    constexpr int jobs_per_thread = 10;
    constexpr int total_jobs = (num_submit_threads + num_try_submit_threads) * jobs_per_thread;
    std::atomic<int> completed_jobs{0};
    std::atomic<int> total_accepted{0};

    espp::ThreadPool pool({
        .worker_count = 4,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    std::vector<std::thread> producers;
    producers.reserve(num_submit_threads + num_try_submit_threads);

    // submit() producers
    for (int p = 0; p < num_submit_threads; ++p) {
      producers.emplace_back([&]() {
        for (int i = 0; i < jobs_per_thread; ++i) {
          if (pool.submit([&]() {
                std::this_thread::sleep_for(10ms);
                ++completed_jobs;
                done_cv.notify_one();
              })) {
            ++total_accepted;
          }
        }
      });
    }

    // try_submit() producers
    for (int p = 0; p < num_try_submit_threads; ++p) {
      producers.emplace_back([&]() {
        for (int i = 0; i < jobs_per_thread; ++i) {
          if (pool.try_submit([&]() {
                std::this_thread::sleep_for(10ms);
                ++completed_jobs;
                done_cv.notify_one();
              })) {
            ++total_accepted;
          }
        }
      });
    }

    for (auto &p : producers) {
      p.join();
    }

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_accepted.load());

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted + s.rejected == total_jobs,
                    "submitted + rejected should equal total attempted");
    passed &= check(name, s.executed == s.submitted,
                    "all accepted jobs should be executed (unbounded queue)");
    passed &= check(name, s.rejected == 0, "unbounded queue should not reject any jobs");

    pool.stop();
    //! [concurrent submit example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 8. Chained pools: a job in pool A submits work to pool B
  // -------------------------------------------------------------------------
  {
    const std::string name = "chained: job in pool_a submits to pool_b";
    //! [chained pools example]
    logger.info("--- {} ---", name);
    bool passed = true;

    std::mutex done_mutex;
    std::condition_variable done_cv;
    constexpr int num_a_jobs = 5;
    constexpr int b_jobs_per_a = 2;
    constexpr int total_b_jobs = num_a_jobs * b_jobs_per_a;
    std::atomic<int> completed_b{0};

    espp::ThreadPool pool_b({
        .worker_count = 2,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "pool_b_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    espp::ThreadPool pool_a({
        .worker_count = 2,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "pool_a_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    for (int i = 0; i < num_a_jobs; ++i) {
      pool_a.submit([&pool_b, &completed_b, &done_cv]() {
        for (int j = 0; j < b_jobs_per_a; ++j) {
          pool_b.submit([&completed_b, &done_cv]() {
            std::this_thread::sleep_for(20ms);
            ++completed_b;
            done_cv.notify_one();
          });
        }
      });
    }

    wait_for_jobs(done_cv, done_mutex, completed_b, total_b_jobs);

    auto sa = pool_a.stats();
    auto sb = pool_b.stats();
    logger.info("  pool_a stats: {}", sa);
    logger.info("  pool_b stats: {}", sb);
    passed &= check(name, sa.executed == num_a_jobs, "pool_a should execute all A jobs");
    passed &= check(name, sb.executed == total_b_jobs, "pool_b should execute all chained B jobs");
    passed &= check(name, sb.rejected == 0, "pool_b should not reject any jobs");

    pool_a.stop();
    pool_b.stop();
    //! [chained pools example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 9. Self-submit: a job submits another job back to the same pool
  // -------------------------------------------------------------------------
  {
    const std::string name = "self-submit: job submits to its own pool";
    //! [self-submit example]
    logger.info("--- {} ---", name);
    bool passed = true;

    std::mutex done_mutex;
    std::condition_variable done_cv;
    constexpr int num_initial_jobs = 4;
    // Each initial job submits one follow-up job → 4 initial + 4 follow-up = 8 total
    constexpr int total_executions = num_initial_jobs * 2;
    std::atomic<int> completed{0};

    espp::ThreadPool pool({
        .worker_count = 2,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    for (int i = 0; i < num_initial_jobs; ++i) {
      pool.submit([&pool, &completed, &done_cv]() {
        ++completed;
        done_cv.notify_one();
        // Submit a follow-up job back to the same pool without deadlock
        pool.submit([&completed, &done_cv]() {
          std::this_thread::sleep_for(10ms);
          ++completed;
          done_cv.notify_one();
        });
      });
    }

    wait_for_jobs(done_cv, done_mutex, completed, total_executions);

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted == total_executions,
                    "all initial + follow-up jobs should be submitted");
    passed &= check(name, s.executed == total_executions,
                    "all initial + follow-up jobs should be executed");
    passed &= check(name, s.rejected == 0, "no jobs should be rejected");

    pool.stop();
    //! [self-submit example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 10. Priority bands: Critical jobs overtake queued Low jobs
  // -------------------------------------------------------------------------
  {
    const std::string name = "priority bands: Critical overtakes Low";
    //! [priority bands example]
    logger.info("--- {} ---", name);
    bool passed = true;

    // Gate the single worker on a barrier so the queue contents (and therefore
    // the dispatch order) are fully deterministic.
    std::mutex barrier_mutex;
    std::condition_variable barrier_cv;
    bool release_worker = false;
    std::mutex started_mutex;
    std::condition_variable started_cv;
    std::atomic<int> jobs_started{0};

    std::mutex done_mutex;
    std::condition_variable done_cv;
    std::atomic<int> completed_jobs{0};
    std::mutex order_mutex;
    std::vector<std::string> execution_order;

    espp::ThreadPool pool({
        .worker_count = 1,
        .auto_start = true,
        // 0 = strict band priority; the default (100ms) also promotes
        // long-waiting jobs up one band to prevent starvation ("aging").
        .aging_threshold = std::chrono::milliseconds(0),
        .worker_task_config =
            {
                .name = "tp_worker",
                .stack_size_bytes = 4096,
                .priority = 5,
                .core_id = -1,
            },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Occupy the worker, then queue Low jobs followed by a Critical one.
    pool.submit([&]() {
      {
        std::lock_guard<std::mutex> lock(started_mutex);
        ++jobs_started;
      }
      started_cv.notify_one();
      std::unique_lock<std::mutex> lock(barrier_mutex);
      barrier_cv.wait(lock, [&]() { return release_worker; });
    });
    {
      std::unique_lock<std::mutex> lock(started_mutex);
      started_cv.wait(lock, [&]() { return jobs_started.load() >= 1; });
    }

    for (int i = 0; i < 2; ++i) {
      pool.submit(
          [&, i]() {
            {
              std::lock_guard<std::mutex> lock(order_mutex);
              execution_order.push_back("low" + std::to_string(i));
            }
            ++completed_jobs;
            done_cv.notify_one();
          },
          espp::QosBand::Low);
    }
    pool.submit(
        [&]() {
          {
            std::lock_guard<std::mutex> lock(order_mutex);
            execution_order.push_back("critical");
          }
          ++completed_jobs;
          done_cv.notify_one();
        },
        espp::QosBand::Critical);

    // Release the worker: the Critical job must run before the queued Low jobs.
    {
      std::lock_guard<std::mutex> lock(barrier_mutex);
      release_worker = true;
    }
    barrier_cv.notify_all();
    wait_for_jobs(done_cv, done_mutex, completed_jobs, 3);

    {
      std::lock_guard<std::mutex> lock(order_mutex);
      passed &= check(name, execution_order.size() == 3 && execution_order[0] == "critical",
                      "Critical job should run before the queued Low jobs");
    }
    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.band_submitted[static_cast<size_t>(espp::QosBand::Critical)] == 1,
                    "one job accounted to the Critical band");
    passed &= check(name, s.band_submitted[static_cast<size_t>(espp::QosBand::Low)] == 2,
                    "two jobs accounted to the Low band");

    pool.stop();
    //! [priority bands example]
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // Summary
  // -------------------------------------------------------------------------
  logger.info("==================== Results ====================");
  int total_passed = 0;
  for (const auto &r : results) {
    if (r.passed) {
      logger.info("  PASS  {}", r.name);
      ++total_passed;
    } else {
      logger.error("  FAIL  {}", r.name);
    }
  }
  logger.info("=================================================");
  logger.info("{}/{} tests passed", total_passed, results.size());
  if (total_passed == static_cast<int>(results.size())) {
    logger.info("All tests passed!");
  } else {
    logger.error("{} test(s) FAILED", static_cast<int>(results.size()) - total_passed);
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

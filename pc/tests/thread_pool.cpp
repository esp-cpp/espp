#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "thread_pool.hpp"

using namespace std::chrono_literals;

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

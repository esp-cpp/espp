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
    bool passed;
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
    logger.info("--- {} ---", name);
    bool passed = true;

    espp::ThreadPool pool({
        .worker_count = 3,
        .max_queue_size = 0,
        .auto_start = false,
        .worker_task_config = {
            .name = "tp_worker",
            .stack_size_bytes = 4096,
            .priority = 5,
            .core_id = -1,
        },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    passed &= check(name, !pool.is_running(),       "pool should not be running before start()");
    passed &= check(name, pool.worker_count() == 3, "worker_count() should be 3");

    pool.start();
    passed &= check(name, pool.is_running(), "pool should be running after start()");

    pool.start(); // no-op
    passed &= check(name, pool.is_running(), "pool should still be running after duplicate start()");

    pool.stop();
    passed &= check(name, !pool.is_running(), "pool should not be running after stop()");

    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 2. submit() + queue_size() + stats()
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: normal dispatch + queue_size + stats";
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
        .worker_task_config = {
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
    passed &= check(name, accepted_count == total_jobs, "all jobs should be accepted (unbounded queue)");

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted == total_jobs, "submitted count should equal total_jobs");
    passed &= check(name, s.executed == total_jobs,  "executed count should equal total_jobs");
    passed &= check(name, s.rejected == 0,           "rejected count should be 0");
    passed &= check(name, pool.queue_size() == 0,    "queue should be empty after all jobs finish");

    pool.stop();
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 3. try_submit() — non-blocking rejection when queue is full
  // -------------------------------------------------------------------------
  {
    const std::string name = "try_submit: rejection when queue full";
    logger.info("--- {} ---", name);
    bool passed = true;

    // 1 slow worker, capacity 2: 1 executing + 2 queued = 3 slots before rejection
    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 2,
        .auto_start = true,
        .block_on_submit_when_full = false,
        .worker_task_config = {
            .name = "tp_worker",
            .stack_size_bytes = 4096,
            .priority = 5,
            .core_id = -1,
        },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Fill worker + queue
    int fill_accepted = 0;
    for (int i = 0; i < 3; ++i) {
      if (pool.try_submit([&]() { std::this_thread::sleep_for(300ms); })) {
        ++fill_accepted;
      }
    }
    passed &= check(name, fill_accepted == 3, "first 3 try_submit calls should be accepted");

    // These should all be rejected immediately
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

    pool.stop();
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 4. submit() blocking when full (block_on_submit_when_full = true)
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: blocking when queue full";
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
        .worker_task_config = {
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
    passed &= check(name, accepted_count == total_jobs, "all jobs should be accepted (blocking submit)");

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);

    auto s = pool.stats();
    logger.info("  stats: {}", s);
    passed &= check(name, s.submitted == total_jobs, "submitted count should equal total_jobs");
    passed &= check(name, s.executed == total_jobs,  "executed count should equal total_jobs");
    passed &= check(name, s.rejected == 0,           "rejected count should be 0");

    pool.stop();
    results.push_back({name, passed});
  }

  // -------------------------------------------------------------------------
  // 5. submit() after stop() — rejected via is_running() guard
  // -------------------------------------------------------------------------
  {
    const std::string name = "submit: rejected after stop()";
    logger.info("--- {} ---", name);
    bool passed = true;

    espp::ThreadPool pool({
        .worker_count = 1,
        .max_queue_size = 0,
        .auto_start = true,
        .worker_task_config = {
            .name = "tp_worker",
            .stack_size_bytes = 4096,
            .priority = 5,
            .core_id = -1,
        },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    pool.stop();
    bool accepted = pool.submit([]() {});
    passed &= check(name, !accepted,                   "submit() after stop() should return false");
    passed &= check(name, pool.stats().submitted == 0, "submitted count should be 0");
    passed &= check(name, pool.stats().rejected == 1,  "rejected count should be 1");

    auto s = pool.stats();
    logger.info("  stats: {}", s);
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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

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

  // -------------------------------------------------------------------------
  // 1. Manual start/stop + is_running() + worker_count()
  // -------------------------------------------------------------------------
  logger.info("--- Test: auto_start=false, manual start() / stop() ---");
  {
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

    logger.info("is_running before start: {}", pool.is_running());  // false
    logger.info("worker_count: {}", pool.worker_count());           // 3

    pool.start();
    logger.info("is_running after start: {}", pool.is_running());   // true

    // start() is a no-op when already running
    pool.start();
    logger.info("is_running after second start: {}", pool.is_running()); // true

    pool.stop();
    logger.info("is_running after stop: {}", pool.is_running());    // false
  }

  // -------------------------------------------------------------------------
  // 2. submit() — normal job dispatch, queue_size()
  // -------------------------------------------------------------------------
  logger.info("--- Test: submit() and queue_size() ---");
  {
    std::mutex done_mutex;
    std::condition_variable done_cv;
    constexpr int total_jobs = 8;
    std::atomic<int> completed_jobs = 0;

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

    for (int i = 0; i < total_jobs; ++i) {
      bool accepted = pool.submit([&, i]() {
        std::this_thread::sleep_for(50ms);
        int done = ++completed_jobs;
        logger.info("Job {} done ({}/{})", i, done, total_jobs);
        done_cv.notify_one();
      });
      logger.info("submit job {}: accepted={}, queue_size={}", i, accepted, pool.queue_size());
    }

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);
    logger.info("Stats after submit test: {}", pool.stats());
    pool.stop();
  }

  // -------------------------------------------------------------------------
  // 3. try_submit() — non-blocking rejection when queue is full
  // -------------------------------------------------------------------------
  logger.info("--- Test: try_submit() with bounded queue ---");
  {
    // 1 slow worker, queue capacity 2 → easy to fill
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

    // Fill the worker + queue
    for (int i = 0; i < 3; ++i) {
      bool accepted = pool.try_submit([&]() { std::this_thread::sleep_for(200ms); });
      if (i == 0) {
        logger.info("First job accepted: {}", accepted);
      }
    }

    // These should be rejected (queue full)
    for (int i = 0; i < 3; ++i) {
      bool accepted = pool.try_submit([&]() { std::this_thread::sleep_for(50ms); });
      logger.info("try_submit when full: accepted={}", accepted); // false
    }

    logger.info("Stats after try_submit test: {}", pool.stats());
    pool.stop();
  }

  // -------------------------------------------------------------------------
  // 4. submit() blocking when full (block_on_submit_when_full = true)
  // -------------------------------------------------------------------------
  logger.info("--- Test: submit() blocking when queue is full ---");
  {
    std::mutex done_mutex;
    std::condition_variable done_cv;
    std::atomic<int> completed_jobs = 0;
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

    // Submit more jobs than capacity — submit() will block until space is free
    for (int i = 0; i < total_jobs; ++i) {
      bool accepted = pool.submit([&, i]() {
        std::this_thread::sleep_for(50ms);
        int done = ++completed_jobs;
        logger.info("Blocking-submit job {} done ({}/{})", i, done, total_jobs);
        done_cv.notify_one();
      });
      logger.info("Blocking-submit job {}: accepted={}", i, accepted);
    }

    wait_for_jobs(done_cv, done_mutex, completed_jobs, total_jobs);
    logger.info("Stats after blocking-submit test: {}", pool.stats());
    pool.stop();
  }

  // -------------------------------------------------------------------------
  // 5. submit() after stop() — rejections via is_running() guard
  // -------------------------------------------------------------------------
  logger.info("--- Test: submit() on a stopped pool ---");
  {
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
    logger.info("submit after stop: accepted={} (expected false)", accepted);
    logger.info("Stats after stopped-pool test: {}", pool.stats());
  }

  logger.info("ThreadPool example complete");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

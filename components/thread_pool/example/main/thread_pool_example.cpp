#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "logger.hpp"
#include "thread_pool.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "ThreadPool Example", .level = espp::Logger::Verbosity::INFO});

  std::mutex done_mutex;
  std::condition_variable done_cv;
  constexpr int total_jobs = 8;
  std::atomic<int> completed_jobs = 0;

  espp::ThreadPool pool({
      .worker_count = 2,
      .max_queue_size = 16,
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

  logger.info("Submitting {} jobs", total_jobs);

  for (int i = 0; i < total_jobs; ++i) {
    bool queued = pool.submit([&, i]() {
      std::this_thread::sleep_for(100ms);
      int done = ++completed_jobs;
      logger.info("Job {} done ({}/{})", i, done, total_jobs);
      if (done == total_jobs) {
        std::lock_guard<std::mutex> lock(done_mutex);
        done_cv.notify_one();
      }
    });

    if (!queued) {
      logger.error("Failed to queue job {}", i);
    }
  }

  {
    std::unique_lock<std::mutex> lock(done_mutex);
    done_cv.wait(lock, [&]() { return completed_jobs.load() == total_jobs; });
  }

  auto stats = pool.stats();
  logger.info("ThreadPool stats: submitted={} executed={} rejected={}", stats.submitted,
              stats.executed, stats.rejected);

  pool.stop();
  logger.info("ThreadPool example complete");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

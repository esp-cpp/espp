#include <chrono>
#include <vector>

#include "logger.hpp"
#include "runqueue.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "RunQueue Example", .level = espp::Logger::Verbosity::DEBUG});

  {
    logger.info("Basic runqueue example!");
    //! [runqueue example]
    espp::RunQueue runqueue({.log_level = espp::Logger::Verbosity::DEBUG});

    // make some functions to run and schedule them with different priorities to
    // show the priority queue works

    std::mutex done_mutex;
    std::condition_variable done_cv;
    bool done = false;

    auto task = [&logger](int id) {
      auto core = xPortGetCoreID();
      logger.info("Task {} running on core {}", id, core);
      std::this_thread::sleep_for(1s);
      logger.info("Task {} done on core {}", id, core);
    };

    auto stop_task = [&]() {
      logger.info("stop task running!");
      std::this_thread::sleep_for(1s);
      logger.info("Stop task done!");
      std::unique_lock lock(done_mutex);
      done = true;
      done_cv.notify_one();
    };

    logger.info("Scheduling tasks...");
    auto task0 = std::bind(task, 0);
    auto task1 = std::bind(task, 1);
    auto task2 = std::bind(task, 2);
    runqueue.add_function(
        task0,
        espp::RunQueue::MIN_PRIORITY); // this will run first because it was first to be added
    runqueue.add_function(task1, espp::RunQueue::MIN_PRIORITY + 1); // this will run third
    runqueue.add_function(task2, espp::RunQueue::MIN_PRIORITY + 2); // this will run second
    runqueue.add_function(stop_task, espp::RunQueue::MIN_PRIORITY); // this will run last
    logger.info("All tasks scheduled!");

    // check the API for queue_size
    logger.info("Queue size: {}", runqueue.queue_size());

    // check the API for is_running
    logger.info("RunQueue is running: {}", runqueue.is_running());

    logger.info("Waiting for stop task to complete...");
    std::unique_lock lock(done_mutex);
    done_cv.wait(lock, [&done] { return done; });

    logger.info("All tasks done!");
    //! [runqueue example]
  }

  {
    logger.info("Multiple runqueue example (on different cores)!");
    //! [multiple runqueue example]
    std::mutex done_mutex;
    std::condition_variable done_cv;
    bool done = false;

    auto task = [&logger](int id) {
      auto core = xPortGetCoreID();
      logger.info("Task {} running on core {}", id, core);
      std::this_thread::sleep_for(1s);
      logger.info("Task {} done on core {}", id, core);
    };

    auto stop_task = [&]() {
      auto core = xPortGetCoreID();
      logger.info("stop task running on core {}", core);
      std::this_thread::sleep_for(1s);
      logger.info("Stop task done on core {}", core);
      std::unique_lock lock(done_mutex);
      done = true;
      done_cv.notify_one();
    };

    espp::RunQueue runqueue0({.task_config = {.name = "core0 runq", .core_id = 0}});
    espp::RunQueue runqueue1({.task_config = {.name = "core1 runq", .core_id = 1}});

    logger.info("Scheduling tasks...");
    auto task0 = std::bind(task, 0);
    auto task1 = std::bind(task, 1);
    auto task2 = std::bind(task, 2);
    runqueue0.add_function(
        task0,
        espp::RunQueue::MIN_PRIORITY); // this will run first because it was first to be added
    runqueue0.add_function(task1, espp::RunQueue::MIN_PRIORITY + 1); // this will run third
    runqueue0.add_function(task2, espp::RunQueue::MIN_PRIORITY + 2); // this will run second

    runqueue1.add_function(
        task0,
        espp::RunQueue::MIN_PRIORITY); // this will run first because it was first to be added
    runqueue1.add_function(task1, espp::RunQueue::MIN_PRIORITY + 1); // this will run third
    runqueue1.add_function(task2, espp::RunQueue::MIN_PRIORITY + 2); // this will run second
    runqueue1.add_function(stop_task, espp::RunQueue::MIN_PRIORITY); // this will run last
    logger.info("All tasks scheduled!");

    logger.info("Waiting for stop task to complete...");
    std::unique_lock lock(done_mutex);
    done_cv.wait(lock, [&done] { return done; });
    //! [multiple runqueue example]
  }

  logger.info("Example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

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
    auto task3 = std::bind(task, 3);
    auto task0_id = runqueue.add_function(
        task0,
        espp::RunQueue::MIN_PRIORITY); // this will run first because it was first to be added
    auto task1_id =
        runqueue.add_function(task1, espp::RunQueue::MIN_PRIORITY + 1); // this will run fourth
    auto task3_id =
        runqueue.add_function(task3, espp::RunQueue::MAX_PRIORITY); // this will run second
    auto task2_id =
        runqueue.add_function(task2, espp::RunQueue::MIN_PRIORITY + 2); // this will run third
    auto stop_task_id =
        runqueue.add_function(stop_task, espp::RunQueue::MIN_PRIORITY); // this will run last
    logger.info("All tasks scheduled!");

    // print our the task ids
    logger.info("Task 0 id: {}", task0_id);
    logger.info("Task 1 id: {}", task1_id);
    logger.info("Task 2 id: {}", task2_id);
    logger.info("Task 3 id: {}", task3_id);
    logger.info("Stop task id: {}", stop_task_id);

    // check the API for queue_size
    logger.info("Queue size: {}", runqueue.queue_size());

    // check the API for is_running
    logger.info("RunQueue is running: {}", runqueue.is_running());

    // check the API for is_function_queued
    logger.info("Task 0 is queued: {}", runqueue.is_function_queued(task0_id));
    logger.info("Task 1 is queued: {}", runqueue.is_function_queued(task1_id));
    logger.info("Task 2 is queued: {}", runqueue.is_function_queued(task2_id));
    logger.info("Task 3 is queued: {}", runqueue.is_function_queued(task3_id));
    logger.info("Stop task is queued: {}", runqueue.is_function_queued(stop_task_id));

    // check the API for removing an invalid id
    if (runqueue.remove_function(espp::RunQueue::INVALID_ID)) {
      logger.error("Removed invalid id!");
    } else {
      logger.info("Correctly failed to remove invalid id!");
    }

    // check the api for removing a valid but non-existent id
    if (runqueue.remove_function(999)) {
      logger.error("Removed non-existent id!");
    } else {
      logger.info("Correctly failed to remove non-existent id!");
    }

    // NOTE: in the next example (below) we'll check removing a valid ID

    // check the API for get_queued_ids(bool include_running)
    auto queued_ids = runqueue.get_queued_ids(true);
    logger.info("Queued ids (including running): {}", queued_ids);

    // check the API for get_running_id
    auto running_id = runqueue.get_running_id();
    logger.info("Running id: {}", running_id);

    logger.info("Waiting for stop task to complete...");
    std::unique_lock lock(done_mutex);
    done_cv.wait(lock, [&done] { return done; });

    // check the API for get_running_id again (should return nullopt)
    running_id = runqueue.get_running_id();
    logger.info("Running id: {}", running_id);

    // check the api for get_queued_ids again
    queued_ids = runqueue.get_queued_ids(true);
    logger.info("Queued ids (including running): {}", queued_ids);

    // check the api for is_function_queued again
    logger.info("Task 0 is queued: {}", runqueue.is_function_queued(task0_id));
    logger.info("Task 1 is queued: {}", runqueue.is_function_queued(task1_id));
    logger.info("Task 2 is queued: {}", runqueue.is_function_queued(task2_id));
    logger.info("Task 3 is queued: {}", runqueue.is_function_queued(task3_id));
    logger.info("Stop task is queued: {}", runqueue.is_function_queued(stop_task_id));

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
    auto id_to_remove = runqueue0.add_function(stop_task,
                                               espp::RunQueue::MIN_PRIORITY); // this will run last

    runqueue1.add_function(
        task0,
        espp::RunQueue::MIN_PRIORITY); // this will run first because it was first to be added
    runqueue1.add_function(task1, espp::RunQueue::MIN_PRIORITY + 1); // this will run third
    runqueue1.add_function(task2, espp::RunQueue::MIN_PRIORITY + 2); // this will run second
    runqueue1.add_function(stop_task, espp::RunQueue::MIN_PRIORITY); // this will run last
    logger.info("All tasks scheduled!");

    // now remove the stop task from runqueue0
    if (!runqueue0.remove_function(id_to_remove)) {
      logger.error("Failed to remove task from runqueue0!");
    } else {
      logger.info("Removed task from runqueue0!");
    }

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

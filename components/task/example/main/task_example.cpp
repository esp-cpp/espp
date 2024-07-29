#include <chrono>
#include <vector>

#include "run_on_core.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

/**
 *   This example showcases a few different ways of using the espp::Task API:
 *
 *     * [LIFECYCLE] Simple task that does a little bit of math/logging, then
 *       waits. This example turns on DEBUG logging to better showcase the Task
 *       lifecycle as it's started and then destroyed when it goes out of scope.
 *
 *     * [CV.WAIT_FOR] Spawning many tasks that each do a little bit of
 *       math/logging/waiting. This example shows how many tasks can be quickly
 *       and efficiently started and stopped leveraging the cv.wait_for() /
 *       cv.wait_until() API.
 *
 *     * [SLEEP_FOR] Spawning many tasks that each do a little bit of
 *       math/logging/waiting. This example shows the effect of significantly
 *       increased task stop / destruction time (and therefore total test
 *       run-time) using the std::this_thread::sleep_for() / sleep_until()
 *       instead of the condition variable provided to the task function.
 *
 *     * [EARLY EXIT] Complex task that performs many steps of work per task
 *       function iteration. This example showcases using the std::cv_status
 *       return value from cv.wait_for() / cv.wait_until() to determine if it
 *       should stop its long-running processing steps early. This example turns
 *       on DEBUG logging to better showcase the Task lifecycle as it's started
 *       and then destroyed when it goes out of scope.
 *
 */
extern "C" void app_main(void) {
  /**
   *   Set up some variables we'll re-use to control and measure our tests.
   */
  size_t num_seconds_to_run = 2;

  /**
   *   Show a simple task running for a short period and then stopping. Enable
   *   DEBUG logging so that the task prints out when it starts, stops, and is
   *   destroyed. NOTE: the task is automatically stopped in its destructor,
   *   which is called at the end of this scope block when the pointer leaves
   *   scope.
   */
  auto test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning 1 task for {} seconds!\n", num_seconds_to_run);
    //! [Task example]
    auto task_fn = [](std::mutex &m, std::condition_variable &cv) {
      static size_t task_iterations{0};
      fmt::print("Task: #iterations = {}\n", task_iterations);
      task_iterations++;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task(
        {.name = "Task 1", .callback = task_fn, .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
    // show explicitly stopping the task (though the destructor called at the
    // end of this scope would do it for us)
    task.stop();
    //! [Task example]
  }
  auto test_end = std::chrono::high_resolution_clock::now();
  auto test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show the most efficient way to wait in a task, using provided mutex /
   *   condition variable. NOTE: that all tasks stop at exactly the same time
   *   after they each print their num_seconds_to_run iteration
   *   (num_seconds_to_run - 1).
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    //! [ManyTask example]
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    fmt::print("Spawning {} tasks!\n", num_tasks);
    tasks.resize(num_tasks);
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_tasks; i++) {
      size_t iterations{0};
      // copy the loop variables and indicate that we intend to mutate them!
      auto task_fn = [i, iterations](std::mutex &m, std::condition_variable &cv) mutable {
        fmt::print("Task {}: #iterations = {}\n", i, iterations);
        iterations++;
        // NOTE: sleeping in this way allows the sleep to exit early when the
        // task is being stopped / destroyed
        {
          std::unique_lock<std::mutex> lk(m);
          cv.wait_for(lk, 1s);
        }
        // we don't want to stop, so return false
        return false;
      };
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({.name = task_name, .callback = task_fn});
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    fmt::print("Tasks spawned, waiting for {} seconds!\n", num_seconds_to_run);
    std::this_thread::sleep_until(start + num_seconds_to_run * 1s);
    //! [ManyTask example]
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show the more convenient way to wait in a task, simply using sleep_for or
   *   sleep_until. NOTE: that NOT all tasks stop at the same time and some
   *   tasks run for multiple additional iterations (printing up to 7 or so).
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    fmt::print("Spawning {} tasks!\n", num_tasks);
    tasks.resize(num_tasks);
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < num_tasks; i++) {
      size_t iterations{0};
      // copy the loop variables and indicate that we intend to mutate them!
      auto task_fn = [i, iterations]() mutable {
        fmt::print("Task {}: #iterations = {}\n", i, iterations);
        iterations++;
        // NOTE: sleeping in this way PREVENTS the sleep / task from early
        // exiting when the task is being stopped / destroyed.
        std::this_thread::sleep_for(1s);
        // we don't want to stop, so return false
        return false;
      };
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({
          .callback = task_fn,
          .task_config =
              {
                  .name = task_name,
              },
      });
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    fmt::print("Tasks spawned, waiting for {} seconds!\n", num_seconds_to_run);
    std::this_thread::sleep_until(start + num_seconds_to_run * 1s);
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show an example of a long running task that has multiple steps per
   *   iteration and can early exit if the task is being shut down.
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning complex task for {} seconds!\n", num_seconds_to_run);
    //! [LongRunningTask example]
    auto task_fn = [](std::mutex &m, std::condition_variable &cv) {
      static size_t task_iterations{0};
      const size_t num_steps_per_iteration = 10;
      fmt::print("Task processing iteration {}...\n", task_iterations);
      for (size_t i = 0; i < num_steps_per_iteration; i++) {
        // NOTE: sleeping in this way allows the sleep to exit early when the
        // task is being stopped / destroyed
        {
          std::unique_lock<std::mutex> lk(m);
          // NOTE: using the return value from the cv.wait_for() allows us to
          // know if the task was asked to stop, for which we can handle and
          // return early.
          auto cv_retval = cv.wait_for(lk, 100ms);
          if (cv_retval == std::cv_status::no_timeout) {
            // if there was no timeout, then we were notified, therefore we need
            // to shut down.
            fmt::print("Task stopping early (step {}/{}) on iteration {}\n", i,
                       num_steps_per_iteration, task_iterations);
            // NOTE: use this_thread::sleep_for() to fake cleaning up work that
            // we would do
            std::this_thread::sleep_for(10ms);
            // now that we've (fake) cleaned-up our work, return from the task
            // function so the task can fully destruct.
            return false;
          }
        }
      }
      fmt::print("Task processing iteration {} complete\n", task_iterations);
      task_iterations++;
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task(
        {.name = "Complex Task", .callback = task_fn, .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();
    //! [LongRunningTask example]
    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show an example of printing out the task info from another thread.
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning 1 task for {} seconds!\n", num_seconds_to_run);
    //! [Task Info example]
    auto task_fn = [](std::mutex &m, std::condition_variable &cv) {
      static size_t task_iterations{0};
      task_iterations++;
      // allocate stack
      char buffer[1024];
      // do something with the bufer (which also uses stack)
      snprintf(buffer, 1024, "%.06f\n", (float)task_iterations);
      fmt::print("{}\n", espp::Task::get_info());
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task = espp::Task(
        {.name = "DynamicTask", .callback = task_fn, .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();
    auto now = std::chrono::high_resolution_clock::now();
    auto elapsed = std::chrono::duration<float>(now - test_start).count();
    while (elapsed < num_seconds_to_run) {
      fmt::print("{}\n", espp::Task::get_info(task));
      std::this_thread::sleep_for(200ms);
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - test_start).count();
    }
    //! [Task Info example]
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show an example of the task auto-stopping itself from within the task
   *   callback function.
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning 1 task for {} seconds!\n", num_seconds_to_run);
    //! [Task Request Stop example]
    auto task_fn = [&num_seconds_to_run](std::mutex &m, std::condition_variable &cv) {
      static auto begin = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - begin).count();
      if (elapsed > num_seconds_to_run) {
        // we've gone long enough, time to stop our task!
        return true;
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // we don't want to stop yet, so return false
      return false;
    };
    auto task = espp::Task({.name = "AutoStop Task",
                            .callback = task_fn,
                            .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();
    while (task.is_started()) {
      std::this_thread::sleep_for(10ms);
    }
    //! [Task Request Stop example]
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show an example of the task auto-stopping itself from within the task
   *   callback function and then starting it again.
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning 1 task for {} seconds!\n", num_seconds_to_run);
    //! [Task Request Stop Then Restart example]
    auto task_fn = [&num_seconds_to_run](std::mutex &m, std::condition_variable &cv) {
      static auto begin = std::chrono::high_resolution_clock::now();
      auto now = std::chrono::high_resolution_clock::now();
      auto elapsed = std::chrono::duration<float>(now - begin).count();
      if (elapsed > num_seconds_to_run) {
        static int num_times_run{0};
        fmt::print("Task stopping early after {} runs!\n", ++num_times_run);
        // we've gone long enough, time to stop our task!
        return true;
      }
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 10ms);
      }
      // we don't want to stop yet, so return false
      return false;
    };
    auto task = espp::Task({.name = "AutoStop Task",
                            .callback = task_fn,
                            .log_level = espp::Logger::Verbosity::DEBUG});
    task.start();
    while (task.is_started()) {
      std::this_thread::sleep_for(10ms);
    }
    // restart the task without explicitly cancelling it
    fmt::print("Restarting task...\n");
    task.start();
    while (task.is_started()) {
      std::this_thread::sleep_for(10ms);
    }
    //! [Task Request Stop Then Restart example]
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  {
    //! [run on core example]
    fmt::print("Example main running on core {}\n", xPortGetCoreID());
    // NOTE: in these examples, because we're logging with libfmt in the
    // function to be run, we need a little more than the default 2k stack size,
    // so we're using 3k.

    // test running a function that returns void on a specific core
    auto task_fn = []() -> void { fmt::print("Void Task running on core {}\n", xPortGetCoreID()); };
    espp::task::run_on_core(task_fn, 0, 3 * 1024);
    espp::task::run_on_core(task_fn, 1, 3 * 1024);
    fmt::print("Void Function returned\n");

    // test running a function that returns bool on a specific core
    auto task_fn2 = []() -> bool {
      auto core_id = xPortGetCoreID();
      fmt::print("Bool Task running on core {}\n", core_id);
      return core_id == 1;
    };
    auto result0 = espp::task::run_on_core(task_fn2, 0, 3 * 1024);
    fmt::print("Bool Function returned {}\n", result0);
    auto result1 = espp::task::run_on_core(task_fn2, 1, 3 * 1024);
    fmt::print("Bool Function returned {}\n", result1);

    // test running a function that returns esp_err_t on a specific core
    auto task_fn3 = []() -> esp_err_t {
      auto core_id = xPortGetCoreID();
      fmt::print("esp_err_t Task running on core {}\n", core_id);
      return core_id == 1 ? ESP_OK : ESP_FAIL;
    };
    auto err0 = espp::task::run_on_core(task_fn3, 0, 3 * 1024);
    fmt::print("esp_err_t Function returned {}\n", esp_err_to_name(err0));
    auto err1 = espp::task::run_on_core(task_fn3, 1, 3 * 1024);
    fmt::print("esp_err_t Function returned {}\n", esp_err_to_name(err1));
    //! [run on core example]
  }

  fmt::print("Task example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

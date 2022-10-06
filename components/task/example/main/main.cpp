#include <chrono>
#include <vector>

#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  size_t num_seconds_to_run = 2;
  auto test_start = std::chrono::high_resolution_clock::now();
  auto test_end = std::chrono::high_resolution_clock::now();
  auto test_duration = std::chrono::duration<float>(test_end - test_start).count();
  /**
   *   Show a simple task running for a short period and then stopping. Enable
   *   DEBUG logging so that the task prints out when it starts, stops, and is
   *   destroyed. NOTE: the task is automatically stopped in its destructor,
   *   which is called at the end of this scope block when the pointer leaves
   *   scope.
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    fmt::print("Spawning 1 task for {} seconds!\n", num_seconds_to_run);
    auto task_fn = [](std::mutex& m, std::condition_variable& cv) {
      static size_t task_iterations{0};
      fmt::print("Task: #iterations = {}\n", task_iterations);
      task_iterations++;
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
    };
    auto task = espp::Task({
        .name = "Task 1",
        .callback = task_fn,
        .log_level = espp::Logger::Level::DEBUG
      });
    task.start();

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  /**
   *   Show the most efficient way to wait in a task, using provided mutex /
   *   condition variable. NOTE: that all tasks stop at exactly the same time
   *   after they each print their num_seconds_to_run iteration
   *   (num_seconds_to_run - 1).
   */
  test_start = std::chrono::high_resolution_clock::now();
  {
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    fmt::print("Spawning {} tasks!\n", num_tasks);
    tasks.resize(num_tasks);
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i=0; i<num_tasks; i++) {
      size_t iterations{0};
      // copy the loop variables and indicate that we intend to mutate them!
      auto task_fn = [i, iterations](std::mutex& m, std::condition_variable& cv) mutable {
        fmt::print("Task {}: #iterations = {}\n", i, iterations);
        iterations++;
        // NOTE: sleeping in this way allows the sleep to exit early when the
        // task is being stopped / destroyed
        {
          std::unique_lock<std::mutex> lk(m);
          cv.wait_for(lk, 1s);
        }
      };
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({
          .name = task_name,
          .callback = task_fn
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
    for (size_t i=0; i<num_tasks; i++) {
      size_t iterations{0};
      // copy the loop variables and indicate that we intend to mutate them!
      auto task_fn = [i, iterations](std::mutex& m, std::condition_variable& cv) mutable {
        fmt::print("Task {}: #iterations = {}\n", i, iterations);
        iterations++;
        // NOTE: sleeping in this way PREVENTS the sleep / task from early
        // exiting when the task is being stopped / destroyed.
        std::this_thread::sleep_for(1s);
      };
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({
          .name = task_name,
          .callback = task_fn
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
    auto task_fn = [](std::mutex& m, std::condition_variable& cv) {
      static size_t task_iterations{0};
      const size_t num_steps_per_iteration = 10;
      fmt::print("Task processing iteration {}...\n", task_iterations);
      for (size_t i=0; i<num_steps_per_iteration; i++) {
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
            fmt::print("Task stopping early (step {}/{}) on iteration {}\n", i, num_steps_per_iteration, task_iterations);
            // now that we've (fake) cleaned-up our work, return from the task
            // function so the task can fully destruct.
            return;
          }
        }
      }
      fmt::print("Task processing iteration {} complete\n", task_iterations);
      task_iterations++;
    };
    auto task = espp::Task({
        .name = "Complex Task",
        .callback = task_fn,
        .log_level = espp::Logger::Level::DEBUG
      });
    task.start();

    std::this_thread::sleep_for(num_seconds_to_run * 1s);
  }
  test_end = std::chrono::high_resolution_clock::now();
  test_duration = std::chrono::duration<float>(test_end - test_start).count();
  fmt::print("Test ran for {:.03f} seconds\n", test_duration);

  fmt::print("Task example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

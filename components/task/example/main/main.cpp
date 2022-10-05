#include <chrono>
#include <vector>

#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // we get access to fmt since we include task.hpp which includes logger.hpp
  {
    fmt::print("Spawning 1 task for 10 seconds!\n");
    auto task_fn = []() {
      static size_t task_iterations{0};
      fmt::print("Task: {}\n", task_iterations);
      task_iterations++;
      // sleep
      std::this_thread::sleep_for(500ms);
    };
    auto task = espp::Task({
        .name = "Task 1",
        .callback = task_fn,
        .log_level = espp::Logger::Level::DEBUG
      });
    task.start();

    std::this_thread::sleep_for(2s);
  }
  {
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    fmt::print("Spawning {} tasks!\n", num_tasks);
    tasks.resize(num_tasks);
    auto start = std::chrono::high_resolution_clock::now();
    for (size_t i=0; i<num_tasks; i++) {
      size_t iterations{0};
      // copy the loop variables and indicate that we intend to mutate them!
      auto task_fn = [i, iterations]() mutable {
        fmt::print("Task {}: {}\n", i, iterations);
        iterations++;
        // sleep
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
    fmt::print("Tasks spawned, waiting for 10 seconds!\n");
    std::this_thread::sleep_until(start + 10s);
  }

  fmt::print("Task example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

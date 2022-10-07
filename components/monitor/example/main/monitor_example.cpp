#include <chrono>
#include <functional>
#include <thread>

#include "task_monitor.hpp"
#include "task.hpp"

// for doing work
#include <math.h>

using namespace std::chrono_literals;
using namespace std::placeholders;

extern "C" void app_main(void) {
  fmt::print("Stating monitor example!\n");
  {
    //! [TaskMonitor example]
    // create the monitor
    espp::TaskMonitor tm({.period = 500ms});
    // create threads
    auto start = std::chrono::high_resolution_clock::now();
    auto task_fn = [&start](int task_id, auto&, auto&) {
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds_since_start = std::chrono::duration<float>(now - start).count();
      // do some work
      float x = 2.0f * M_PI * sin(exp(task_id) * seconds_since_start);
      fmt::print("[Task {}]: {:.03}\n", task_id, x);
      // sleep
      std::this_thread::sleep_for(100ms);
    };
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    tasks.resize(num_tasks);
    for (size_t i=0; i<num_tasks; i++) {
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({
          .name = task_name,
          .callback = std::bind(task_fn, i, _1, _2)
        });
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(10s);
    //! [TaskMonitor example]
  }

  fmt::print("Monitor example finished!\n");

  // sleep forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

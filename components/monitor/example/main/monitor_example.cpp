#include <chrono>
#include <functional>
#include <thread>

#include "task.hpp"
#include "task_monitor.hpp"

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
    auto task_fn = [&start](int task_id, auto &, auto &, auto &) {
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds_since_start = std::chrono::duration<float>(now - start).count();
      // do some work
      float x = 2.0f * M_PI * sin(exp(task_id) * seconds_since_start);
      // sleep
      std::this_thread::sleep_for((10ms * std::abs(x)) + 1ms);
      // don't want to stop the task
      return false;
    };
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    tasks.resize(num_tasks);
    for (size_t i = 0; i < num_tasks; i++) {
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({.name = task_name,
                                           .callback = std::bind(task_fn, i, _1, _2, _3),
                                           .stack_size_bytes = 5 * 1024});
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(5s);
    //! [TaskMonitor example]
  }

  {
    //! [get_latest_info_vector example]
    // create threads
    auto start = std::chrono::high_resolution_clock::now();
    auto task_fn = [&start](int task_id, auto &, auto &, auto &) {
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds_since_start = std::chrono::duration<float>(now - start).count();
      // do some work
      float x = 2.0f * M_PI * sin(exp(task_id) * seconds_since_start);
      // sleep
      std::this_thread::sleep_for((10ms * std::abs(x)) + 1ms);
      // don't want to stop the task
      return false;
    };
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    tasks.resize(num_tasks);
    for (size_t i = 0; i < num_tasks; i++) {
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({.name = task_name,
                                           .callback = std::bind(task_fn, i, _1, _2, _3),
                                           .stack_size_bytes = 5 * 1024});
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(1s);
    auto task_info = espp::TaskMonitor::get_latest_info_vector();
    for (const auto &info : task_info) {
      fmt::print("{}\n", info);
    }
    //! [get_latest_info_vector example]
  }

  {
    //! [get_latest_info example]
    // create threads
    auto start = std::chrono::high_resolution_clock::now();
    auto task_fn = [&start](int task_id, auto &, auto &, auto &) {
      auto now = std::chrono::high_resolution_clock::now();
      auto seconds_since_start = std::chrono::duration<float>(now - start).count();
      // do some work
      float x = 2.0f * M_PI * sin(exp(task_id) * seconds_since_start);
      // sleep
      std::this_thread::sleep_for((10ms * std::abs(x)) + 1ms);
      // don't want to stop the task
      return false;
    };
    std::vector<std::unique_ptr<espp::Task>> tasks;
    size_t num_tasks = 10;
    tasks.resize(num_tasks);
    for (size_t i = 0; i < num_tasks; i++) {
      std::string task_name = fmt::format("Task {}", i);
      auto task = espp::Task::make_unique({.name = task_name,
                                           .callback = std::bind(task_fn, i, _1, _2, _3),
                                           .stack_size_bytes = 5 * 1024});
      tasks[i] = std::move(task);
      tasks[i]->start();
    }
    // now sleep for a while to let the monitor do its thing
    std::this_thread::sleep_for(1s);
    // single line string
    fmt::print("Task Monitor Info (single line):\n");
    fmt::print("{}\n", espp::TaskMonitor::get_latest_info_string());
    // pretty table
    fmt::print("Task Monitor Info (pretty table):\n");
    auto task_table = espp::TaskMonitor::get_latest_info_table();
    std::cout << task_table << std::endl;
    //! [get_latest_info example]
  }

  fmt::print("Monitor example finished!\n");

  // sleep forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

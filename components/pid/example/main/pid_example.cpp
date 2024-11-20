#include <chrono>
#include <vector>

#include "pid.hpp"
#include "task.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  static auto start = std::chrono::high_resolution_clock::now();
  size_t num_seconds_to_run = 5;

  {
    fmt::print("Basic PID example\n");
    //! [pid example]
    espp::Pid pid({.kp = 1.0f,
                   .ki = 0.1f,
                   .kd = 0.0f,
                   .integrator_min = -1000.0f,
                   .integrator_max = 1000.0f,
                   .output_min = -100.0f,
                   .output_max = 100.0f});
    for (int i = 0; i < num_seconds_to_run; i++) {
      float error = (float)num_seconds_to_run / (float)(i + 1);
      float output = pid.update(error);
      fmt::print("PID: ({}) -> {:0.3f}\n", pid, output);
      // std::this_thread::sleep_for(5ms);
    }
    //! [pid example]
  }

  {
    fmt::print("Running complex (mutating) PID example {} seconds\n", num_seconds_to_run);
    //! [complex pid example]
    espp::Pid::Config pid_config{.kp = 1.0f,
                                 .ki = 0.1f,
                                 .kd = 0.0f,
                                 .integrator_min = -1000.0f,
                                 .integrator_max = 1000.0f,
                                 .output_min = -100.0f,
                                 .output_max = 100.0f,
                                 .log_level = espp::Logger::Verbosity::INFO};
    espp::Pid pid(pid_config);
    auto task_fn = [&pid](std::mutex &m, std::condition_variable &cv) {
      auto now = std::chrono::high_resolution_clock::now();
      float elapsed = std::chrono::duration<float>(now - start).count();
      float error = 2.0f / elapsed;
      float output = pid.update(error);
      fmt::print("PID: ({}) -> {:0.3f}\n", pid, output);
      // NOTE: sleeping in this way allows the sleep to exit early when the
      // task is being stopped / destroyed
      {
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 100ms);
      }
      // don't want to stop the task
      return false;
    };
    auto task = espp::Task({.callback = task_fn,
                            .task_config = {.name = "PID Update"},
                            .log_level = espp::Logger::Verbosity::INFO});
    task.start();
    for (int i = 0; i < num_seconds_to_run; i++) {
      // change PID gains here
      fmt::print("Increasing p-gain\n");
      pid_config.kp = (float)i / (float)num_seconds_to_run;
      pid.change_gains(pid_config);
      std::this_thread::sleep_for(1s);
    }
    //! [complex pid example]
  }

  fmt::print("PID example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

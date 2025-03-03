#include "task.hpp"

using namespace std::chrono_literals;

int main() {

  static auto start = std::chrono::steady_clock::now();
  static auto elapsed = []() -> float {
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - start).count();
  };

  espp::Logger logger({.tag = "Task Test", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting task test");

  espp::Task task({.name = "Task", .callback = [&](auto &, auto &) -> bool {
                     auto now = std::chrono::steady_clock::now();
                     logger.info("[{:.3f}] Hello from the task!", elapsed());
                     std::this_thread::sleep_until(now + 1s);
                     // don't want to stop the task
                     return false;
                   }});
  task.start();

  std::this_thread::sleep_for(10s);

  logger.info("Stopping task test");

  return 0;
}

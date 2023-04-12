#include <chrono>
#include <thread>
#include <vector>

#include "event_manager.hpp"
#include "format.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  {
    fmt::print("Starting event manager example!\n");
    //! [event manager example]
    espp::EventManager::get().set_log_level(espp::Logger::Verbosity::WARN);

    // let's make some event names that we'll use for this example
    const std::string event1 = "battery/state";
    const std::string event2 = "drive/control";

    // Make a task which has a pub/sub in it. NOTE: in real code, this would
    // likely be within a custom class and the registration would happen in the
    // constructor, with the remove_publisher / remove_subscriber calls in its
    // destructor. Also NOTE: you would likely use a serialization library like
    // alpaca (wrapped in espp serialization component) for serializing and
    // deserializing the data structures being passed to string data.
    auto task_1_fn = [&event1, &event2](auto &m, auto &cv) {
      {
        // Just for fun, we'll only define the subscriber callback within the
        // context of this task function
        static auto cb = [](const std::string &data) {
          fmt::print("Task 1 cb got data: '{}'\n", data);
        };
        // we only want to register once, so ust the std::call_once /
        // std::once_flag functionality to only register the first time the
        // task is run
        static std::once_flag flag;
        std::call_once(flag, [&event1, &event2]() {
          auto &em = espp::EventManager::get();
          fmt::print("Task 1 registering!\n");
          auto did_pub = em.add_publisher(event1, "task 1");
          auto did_sub = em.add_subscriber(event2, "task 1", cb);
          fmt::print("Task 1 publishing:  {}\n", did_pub);
          fmt::print("Task 1 subscribing: {}\n", did_sub);
        });
        // periodically publish on event1
        fmt::print("Task 1 publishing on event1\n");
        espp::EventManager::get().publish(event1, "Event 1 data");
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task1 = espp::Task({.name = "Task 1", .callback = task_1_fn});
    task1.start();

    // Now let's make another task which will have pub/sbu as well
    auto task_2_fn = [&event1, &event2](auto &m, auto &cv) {
      {
        // Just for fun, we'll only define the subscriber callback within the
        // context of this task function
        static auto cb = [](const std::string &data) {
          fmt::print("Task 2 cb got data: '{}'\n", data);
        };
        // we only want to register once, so ust the std::call_once /
        // std::once_flag functionality to only register the first time the
        // task is run
        static std::once_flag flag;
        std::call_once(flag, [&event1, &event2]() {
          auto &em = espp::EventManager::get();
          fmt::print("Task 2 registering!\n");
          auto did_pub = em.add_publisher(event2, "task 2");
          auto did_sub = em.add_subscriber(event1, "task 2", cb);
          fmt::print("Task 2 publishing:  {}\n", did_pub);
          fmt::print("Task 2 subscribing: {}\n", did_sub);
        });
        // periodically publish on event2
        fmt::print("Task 2 publishing on event2\n");
        espp::EventManager::get().publish(event2, "Event 2 data");
        std::unique_lock<std::mutex> lk(m);
        cv.wait_for(lk, 500ms);
      }
      // we don't want to stop, so return false
      return false;
    };
    auto task2 = espp::Task({.name = "Task 2", .callback = task_2_fn});
    task2.start();

    // Now let's just wait for a little while for those tasks to run, showcasing
    // the pub/sub interactions in the log output.
    fmt::print("Sleeping for 5s...\n");
    std::this_thread::sleep_for(5s);

    task1.stop();
    task2.stop();

    // since the tasks are done, let's remove their publishers/subscibers here
    // (though as noted above, this would normally be done in a class
    // destructor)
    auto &em = espp::EventManager::get();
    em.remove_publisher(event1, "task 1");
    em.remove_subscriber(event2, "task 1");
    em.remove_publisher(event2, "task 2");
    em.remove_subscriber(event1, "task 2");
    //! [event manager example]
  }

  fmt::print("Event manager example complete!\n");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

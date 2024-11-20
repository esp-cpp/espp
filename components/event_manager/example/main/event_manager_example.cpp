#include <chrono>
#include <thread>
#include <vector>

#include "event_manager.hpp"
#include "logger.hpp"
#include "serialization.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "main", .level = espp::Logger::Verbosity::DEBUG});
  logger.info("Starting event manager example!");
  //! [event manager example]
  espp::EventManager::get().set_log_level(espp::Logger::Verbosity::WARN);

  // NOTE: we'll use a simple string for publishing on drive/control, but
  // normally you'd use a struct and a serialization library like alpaca, so
  // that's what we'll do for battery/state

  // let's define a struct to contain data for an event
  struct BatteryState {
    float voltage{48.0f};
    float current{0.0f};
    bool is_charging{false};
    float temperature_celsius{25.0f};
    float state_of_charge{100.0f};
  };

  // let's make some event names that we'll use for this example
  const std::string event1 = "battery/state";
  const std::string event2 = "drive/control";

  static int num_published = 0;
  static int num_received = 0;

  // Make a task which has a pub/sub in it. NOTE: in real code, this would
  // likely be within a custom class and the registration would happen in the
  // constructor, with the remove_publisher / remove_subscriber calls in its
  // destructor. Also NOTE: you would likely use a serialization library like
  // alpaca (wrapped in espp serialization component) for serializing and
  // deserializing the data structures being passed to string data.
  auto task_1_fn = [&](auto &m, auto &cv) {
    {
      // Just for fun, we'll only define the subscriber callback within the
      // context of this task function
      static auto event2_cb = [&](const std::vector<uint8_t> &data) {
        // we know this is a string, so just convert it to a string and
        // print it
        std::string data_str(data.begin(), data.end());
        logger.debug("Task 1 cb got data: '{}'", data_str);
        num_received++;
        // block here like we're doing work
        std::this_thread::sleep_for(10ms);
      };
      // we only want to register once, so ust the std::call_once /
      // std::once_flag functionality to only register the first time the
      // task is run
      static std::once_flag flag;
      std::call_once(flag, [&]() {
        auto &em = espp::EventManager::get();
        logger.info("Task 1 registering!");
        auto did_pub = em.add_publisher(event1, "task 1");
        auto did_sub = em.add_subscriber(event2, "task 1", event2_cb);
        logger.info("Task 1 publishing:  {}", did_pub);
        logger.info("Task 1 subscribing: {}", did_sub);
        // sleep for a little bit to let the other task register its
        // subscribers/publishers before we start publishing ensuring that the
        // subscriber callback is registered before the publisher publishes
        std::this_thread::sleep_for(10ms);
      });
      // periodically publish on event1
      logger.debug("Task 1 publishing on {}", event1);
      static BatteryState bs;
      bs.current = 1.0f;
      bs.voltage -= 0.1f;
      bs.state_of_charge -= 5.0f;
      bs.temperature_celsius += 0.2f;
      std::vector<uint8_t> buffer;
      espp::serialize(bs, buffer);
      espp::EventManager::get().publish(event1, buffer);
      num_published++;
      buffer.clear();
      bs.current = -1.0f;
      espp::serialize(bs, buffer);
      espp::EventManager::get().publish(event1, buffer);
      num_published++;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 500ms);
    }
    // we don't want to stop, so return false
    return false;
  };
  auto task1 = espp::Task({.callback = task_1_fn, .task_config = {.name = "Task 1"}});

  // Now let's make another task which will have pub/sub as well
  auto task_2_fn = [&](auto &m, auto &cv) {
    {
      // Just for fun, we'll only define the subscriber callback within the
      // context of this task function
      static auto event1_cb = [&](const std::vector<uint8_t> &data) {
        // we know the data is a BatteryState struct, so deserialize it and
        // print it
        std::error_code ec;
        auto bs = espp::deserialize<BatteryState>(data, ec);
        if (ec) {
          logger.error("Couldn't deserialize BatteryState: {}", ec.message());
          return;
        }
        logger.debug("Task 2 got battery state data:\n"
                     "  voltage:             {:.2f}\n"
                     "  current:             {:.2f}\n"
                     "  is_charging:         {}\n"
                     "  temperature_celsius: {:.2f}\n"
                     "  state_of_charge:     {:.2f}",
                     bs.voltage, bs.current, bs.is_charging, bs.temperature_celsius,
                     bs.state_of_charge);
        num_received++;
        // block here like we're doing work
        std::this_thread::sleep_for(10ms);
      };
      // we only want to register once, so ust the std::call_once /
      // std::once_flag functionality to only register the first time the
      // task is run
      static std::once_flag flag;
      std::call_once(flag, [&]() {
        auto &em = espp::EventManager::get();
        logger.info("Task 2 registering!");
        auto did_pub = em.add_publisher(event2, "task 2");
        // NOTE: we're using a custom task config here to show how you can
        // configure the task that the subscriber callback will run in
        // (priority, stack size, etc.). Only the first subscription on a topic
        // will use the task config, any subsequent subscriptions will use the
        // same task as the first subscription.
        espp::Task::BaseConfig task_config{
            .name = "Task 2 subscriber task",
            .stack_size_bytes = 8192,
            .priority = 10, // 5 is default, 10 is higher
        };
        auto did_sub = em.add_subscriber(event1, "task 2", event1_cb, task_config);
        logger.info("Task 2 publishing:  {}", did_pub);
        logger.info("Task 2 subscribing: {}", did_sub);
        // sleep for a little bit to let the other task register its
        // subscribers/publishers, ensuring that the subscriber callback is
        // registered before the publisher publishes
        std::this_thread::sleep_for(10ms);
      });
      // periodically publish on event2
      logger.debug("Task 2 publishing on {}", event2);
      static int iteration = 0;
      std::string data = fmt::format("Task 2 data {}", iteration++);
      std::vector<uint8_t> buffer(data.begin(), data.end());
      espp::EventManager::get().publish(event2, buffer);
      num_published++;
      espp::EventManager::get().publish(event2, buffer);
      num_published++;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_for(lk, 500ms);
    }
    // we don't want to stop, so return false
    return false;
  };
  auto task2 = espp::Task({.callback = task_2_fn, .task_config = {.name = "Task 2"}});

  // now start the tasks
  task1.start();
  task2.start();

  // Now let's just wait for a little while for those tasks to run, showcasing
  // the pub/sub interactions in the log output.
  logger.info("Sleeping for 5s...");
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

  logger.info("Published {} times", num_published);
  logger.info("Received {} times", num_received);

  if (num_published != num_received) {
    logger.error("Mismatch between published and received!");
  } else {
    logger.info("Published and received match!");
  }

  logger.info("Event manager example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}

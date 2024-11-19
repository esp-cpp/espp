#pragma once

#include <deque>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"
#include "event_map.hpp"
#include "task.hpp"

namespace espp {
/**
 * @brief Singleton class for managing events. Provides mechanisms for
 *        anonymous publish / subscribe interactions - enabling one to one,
 *        one to many, many to one, and many to many data distribution with
 *        loose coupling and low overhead. Each topic runs a thread for that
 *        topic's subscribers, executing all the callbacks in sequence and
 *        then going to sleep again until new data is published.
 *
 * @note In c++ objects, it's recommended to call the
 *       add_publisher/add_subscriber functions in the class constructor and
 *       then to call the remove_publisher/remove_subscriber functions in the
 *       class destructor.
 *
 * @note It is recommended (unless you are only interested in events and not
 *       data or are only needing to transmit actual strings) to use a
 *       serialization library (such as espp::serialization - which wraps
 *       alpaca) to serialize your data structures to string when publishing
 *       and then deserialize your data from string in the subscriber
 *       callbacks.
 *
 * \section event_manager_ex1 Event Manager Example
 * \snippet event_manager_example.cpp event manager example
 */
class EventManager : public espp::BaseComponent {
public:
  /**
   * @brief Function definition for function prototypes to be called when
   *        subscription/event data is available.
   * @param std::vector<uint8_t>& The data associated with the event
   */
  typedef std::function<void(const std::vector<uint8_t> &)> event_callback_fn;

  /**
   * @brief Get the singleton instance of the EventManager.
   * @return A reference to the EventManager singleton.
   */
  static EventManager &get() {
    static EventManager INSTANCE;
    return INSTANCE;
  }

  EventManager(EventManager const &) = delete;
  EventManager &operator=(EventManager const &) = delete;
  EventManager(EventManager &&) = delete;
  EventManager &operator=(EventManager &&) = delete;

  /**
   * @brief Register a publisher for \p component on \p topic.
   * @param topic Topic name for the data being published.
   * @param component Name of the component publishing data.
   * @return True if the publisher was added, false if it was already
   *         registered for that component.
   */
  bool add_publisher(const std::string &topic, const std::string &component);

  /**
   * @brief Register a subscriber for \p component on \p topic.
   * @param topic Topic name for the data being subscribed to.
   * @param component Name of the component publishing data.
   * @param callback The event_callback_fn to be called when receicing data on
   *        \p topic.
   * @param stack_size_bytes The stack size in bytes to use for the subscriber
   * @note The stack size is only used if a subscriber is not already registered
   *       for that topic. If a subscriber is already registered for that topic,
   *       the stack size is ignored.
   * @return True if the subscriber was added, false if it was already
   *         registered for that component.
   */
  bool add_subscriber(const std::string &topic, const std::string &component,
                      const espp::EventManager::event_callback_fn &callback,
                      const size_t stack_size_bytes = 8192);

  /**
   * @brief Register a subscriber for \p component on \p topic.
   * @param topic Topic name for the data being subscribed to.
   * @param component Name of the component publishing data.
   * @param callback The event_callback_fn to be called when receicing data on
   *        \p topic.
   * @param task_config The task configuration to use for the subscriber.
   * @note The task_config is only used if a subscriber is not already
   *       registered for that topic. If a subscriber is already registered for
   *       that topic, the task_config is ignored.
   * @return True if the subscriber was added, false if it was already
   *         registered for that component.
   */
  bool add_subscriber(const std::string &topic, const std::string &component,
                      const espp::EventManager::event_callback_fn &callback,
                      const espp::Task::BaseConfig &task_config);

  /**
   * @brief Publish \p data on \p topic.
   * @param topic Topic to publish data on.
   * @param data Data to publish, within a vector container.
   * @return True if \p data was successfully published to \p topic, false
   *         otherwise. Publish will not occur (and will return false) if
   *         there are no subscribers for this topic.
   */
  bool publish(const std::string &topic, const std::vector<uint8_t> &data);

  /**
   * @brief Remove \p component's publisher for \p topic.
   * @param topic The topic that \p component was publishing on.
   * @param component The component for which the publisher was registered.
   * @return True if the publisher was removed, false if it was not
   *         registered.
   */
  bool remove_publisher(const std::string &topic, const std::string &component);

  /**
   * @brief Remove \p component's subscriber for \p topic.
   * @param topic The topic that \p component was subscribing to.
   * @param component The component for which the subscriber was registered.
   * @return True if the subscriber was removed, false if it was not
   *         registered.
   */
  bool remove_subscriber(const std::string &topic, const std::string &component);

protected:
  EventManager()
      : espp::BaseComponent("Event Manager") {}

  struct SubscriberData {
    std::mutex m;
    bool notified = false; // Allows cv to ignore spurious wakeups
    std::condition_variable cv;
    std::deque<std::vector<uint8_t>> deq;
  };

  bool subscriber_task_fn(const std::string &topic, std::mutex &m, std::condition_variable &cv);

  std::recursive_mutex events_mutex_;
  detail::EventMap events_;

  std::recursive_mutex callbacks_mutex_;
  std::unordered_map<std::string, std::vector<std::pair<std::string, event_callback_fn>>>
      subscriber_callbacks_;

  std::recursive_mutex tasks_mutex_;
  std::unordered_map<std::string, std::unique_ptr<Task>> subscriber_tasks_;

  std::recursive_mutex data_mutex_;
  std::unordered_map<std::string, SubscriberData> subscriber_data_;
};
} // namespace espp

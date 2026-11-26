#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
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
 *       class destructor. When remove_subscriber() returns, the callback is
 *       not running and will not be called again, so the object can be
 *       destroyed safely right after it.
 *
 * @note Every function is thread safe and can be called from any thread,
 *       including from inside a subscriber callback (for example a callback
 *       that removes its own subscription). remove_subscriber() and clear()
 *       wait for a callback of that topic that is running on another thread
 *       to return, so don't call them while holding a lock which that
 *       topic's callbacks take. Called from one of the topic's own callbacks,
 *       they don't wait; the removed callbacks are not called again once the
 *       current one returns.
 *
 * @note Data published on a topic is queued and delivered in order on the
 *       topic's thread. Data still queued when the topic's last subscriber is
 *       removed is dropped. When C++ exceptions are enabled, an exception
 *       thrown by a callback is logged and the other callbacks still run.
 *
 * @note It is recommended (unless you are only interested in events and not
 *       data or are only needing to transmit actual strings) to use a
 *       serialization library (such as espp::serialization - which wraps
 *       alpaca) to serialize your data structures to bytes when publishing
 *       and then deserialize your data from bytes in the subscriber
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
   * @brief Destroy the EventManager, stopping every topic's thread (see
   *        clear()).
   */
  ~EventManager();

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
   * @param component Name of the component subscribing to the data.
   * @param callback The event_callback_fn to be called when receiving data on
   *        \p topic.
   * @param stack_size_bytes The stack size in bytes to use for the subscriber
   * @note The stack size is only used if a subscriber is not already registered
   *       for that topic. If a subscriber is already registered for that topic,
   *       the stack size is ignored.
   * @return True if the subscriber was added, false if it was already
   *         registered for that component, the callback is empty, or the
   *         topic's thread could not be started.
   */
  bool add_subscriber(const std::string &topic, const std::string &component,
                      const espp::EventManager::event_callback_fn &callback,
                      const size_t stack_size_bytes = 8192);

  /**
   * @brief Register a subscriber for \p component on \p topic.
   * @param topic Topic name for the data being subscribed to.
   * @param component Name of the component subscribing to the data.
   * @param callback The event_callback_fn to be called when receiving data on
   *        \p topic.
   * @param task_config The task configuration to use for the subscriber.
   * @note The task_config is only used if a subscriber is not already
   *       registered for that topic. If a subscriber is already registered for
   *       that topic, the task_config is ignored.
   * @return True if the subscriber was added, false if it was already
   *         registered for that component, the callback is empty, or the
   *         topic's thread could not be started.
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
   * @details When it returns, the callback is not running and will not be
   *          called again, unless it was called from one of \p topic's own
   *          callbacks (then the callback is not called again once the
   *          current one returns). Removing the topic's last subscriber stops
   *          the topic's thread and drops any data still queued for it.
   * @param topic The topic that \p component was subscribing to.
   * @param component The component for which the subscriber was registered.
   * @return True if the subscriber was removed, false if it was not
   *         registered.
   */
  bool remove_subscriber(const std::string &topic, const std::string &component);

  /**
   * @brief Remove every publisher and subscriber and stop every topic's
   *        thread.
   * @details Waits for callbacks running on other threads to return. The
   *          EventManager can be used again afterwards.
   */
  void clear();

protected:
  EventManager()
      : espp::BaseComponent("Event Manager") {}

  /// One component's subscription. Shared by the topic's subscriber list and
  /// by the snapshot a delivery in progress iterates, so a callback is never
  /// destroyed while it runs.
  struct Subscriber {
    Subscriber(const std::string &component, const event_callback_fn &callback)
        : component(component)
        , callback(callback) {}
    const std::string component;
    const event_callback_fn callback;
    std::atomic<bool> removed{false}; ///< Set when removed; deliveries skip it from then on.
  };
  using SubscriberList = std::vector<std::shared_ptr<Subscriber>>;

  /// The state of one topic that has subscribers. Owned through a shared_ptr
  /// so publish() and the removal of the last subscriber can overlap safely.
  struct Topic {
    explicit Topic(const std::string &name)
        : name(name) {}
    const std::string name;
    std::mutex m;                    ///< Guards everything below except task.
    std::condition_variable cv;      ///< Wakes the topic's thread (data queued or stopping).
    std::condition_variable idle_cv; ///< Signalled when a delivery finishes.
    std::deque<std::vector<uint8_t>> queue;
    bool stopping{false};   ///< Set once, when the topic loses its last subscriber.
    bool delivering{false}; ///< True while the topic's thread is running callbacks.
    uint64_t deliveries{0}; ///< Number of finished deliveries.
    /// Replaced, never changed in place, with both EventManager::mutex_ and m
    /// held, so holding either one is enough to read it.
    std::shared_ptr<const SubscriberList> subscribers;
    /// Declared last so it is destroyed (joining the thread) first.
    std::unique_ptr<Task> task;
  };

  static bool deliver_next(Topic &topic);
  static bool is_delivering(const Topic &topic);
  static void wait_for_delivery(Topic &topic);
  void stop_topics(std::vector<std::shared_ptr<Topic>> topics);
  void reap_stopped_topics();

  std::mutex mutex_; ///< Guards the members below.
  std::unordered_map<std::string, std::vector<std::string>> publishers_;
  std::unordered_map<std::string, std::shared_ptr<Topic>> topics_;
  /// Topics that lost their last subscriber from inside one of their own
  /// callbacks: their thread exits on its own and is joined later (by
  /// reap_stopped_topics(), clear(), or the destructor).
  std::vector<std::shared_ptr<Topic>> stopping_topics_;
};
} // namespace espp

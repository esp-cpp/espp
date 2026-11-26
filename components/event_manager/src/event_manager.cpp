#include "event_manager.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

using namespace espp;

namespace {
// The topic whose callbacks this thread is running right now, if any. A
// removal made from one of those callbacks must not wait for the delivery in
// progress or join the topic's thread: both are this thread.
thread_local const void *t_delivering_topic = nullptr;

void call_subscriber(const std::string &topic, const std::string &component,
                     const EventManager::event_callback_fn &callback,
                     const std::vector<uint8_t> &data) {
#if defined(__cpp_exceptions) && __cpp_exceptions
  // A callback that throws must not end the topic's thread (an exception
  // escaping it calls std::terminate): log it and go on with the next one.
  static espp::Logger logger({.tag = "Event Manager", .level = espp::Logger::Verbosity::WARN});
  try {
    callback(data);
  } catch (const std::exception &e) {
    logger.error("Exception in subscriber '{}' on topic '{}': {}", component, topic, e.what());
  } catch (...) {
    logger.error("Unknown exception in subscriber '{}' on topic '{}'", component, topic);
  }
#else
  // C++ exceptions are disabled (the ESP-IDF default): nothing can be thrown.
  callback(data);
#endif
}
} // namespace

EventManager::~EventManager() {
  clear();
  // Left over only if the EventManager is destroyed from inside a topic's
  // callback (e.g. exit() called from one). That thread can't be joined from
  // itself, and destroying its Task here would std::terminate, so keep the
  // topic alive: its thread exits on its own once the callback returns.
  std::vector<std::shared_ptr<Topic>> left;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    left.swap(stopping_topics_);
  }
  if (!left.empty()) {
    static auto &kept = *new std::vector<std::shared_ptr<Topic>>(); // never destroyed
    kept.insert(kept.end(), std::make_move_iterator(left.begin()),
                std::make_move_iterator(left.end()));
  }
}

bool EventManager::add_publisher(const std::string &topic, const std::string &component) {
  logger_.info("Adding publisher '{}' to topic '{}'", component, topic);
  std::lock_guard<std::mutex> lk(mutex_);
  // NOTE: this will default construct this if it does not exist
  auto &topic_publishers = publishers_[topic];
  if (detail::get_index_in_container(component, topic_publishers).first) {
    // component is already registered as a publisher, so return false
    return false;
  }
  topic_publishers.push_back(component);
  return true;
}

bool EventManager::add_subscriber(const std::string &topic, const std::string &component,
                                  const event_callback_fn &callback,
                                  const size_t stack_size_bytes) {
  return add_subscriber(topic, component, callback,
                        {.name = "", .stack_size_bytes = stack_size_bytes});
}

bool EventManager::add_subscriber(const std::string &topic, const std::string &component,
                                  const event_callback_fn &callback,
                                  const Task::BaseConfig &task_config) {
  logger_.info("Adding subscriber '{}' to topic '{}'", component, topic);
  if (!callback) {
    logger_.error("Cannot add subscriber '{}' to topic '{}': its callback is empty", component,
                  topic);
    return false;
  }
  reap_stopped_topics();
  // Declared before the lock so that whatever they hold when we return (a
  // callback, a replaced subscriber list, a topic whose thread didn't start) is
  // destroyed after the lock is released, never under it.
  auto subscriber = std::make_shared<Subscriber>(component, callback);
  std::shared_ptr<const SubscriberList> replaced;
  std::shared_ptr<Topic> state;
  std::lock_guard<std::mutex> lk(mutex_);
  if (auto it = topics_.find(topic); it != topics_.end()) {
    state = it->second;
    const auto &current = *state->subscribers;
    auto is_component = [&component](const std::shared_ptr<Subscriber> &s) {
      return s->component == component;
    };
    if (std::find_if(current.begin(), current.end(), is_component) != current.end()) {
      // callback for this component is already registered, so return false
      return false;
    }
    auto subscribers = current;
    subscribers.push_back(std::move(subscriber));
    std::lock_guard<std::mutex> topic_lk(state->m);
    replaced = std::exchange(state->subscribers,
                             std::make_shared<const SubscriberList>(std::move(subscribers)));
    return true;
  }
  // first subscriber on this topic: create it and start its thread
  state = std::make_shared<Topic>(topic);
  state->subscribers = std::make_shared<const SubscriberList>(SubscriberList{subscriber});
  auto config = task_config;
  if (config.name.empty()) {
    config.name = topic + " subscriber";
  }
  logger_.debug("Creating task for topic '{}'", topic);
  logger_.debug("  with config: {}", config);
  state->task = Task::make_unique(
      {.callback = Task::callback_no_params_fn{[t = state.get()]() { return deliver_next(*t); }},
       .task_config = config});
  if (!state->task->start()) {
    logger_.error("Cannot add subscriber '{}' to topic '{}': could not start the topic's task",
                  component, topic);
    return false;
  }
  topics_.emplace(topic, std::move(state));
  return true;
}

bool EventManager::publish(const std::string &topic, const std::vector<uint8_t> &data) {
  logger_.info("Publishing on topic '{}'", topic);
  std::shared_ptr<Topic> state;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      return false;
    }
    state = it->second;
  }
  {
    std::vector<uint8_t> copy(data); // copied before taking the lock
    std::lock_guard<std::mutex> lk(state->m);
    if (state->stopping) {
      // its last subscriber was removed while we were looking it up
      return false;
    }
    state->queue.push_back(std::move(copy));
  }
  state->cv.notify_one();
  return true;
}

bool EventManager::remove_publisher(const std::string &topic, const std::string &component) {
  logger_.info("Removing publisher '{}' on topic '{}'", component, topic);
  std::lock_guard<std::mutex> lk(mutex_);
  auto it = publishers_.find(topic);
  if (it == publishers_.end()) {
    // there is no publisher for this topic
    return false;
  }
  auto &topic_publishers = it->second;
  auto elem = std::find(topic_publishers.begin(), topic_publishers.end(), component);
  if (elem == topic_publishers.end()) {
    // component is not registered as a publisher, so return false
    return false;
  }
  topic_publishers.erase(elem);
  if (topic_publishers.empty()) {
    publishers_.erase(it);
  }
  return true;
}

bool EventManager::remove_subscriber(const std::string &topic, const std::string &component) {
  logger_.info("Removing subscriber '{}' on topic '{}'", component, topic);
  // Destroyed after the lock is released (see add_subscriber).
  std::shared_ptr<Subscriber> removed;
  std::shared_ptr<const SubscriberList> replaced;
  std::shared_ptr<Topic> state;
  bool was_last_subscriber = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto it = topics_.find(topic);
    if (it == topics_.end()) {
      logger_.warn("Cannot remove subscriber, there are no subscribers for topic '{}'", topic);
      return false;
    }
    state = it->second;
    auto subscribers = *state->subscribers;
    auto elem = std::find_if(
        subscribers.begin(), subscribers.end(),
        [&component](const std::shared_ptr<Subscriber> &s) { return s->component == component; });
    if (elem == subscribers.end()) {
      logger_.warn(
          "Cannot remove subscriber, '{}' is not registered as a subscriber for topic '{}'",
          component, topic);
      return false;
    }
    removed = *elem;
    // a delivery in progress skips it from now on
    removed->removed = true;
    subscribers.erase(elem);
    was_last_subscriber = subscribers.empty();
    if (was_last_subscriber) {
      topics_.erase(it);
    }
    std::lock_guard<std::mutex> topic_lk(state->m);
    replaced = std::exchange(state->subscribers,
                             std::make_shared<const SubscriberList>(std::move(subscribers)));
    if (was_last_subscriber) {
      state->stopping = true;
    }
  }
  if (was_last_subscriber) {
    logger_.info("It was the last subscriber for '{}', stopping its task", topic);
    std::vector<std::shared_ptr<Topic>> stopped;
    stopped.push_back(std::move(state));
    stop_topics(std::move(stopped));
  } else if (!is_delivering(*state)) {
    // the callback may be running right now on the topic's thread
    wait_for_delivery(*state);
  }
  reap_stopped_topics();
  return true;
}

void EventManager::clear() {
  logger_.info("Removing every publisher and subscriber");
  std::vector<std::shared_ptr<Topic>> topics;
  std::vector<std::shared_ptr<const SubscriberList>> replaced;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    publishers_.clear();
    for (auto &[name, state] : topics_) {
      topics.push_back(std::move(state));
    }
    topics_.clear();
    topics.insert(topics.end(), std::make_move_iterator(stopping_topics_.begin()),
                  std::make_move_iterator(stopping_topics_.end()));
    stopping_topics_.clear();
    for (const auto &state : topics) {
      std::lock_guard<std::mutex> topic_lk(state->m);
      for (const auto &subscriber : *state->subscribers) {
        subscriber->removed = true;
      }
      replaced.push_back(
          std::exchange(state->subscribers, std::make_shared<const SubscriberList>()));
      state->stopping = true;
    }
  }
  stop_topics(std::move(topics));
  // destroy the callbacks now that every lock is released and every thread
  // is stopped (a callback still running belongs to the thread calling us)
  replaced.clear();
}

void EventManager::stop_topics(std::vector<std::shared_ptr<Topic>> topics) {
  // every topic here is already marked stopping
  for (const auto &state : topics) {
    state->cv.notify_all();
  }
  for (auto &state : topics) {
    if (is_delivering(*state)) {
      // Removed from one of its own callbacks: this is the topic's thread, so
      // it can't be joined here. It exits once the callback returns.
      std::lock_guard<std::mutex> lk(mutex_);
      stopping_topics_.push_back(std::move(state));
      continue;
    }
    // waits for a callback in progress to return, then joins the thread
    state->task->stop();
  }
}

void EventManager::reap_stopped_topics() {
  std::vector<std::shared_ptr<Topic>> finished;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    auto still_running = [](const std::shared_ptr<Topic> &state) {
      return state->task->is_started();
    };
    auto first_finished =
        std::stable_partition(stopping_topics_.begin(), stopping_topics_.end(), still_running);
    finished.insert(finished.end(), std::make_move_iterator(first_finished),
                    std::make_move_iterator(stopping_topics_.end()));
    stopping_topics_.erase(first_finished, stopping_topics_.end());
  }
  // destroying them (without the lock) joins threads which have already returned
  finished.clear();
}

bool EventManager::is_delivering(const Topic &topic) { return t_delivering_topic == &topic; }

void EventManager::wait_for_delivery(Topic &topic) {
  std::unique_lock<std::mutex> lk(topic.m);
  if (!topic.delivering) {
    return;
  }
  const auto in_progress = topic.deliveries + 1;
  topic.idle_cv.wait(lk, [&topic, in_progress] { return topic.deliveries >= in_progress; });
}

bool EventManager::deliver_next(Topic &topic) {
  std::vector<uint8_t> data;
  std::shared_ptr<const SubscriberList> subscribers;
  {
    std::unique_lock<std::mutex> lk(topic.m);
    topic.cv.wait(lk, [&topic] { return topic.stopping || !topic.queue.empty(); });
    if (topic.stopping) {
      // stop the task; anything still queued is dropped with the topic
      return true;
    }
    data = std::move(topic.queue.front());
    topic.queue.pop_front();
    subscribers = topic.subscribers;
    topic.delivering = true;
  }
  t_delivering_topic = &topic;
  for (const auto &subscriber : *subscribers) {
    if (!subscriber->removed) {
      call_subscriber(topic.name, subscriber->component, subscriber->callback, data);
    }
  }
  t_delivering_topic = nullptr;
  // Let go of the snapshot before reporting the delivery finished: a removed
  // subscriber's callback is then destroyed here, not after its remover has
  // returned.
  subscribers.reset();
  {
    std::lock_guard<std::mutex> lk(topic.m);
    topic.delivering = false;
    ++topic.deliveries;
  }
  topic.idle_cv.notify_all();
  return false;
}

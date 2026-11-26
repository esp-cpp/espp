#include "event_manager.hpp"

#include <algorithm>
#include <iterator>
#include <utility>

using namespace espp;

namespace {
// The topic whose event this thread is delivering right now, if any: set while
// its callbacks run and while the delivery lets go of its subscriber snapshot
// (which can destroy removed callbacks). A call made from that thread never
// waits for that topic's thread or joins it: both are this thread.
thread_local void *t_delivering_topic = nullptr;

// Guards every Topic's waiting_on / waiting_for. Never destroyed, so that a
// wait during static destruction can still use it.
std::mutex &wait_graph_mutex() {
  static auto *m = new std::mutex();
  return *m;
}

void call_subscriber(const std::string &topic, const std::string &component,
                     const EventManager::event_callback_fn &callback,
                     const std::vector<uint8_t> &data) {
#if defined(__cpp_exceptions) && __cpp_exceptions
  // A callback that throws must not end the topic's thread (an exception
  // escaping it calls std::terminate): log it and go on with the next one.
  // Never destroyed, so that it still works for callbacks that run while the
  // program exits.
  static auto &logger =
      *new espp::Logger({.tag = "Event Manager", .level = espp::Logger::Verbosity::WARN});
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

EventManager::Topic::~Topic() {
  // Wake the thread so that destroying its task can join it: Task::stop()
  // only signals the Task's own condition variable, not cv.
  {
    std::lock_guard<std::mutex> lk(m);
    stopping = true;
  }
  cv.notify_all();
  task.reset();
}

EventManager::~EventManager() {
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ++clearing_; // refuse new subscribers for good, also once clear() returns
  }
  clear();
  // Left over only if the EventManager is destroyed on one of its topics'
  // threads (e.g. exit() called from a callback). That thread can't be joined
  // from itself, and destroying its Task here would std::terminate, so keep
  // the topic alive: its thread exits on its own once the callback returns.
  std::vector<std::shared_ptr<Topic>> finished;
  std::vector<std::shared_ptr<Topic>> left;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    take_finished_topics(finished);
    left.swap(retiring_);
  }
  finished.clear();
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
  // Declared before the lock so that whatever they hold when we return (a
  // callback, a replaced subscriber list, topics whose thread is to be joined,
  // a topic whose thread didn't start) is destroyed after the lock is
  // released, never under it.
  std::vector<std::shared_ptr<Topic>> finished;
  auto subscriber = std::make_shared<Subscriber>(component, callback);
  std::shared_ptr<const SubscriberList> replaced;
  std::shared_ptr<Topic> state;
  std::lock_guard<std::mutex> lk(mutex_);
  if (clearing_ > 0) {
    logger_.warn("Cannot add subscriber '{}' to topic '{}' while clear() runs", component, topic);
    return false;
  }
  take_finished_topics(finished);
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
  // The topic just lost its last subscriber but its thread hasn't exited yet
  // (e.g. a callback that removed itself subscribes again): keep using that
  // thread rather than starting a second one for the same topic.
  for (auto it = retiring_.rbegin(); it != retiring_.rend(); ++it) {
    if ((*it)->name == topic && revive(**it, subscriber, replaced)) {
      state = *it;
      retiring_.erase(std::next(it).base());
      topics_.emplace(topic, std::move(state));
      return true;
    }
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
    std::vector<std::shared_ptr<Topic>> finished; // joined after the lock is released
    std::lock_guard<std::mutex> lk(mutex_);
    take_finished_topics(finished);
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
  // Destroyed after the locks are released (see add_subscriber).
  std::vector<std::shared_ptr<Topic>> finished;
  event_callback_fn callback;
  std::shared_ptr<Subscriber> removed;
  std::shared_ptr<const SubscriberList> replaced;
  std::shared_ptr<Topic> state;
  // Other topics of this name, each of which can still be running the
  // component's callback although it is no longer registered there: the
  // callback removed itself, or clear() removed it.
  std::vector<std::shared_ptr<Topic>> others;
  bool topic_found = false;
  bool was_last_subscriber = false;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    take_finished_topics(finished);
    std::copy_if(retiring_.begin(), retiring_.end(), std::back_inserter(others),
                 [&topic](const std::shared_ptr<Topic> &t) { return t->name == topic; });
    if (auto it = topics_.find(topic); it != topics_.end()) {
      topic_found = true;
      auto subscribers = *it->second->subscribers;
      auto elem = std::find_if(
          subscribers.begin(), subscribers.end(),
          [&component](const std::shared_ptr<Subscriber> &s) { return s->component == component; });
      if (elem == subscribers.end()) {
        others.push_back(it->second);
      } else {
        state = it->second;
        removed = *elem;
        subscribers.erase(elem);
        was_last_subscriber = subscribers.empty();
        if (was_last_subscriber) {
          topics_.erase(it);
          retiring_.push_back(state);
        }
        std::lock_guard<std::mutex> topic_lk(state->m);
        // a delivery in progress skips it from now on
        removed->removed = true;
        replaced = std::exchange(state->subscribers,
                                 std::make_shared<const SubscriberList>(std::move(subscribers)));
        if (was_last_subscriber) {
          retire(*state);
        }
      }
    }
  }
  for (const auto &other : others) {
    wait_while_running(*other, &component);
  }
  if (!state) {
    if (!topic_found) {
      logger_.warn("Cannot remove subscriber, there are no subscribers for topic '{}'", topic);
    } else {
      logger_.warn(
          "Cannot remove subscriber, '{}' is not registered as a subscriber for topic '{}'",
          component, topic);
    }
    return false;
  }
  if (was_last_subscriber) {
    logger_.info("It was the last subscriber for '{}', stopping its task", topic);
    state->cv.notify_all();
  }
  // the callback may be running right now on the topic's thread
  wait_while_running(*state, &component);
  {
    std::lock_guard<std::mutex> topic_lk(state->m);
    if (state->calling != removed.get()) {
      // Not running, and never called again: destroy it here, once the locks
      // are released, rather than later on the topic's thread.
      std::swap(callback, removed->callback);
    }
  }
  if (was_last_subscriber && wait_while_running(*state, nullptr)) {
    // join its thread now rather than when the next call notices it finished
    std::lock_guard<std::mutex> lk(mutex_);
    take_finished_topics(finished);
  }
  return true;
}

void EventManager::clear() {
  logger_.info("Removing every publisher and subscriber");
  // Destroyed after the locks are released (see add_subscriber).
  std::vector<std::shared_ptr<Topic>> topics;
  std::vector<std::shared_ptr<const SubscriberList>> replaced;
  {
    std::lock_guard<std::mutex> lk(mutex_);
    ++clearing_; // refuse new subscribers until every topic is stopped
    publishers_.clear();
    for (auto &[name, state] : topics_) {
      retiring_.push_back(std::move(state));
    }
    topics_.clear();
    for (const auto &state : retiring_) {
      std::lock_guard<std::mutex> topic_lk(state->m);
      for (const auto &subscriber : *state->subscribers) {
        subscriber->removed = true;
      }
      replaced.push_back(
          std::exchange(state->subscribers, std::make_shared<const SubscriberList>()));
      retire(*state);
    }
    topics = retiring_;
  }
  for (const auto &state : topics) {
    state->cv.notify_all();
  }
  for (const auto &state : topics) {
    wait_while_running(*state, nullptr);
  }
  {
    std::vector<std::shared_ptr<Topic>> finished; // joined after the lock is released
    std::lock_guard<std::mutex> lk(mutex_);
    take_finished_topics(finished);
    --clearing_;
  }
  topics.clear();
  // destroy the callbacks now that every lock is released and every thread
  // is stopped (one still running belongs to a thread that we can't wait for)
  replaced.clear();
}

void EventManager::take_finished_topics(std::vector<std::shared_ptr<Topic>> &finished) {
  // mutex_ is held. A retiring topic that is no longer delivering won't run a
  // callback again: its thread is exiting, so joining it doesn't block.
  if (retiring_.empty()) {
    return;
  }
  auto still_delivering = [](const std::shared_ptr<Topic> &state) {
    std::lock_guard<std::mutex> lk(state->m);
    return state->busy;
  };
  auto first_finished = std::stable_partition(retiring_.begin(), retiring_.end(), still_delivering);
  finished.insert(finished.end(), std::make_move_iterator(first_finished),
                  std::make_move_iterator(retiring_.end()));
  retiring_.erase(first_finished, retiring_.end());
}

void EventManager::retire(Topic &topic) {
  // topic.m is held
  topic.stopping = true;
  topic.queue.clear();
}

bool EventManager::revive(Topic &topic, const std::shared_ptr<Subscriber> &subscriber,
                          std::shared_ptr<const SubscriberList> &replaced) {
  {
    std::lock_guard<std::mutex> lk(topic.m);
    if (topic.exited) {
      return false;
    }
    // the thread hasn't seen stopping yet, and won't now
    topic.stopping = false;
    replaced = std::exchange(topic.subscribers,
                             std::make_shared<const SubscriberList>(SubscriberList{subscriber}));
  }
  // a remover waiting to join the topic's thread stops waiting
  topic.idle_cv.notify_all();
  return true;
}

EventManager::Topic *EventManager::delivering_topic() {
  return static_cast<Topic *>(t_delivering_topic);
}

bool EventManager::is_delivering(const Topic &topic) { return t_delivering_topic == &topic; }

bool EventManager::is_running(const Topic &topic, const std::string *component) {
  // topic.m is held
  if (component) {
    return topic.calling && topic.calling->component == *component;
  }
  return topic.busy && topic.stopping;
}

bool EventManager::waits_for(const Topic &from, const Topic &self) {
  // The wait-graph mutex is held. Follow what each topic's thread is waiting
  // for, starting at from's: true if that leads back to self's thread. An
  // edge only counts while what it waits for is still running; otherwise that
  // thread is about to stop waiting.
  const Topic *t = &from;
  // a chain is never longer than the number of topic threads; the bound only
  // guards against a bug turning this into an endless loop
  for (int hops = 0; hops < 1024; ++hops) {
    Topic *next = t->waiting_on;
    if (!next) {
      return false;
    }
    bool live = false;
    {
      std::lock_guard<std::mutex> lk(next->m);
      live = is_running(*next, t->waiting_for);
    }
    if (!live) {
      return false;
    }
    if (next == &self) {
      return true;
    }
    t = next;
  }
  return true;
}

bool EventManager::wait_while_running(Topic &topic, const std::string *component) {
  // Waits until no callback of *component runs on topic (component null:
  // until a stopping topic's thread has finished its last delivery). Returns
  // false, without waiting, if that would deadlock: on the topic's own
  // thread, or when the topic's thread is itself waiting, directly or through
  // other topics, for this thread.
  if (is_delivering(topic)) {
    return false;
  }
  {
    std::lock_guard<std::mutex> lk(topic.m);
    if (!is_running(topic, component)) {
      return true;
    }
  }
  Topic *self = delivering_topic();
  if (self) {
    std::lock_guard<std::mutex> lk(wait_graph_mutex());
    if (waits_for(topic, *self)) {
      return false;
    }
    self->waiting_on = &topic;
    self->waiting_for = component;
  }
  {
    std::unique_lock<std::mutex> lk(topic.m);
    topic.idle_cv.wait(lk, [&topic, component] { return !is_running(topic, component); });
  }
  if (self) {
    std::lock_guard<std::mutex> lk(wait_graph_mutex());
    self->waiting_on = nullptr;
    self->waiting_for = nullptr;
  }
  return true;
}

bool EventManager::deliver_next(Topic &topic) {
  std::vector<uint8_t> data;
  std::shared_ptr<const SubscriberList> subscribers;
  {
    std::unique_lock<std::mutex> lk(topic.m);
    topic.cv.wait(lk, [&topic] { return topic.stopping || !topic.queue.empty(); });
    if (topic.stopping) {
      // stop the task
      topic.exited = true;
      return true;
    }
    data = std::move(topic.queue.front());
    topic.queue.pop_front();
    subscribers = topic.subscribers;
    topic.busy = true;
  }
  t_delivering_topic = &topic;
  for (const auto &subscriber : *subscribers) {
    {
      std::lock_guard<std::mutex> lk(topic.m);
      if (subscriber->removed) {
        continue;
      }
      topic.calling = subscriber.get();
    }
    call_subscriber(topic.name, subscriber->component, subscriber->callback, data);
    {
      std::lock_guard<std::mutex> lk(topic.m);
      topic.calling = nullptr;
    }
    topic.idle_cv.notify_all();
  }
  // Letting go of the snapshot destroys the callbacks that were removed while
  // they ran (their remover couldn't take them). Still part of the delivery,
  // so a destructor that calls back into the EventManager is treated like one
  // of this topic's callbacks.
  subscribers.reset();
  t_delivering_topic = nullptr;
  {
    std::lock_guard<std::mutex> lk(topic.m);
    topic.busy = false;
  }
  topic.idle_cv.notify_all();
  return false;
}

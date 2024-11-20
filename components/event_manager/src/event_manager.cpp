#include "event_manager.hpp"

using namespace espp;

bool EventManager::add_publisher(const std::string &topic, const std::string &component) {
  logger_.info("Adding publisher '{}' to topic '{}'", component, topic);
  std::lock_guard<std::recursive_mutex> lk(events_mutex_);
  // add to `events_`
  // NOTE: this will default construct this if it does not exist
  auto &topic_publishers = events_.publishers[topic];
  auto [exists, index] = detail::get_index_in_container(component, topic_publishers);
  if (exists) {
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
  {
    std::lock_guard<std::recursive_mutex> lk(events_mutex_);
    // add to `events_`
    // NOTE: this will default construct this if it does not exist
    auto &topic_subscribers = events_.subscribers[topic];
    auto [exists, index] = detail::get_index_in_container(component, topic_subscribers);
    if (exists) {
      // component is already registered as a subscriber, so return false
      return false;
    }
    topic_subscribers.push_back(component);
  }
  // add to `subscriber_callbacks_`
  {
    std::lock_guard<std::recursive_mutex> lk(callbacks_mutex_);
    auto &callbacks = subscriber_callbacks_[topic];
    auto is_component = [&component](std::pair<std::string, event_callback_fn> &e) {
      return std::get<0>(e) == component;
    };
    auto elem = std::find_if(std::begin(callbacks), std::end(callbacks), is_component);
    if (elem != std::end(callbacks)) {
      // callback for this component is already registered, so return false
      return false;
    }
    callbacks.emplace_back(component, callback);
  }
  // if not in `subscriber_tasks_`
  {
    std::lock_guard<std::recursive_mutex> lk(tasks_mutex_);
    if (!subscriber_tasks_.contains(topic)) {
      // add to `subscriber_data_`
      subscriber_data_[topic]; // insert default constructed data
      // create new task (using bound subscriber_task_fn) and add to
      // `subscriber_tasks_`
      using namespace std::placeholders;
      auto config = task_config;
      if (config.name.empty()) {
        config.name = topic + " subscriber";
      }
      logger_.debug("Creating task for topic '{}'", topic);
      logger_.debug("  with config: {}", config);
      subscriber_tasks_[topic] = Task::make_unique(
          {.callback = std::bind(&EventManager::subscriber_task_fn, this, topic, _1, _2, _3),
           .task_config = config});
      // and start it
      subscriber_tasks_[topic]->start();
    }
  }
  return true;
}

bool EventManager::publish(const std::string &topic, const std::vector<uint8_t> &data) {
  logger_.info("Publishing on topic '{}'", topic);
  // find topic in `subscriber_data_`, push_back into the queue there and notify
  // the cv.
  // get the data queue
  SubscriberData *sub_data;
  {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    // find sub_data in `subscriber_data_`
    if (!subscriber_data_.contains(topic)) {
      return false;
    }
    sub_data = &subscriber_data_[topic];
  }
  {
    // lock the data queue
    std::unique_lock<std::mutex> lk(sub_data->m);
    // push the data into the queue
    sub_data->deq.push_back(data);
    // update the notified flag (used to ignore spurious wakeups)
    sub_data->notified = true;
  }
  // notify the task that there is new data in the queue
  sub_data->cv.notify_all();
  return true;
}

bool EventManager::remove_publisher(const std::string &topic, const std::string &component) {
  logger_.info("Removing publisher '{}' on topic '{}'", component, topic);
  // remove from `events_`
  std::lock_guard<std::recursive_mutex> lk(events_mutex_);
  if (!events_.publishers.contains(topic)) {
    // there is no publisher for this topic
    return false;
  }
  // We know the container contains a value for [topic]
  auto &topic_publishers = events_.publishers[topic];
  auto elem = std::find(std::begin(topic_publishers), std::end(topic_publishers), component);
  bool exists = elem != std::end(topic_publishers);
  if (!exists) {
    // component is not registered as a publisher, so return false
    return false;
  }
  // we found it, so remove it from the list
  topic_publishers.erase(elem);
  return true;
}

bool EventManager::remove_subscriber(const std::string &topic, const std::string &component) {
  logger_.info("Removing subscriber '{}' on topic '{}'", component, topic);
  bool was_last_subscriber{false};
  // remove from `events_`
  {
    std::lock_guard<std::recursive_mutex> lk(events_mutex_);
    if (!events_.subscribers.contains(topic)) {
      logger_.warn("Cannot remove subscriber, there are no subscribers for topic '{}'", topic);
      // there is no publisher for this topic
      return false;
    }
    // We know the container contains a value for [topic]
    auto &topic_subscribers = events_.subscribers[topic];
    auto elem = std::find(std::begin(topic_subscribers), std::end(topic_subscribers), component);
    bool exists = elem != std::end(topic_subscribers);
    if (!exists) {
      logger_.warn(
          "Cannot remove subscriber, '{}' is not registered as a subscriber for topic '{}'",
          component, topic);
      // component is not registered as a subscriber, so return false
      return false;
    }
    // we found it, so remove it from the list
    topic_subscribers.erase(elem);
    was_last_subscriber = topic_subscribers.size() == 0;
  }
  // remove from `subscriber_callbacks_`
  {
    std::lock_guard<std::recursive_mutex> lk(callbacks_mutex_);
    auto &callbacks = subscriber_callbacks_[topic];

    auto is_component = [&component](std::pair<std::string, event_callback_fn> &e) {
      return std::get<0>(e) == component;
    };
    auto elem = std::find_if(std::begin(callbacks), std::end(callbacks), is_component);
    if (elem != std::end(callbacks)) {
      callbacks.erase(elem);
    }
    if (was_last_subscriber) {
      // remove the key from the map
      subscriber_callbacks_.erase(topic);
    }
  }
  // if this was the last subscriber
  if (was_last_subscriber) {
    logger_.info("It was the last subscriber for '{}', cleaning up tasks", topic);
    // notify the data (so the subscriber task function can stop waiting on the data cv)
    {
      std::lock_guard<std::recursive_mutex> lk(data_mutex_);
      {
        std::unique_lock<std::mutex> data_lk(subscriber_data_[topic].m);
        subscriber_data_[topic].notified = true;
      }
      subscriber_data_[topic].cv.notify_all();
    }
    {
      std::lock_guard<std::recursive_mutex> lk(tasks_mutex_);
      // stop the task
      subscriber_tasks_[topic]->stop();
      // remove from `subscriber_tasks_`
      subscriber_tasks_.erase(topic);
    }
    {
      std::lock_guard<std::recursive_mutex> lk(data_mutex_);
      // remove from `subscriber_data_`
      subscriber_data_.erase(topic);
    }
  }
  return true;
}

bool EventManager::subscriber_task_fn(const std::string &topic, std::mutex &m,
                                      std::condition_variable &cv, bool &task_notified) {
  // get the data queue
  SubscriberData *sub_data;
  {
    std::lock_guard<std::recursive_mutex> lk(data_mutex_);
    // find sub_data in `subscriber_data_`
    if (!subscriber_data_.contains(topic)) {
      // stop the task, we don't have valid subscriber data
      return true;
    }
    sub_data = &subscriber_data_[topic];
  }
  // get the data
  logger_.debug("Waiting on data for topic '{}'", topic);
  {
    // wait on sub_data's mutex/cv
    std::unique_lock<std::mutex> lk(sub_data->m);
    sub_data->cv.wait(lk, [&sub_data] { return sub_data->notified; });
    if (sub_data->deq.empty()) {
      // stop the task, we were notified, but there was no data available.
      return true;
    }
  }
  // we were woken up - that means there must be >= 1 element in the queue,
  // so let's loop until we get all the data
  while (true) {
    logger_.debug("Getting data for topic '{}'", topic);
    std::vector<uint8_t> data;
    {
      std::unique_lock<std::mutex> lk(sub_data->m);
      if (sub_data->deq.empty()) {
        // reset the notified flag
        sub_data->notified = false;
        // we've gotten all the data, so break out of the loop
        break;
      }
      // set data to sub_data's deque front
      data = sub_data->deq.front();
      // and pop the front data off
      sub_data->deq.pop_front();
    }
    // get all the callbacks
    logger_.debug("Finding callbacks for topic '{}'", topic);
    std::vector<std::pair<std::string, event_callback_fn>> *callbacks;
    {
      std::lock_guard<std::recursive_mutex> lk(callbacks_mutex_);
      if (!subscriber_callbacks_.contains(topic)) {
        // stop the task, we don't have any callbacks anymore.
        return true;
      }
      // copy here so that we don't hold this lock the whole time we're calling
      // callbacks
      callbacks = &subscriber_callbacks_[topic];
    }
    // call all the callbacks
    logger_.debug("Calling {} callbacks for topic '{}'", callbacks->size(), topic);
    for (auto [comp, callback] : *callbacks) {
      logger_.debug("Callback for '{}'", comp);
      callback(data);
    }
  }
  // we don't want to stop the task...
  return false;
}

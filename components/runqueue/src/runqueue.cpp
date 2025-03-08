#include "runqueue.hpp"

using namespace espp;

RunQueue::RunQueue(const RunQueue::Config &config)
    : BaseComponent("RunQueue", config.log_level) {
  using namespace std::placeholders;
  runner_ = espp::Task::make_unique({
      .callback = std::bind(&RunQueue::task_fn, this, _1, _2, _3),
      .task_config = config.task_config,
  });
  if (config.auto_start) {
    start();
  }
}

RunQueue::~RunQueue() { stop(); }

std::size_t RunQueue::queue_size() const { return priority_function_queue_.size(); }

bool RunQueue::is_running() const { return runner_->is_running(); }

void RunQueue::start() {
  logger_.debug("Starting run queue");
  runner_->start();
}

void RunQueue::stop() {
  logger_.debug("Stopping run queue");
  // notify the queue so that the runner wakes up and exits
  {
    std::unique_lock lock(queue_mutex_);
    queue_notified_ = true;
  }
  queue_cv_.notify_all();
  runner_->stop();
}

RunQueue::Id RunQueue::add_function(const Function &function, Priority priority) {
  // generate an Id for this function
  Id id = ++id_counter_; // pre-increment so the first id is 1
  if (id == INVALID_ID) {
    // if the id is min, then increment again. this should be rare, and only
    // happen if the id_counter_ overflows
    id = ++id_counter_;
  }
  std::unique_lock lock(queue_mutex_);
  priority_function_queue_.insert({priority, id, function});
  // notify the queue so that the runner wakes up and runs the function
  queue_cv_.notify_all();
  return id;
}

bool RunQueue::remove_function(RunQueue::Id id) {
  // return false if the id is invalid
  if (id == INVALID_ID) {
    return false;
  }
  // return false if the function is the one currently running, as we cannot
  // remove the running function
  if (id == running_id_) {
    return false;
  }
  logger_.debug("Removing function from queue with id: {}", id);
  std::unique_lock lock(queue_mutex_);
  auto it = std::find_if(priority_function_queue_.begin(), priority_function_queue_.end(),
                         [&](const auto &pf) { return pf.id == id; });
  if (it != priority_function_queue_.end()) {
    priority_function_queue_.erase(it);
    return true;
  }
  return false;
}

bool RunQueue::is_function_queued(RunQueue::Id id) {
  // return false if the id is invalid
  if (id == INVALID_ID) {
    return false;
  }
  // return true if the function is the one currently running
  if (id == running_id_) {
    return true;
  }
  // otherwise, check if the function is in the queue
  std::unique_lock lock(queue_mutex_);
  auto it = std::find_if(priority_function_queue_.begin(), priority_function_queue_.end(),
                         [&](const auto &pf) { return pf.id == id; });
  // return whether or not the function was found
  return it != priority_function_queue_.end();
}

void RunQueue::clear_queue() {
  logger_.debug("Clearing queue");
  std::unique_lock lock(queue_mutex_);
  priority_function_queue_.clear();
}

std::vector<RunQueue::Id> RunQueue::get_queued_ids(bool include_running) {
  std::vector<Id> ids;
  // Note: the vector will be in order of priority, with the highest priority
  // function at the end of the vector
  {
    std::unique_lock lock(queue_mutex_);
    for (const auto &pf : priority_function_queue_) {
      ids.push_back(pf.id);
    }
  }
  if (include_running && running_id_ != INVALID_ID) {
    ids.push_back(running_id_);
  }
  return ids;
}

std::optional<RunQueue::Id> RunQueue::get_running_id() {
  if (running_id_ != INVALID_ID) {
    return running_id_;
  }
  return std::nullopt;
}

bool RunQueue::manage_queue() {
  logger_.debug("Managing queue");
  // check to see if there are any functions in the queue, and if so, then get
  // the highest priority function and run it
  PriorityFunction highest_priority_function;
  {
    std::unique_lock lock(queue_mutex_);
    if (priority_function_queue_.empty()) {
      // there are no functions in the queue, so return false
      return false;
    }
    // store the function to run. rbegin() is the highest priority function
    auto top = priority_function_queue_.rbegin();
    highest_priority_function = *top;
    // remove the function from the queue
    priority_function_queue_.erase(std::next(top).base());
  }

  // set the running id
  running_id_ = highest_priority_function.id;

  logger_.debug("Running function with priority: {}", highest_priority_function.priority);
  // run the function
  highest_priority_function.function();

  // clear the running id
  running_id_ = INVALID_ID;

  // return whether or not there are more functions in the queue
  std::unique_lock lock(queue_mutex_);
  return !priority_function_queue_.empty();
}

bool RunQueue::task_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  // run manage queue, and if it returns false, then wait for the queue cv to be
  // notified
  if (!manage_queue()) {
    logger_.debug("Waiting for queue to be notified");
    std::unique_lock lock(queue_mutex_);
    queue_cv_.wait(lock, [&] { return queue_notified_ || !priority_function_queue_.empty(); });
    if (queue_notified_) {
      // the queue was notified, so we should return true to indicate that the
      // task is being stopped
      return true;
    }
  }

  // NOTE: we should probably sleep here to ensure we don't consume all the CPU

  // return whether or not the task was notified. if the task was notified, that
  // means the task is being stopped, so we will simply return it (true)
  // indicating we are ready to stop.
  std::unique_lock lock(m); // lock before access to task_notified
  logger_.debug("Task notified: {}", task_notified);
  return task_notified;
}

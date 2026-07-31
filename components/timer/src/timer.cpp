#include "timer.hpp"

using namespace espp;

Timer::Timer(const Timer::Config &config)
    : BaseComponent(config.name, config.log_level)
    , period_(std::chrono::duration_cast<std::chrono::microseconds>(config.period))
    , delay_(std::chrono::duration_cast<std::chrono::microseconds>(config.delay))
    , callback_(config.callback) {
  // set the logger rate limit
  logger_.set_rate_limit(std::chrono::milliseconds(100));
  // make the task
  task_ = espp::Task::make_unique({
      .callback = std::bind(&Timer::timer_callback_fn, this, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3),
      .task_config =
          {
              .name = std::string(config.name) + "_task",
              .stack_size_bytes = config.stack_size_bytes,
              .priority = config.priority,
              .core_id = config.core_id,
          },
      .log_level = config.log_level,
  });
  period_float = std::chrono::duration<float>(period_).count();
  delay_float = std::chrono::duration<float>(delay_).count();
  if (config.auto_start) {
    start();
  }
}

Timer::Timer(const Timer::AdvancedConfig &config)
    : BaseComponent(config.task_config.name, config.log_level)
    , period_(std::chrono::duration_cast<std::chrono::microseconds>(config.period))
    , delay_(std::chrono::duration_cast<std::chrono::microseconds>(config.delay))
    , callback_(config.callback) {
  // set the logger rate limit
  logger_.set_rate_limit(std::chrono::milliseconds(100));
  // make the task
  task_ = espp::Task::make_unique({
      .callback = std::bind(&Timer::timer_callback_fn, this, std::placeholders::_1,
                            std::placeholders::_2, std::placeholders::_3),
      .task_config = config.task_config,
      .log_level = config.log_level,
  });
  period_float = std::chrono::duration<float>(period_).count();
  if (period_float < 0) {
    logger_.warn("period cannot be negative, setting to 0");
    period_ = std::chrono::microseconds(0);
    period_float = 0;
  }
  delay_float = std::chrono::duration<float>(delay_).count();
  if (delay_float < 0) {
    logger_.warn("delay cannot be negative, setting to 0");
    delay_ = std::chrono::microseconds(0);
    delay_float = 0;
  }
  if (config.auto_start) {
    start();
  }
}

Timer::~Timer() { cancel(); }

bool Timer::start() {
  if (is_running()) {
    logger_.info("timer is already running, not starting");
    return true;
  }
  // set the flag here to avoid race condition
  running_ = true;
  float local_period_float;
  float local_delay_float;
  std::chrono::time_point<std::chrono::steady_clock> local_wakeup_time;
  std::chrono::time_point<std::chrono::steady_clock> local_start_time;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    start_time_ = std::chrono::steady_clock::now();
    wakeup_time_ = start_time_;
    if (delay_float > 0) {
      wakeup_time_ += delay_;
      delay_wakeup_time_ = wakeup_time_;
    }
    if (period_float > 0) {
      wakeup_time_ += period_;
    }
    local_period_float = period_float;
    local_delay_float = delay_float;
    local_wakeup_time = wakeup_time_;
    local_start_time = start_time_;
  }
  if (task_->start()) {
    logger_.info("Started with period {:.3f} s and delay {:.3f} s. Will wake up in {:.3f} s",
                 local_period_float, local_delay_float,
                 std::chrono::duration<float>(local_wakeup_time - local_start_time).count());
    return true;
  }
  // reset the flag if the task failed to start
  running_ = false;
  logger_.error("failed to start timer task");
  return false;
}

bool Timer::start(const std::chrono::duration<float> &delay) {
  if (delay.count() < 0) {
    logger_.warn("delay cannot be negative, not starting");
    return false;
  }
  if (is_running()) {
    logger_.info("restarting with delay {:.3f} s", delay.count());
    cancel();
  }
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    delay_ = std::chrono::duration_cast<std::chrono::microseconds>(delay);
    delay_float = std::chrono::duration<float>(delay_).count();
  }
  return start();
}

void Timer::stop() { cancel(); }

void Timer::cancel() {
  logger_.info("canceling");
  running_ = false;
  // cancel the task
  task_->stop();
}

#if defined(ESP_PLATFORM)
bool Timer::start_watchdog() { return task_->start_watchdog(); }

bool Timer::stop_watchdog() { return task_->stop_watchdog(); }
#endif

void Timer::set_period(const std::chrono::duration<float> &period) {
  if (period.count() < 0) {
    logger_.warn("period cannot be negative, not setting");
    return;
  }
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    period_ = std::chrono::duration_cast<std::chrono::microseconds>(period);
    period_float = std::chrono::duration<float>(period_).count();
  }
  logger_.info("Period set to {:.3f} s", period.count());
}

bool Timer::is_running() const { return running_ && task_->is_running(); }

bool Timer::timer_callback_fn(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
  logger_.debug("callback entered");
  if (!running_) {
    // stop the timer, the timer was canceled
    logger_.debug("timer was canceled, stopping");
    return true;
  }
  if (!callback_) {
    // stop the timer, the callback is null
    logger_.debug("callback is null, stopping");
    running_ = false;
    return true;
  }

  // initial delay, if any - this is only used the first time the timer
  // runs
  float local_delay_float;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    local_delay_float = delay_float;
  }
  if (local_delay_float > 0) {
    std::chrono::time_point<std::chrono::steady_clock> local_delay_wakeup_time;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      local_delay_wakeup_time = delay_wakeup_time_;
    }
    logger_.debug("waiting for delay {:.3f} s", local_delay_float);
    {
      std::unique_lock<std::mutex> lock(m);
      cv.wait_until(lock, local_delay_wakeup_time, [&task_notified] { return task_notified; });
      // reset the task_notified flag
      task_notified = false;
    }
    if (!running_) {
      logger_.debug("delay canceled, stopping");
      return true;
    }
    // now set the delay to 0
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      delay_ = std::chrono::microseconds(0);
      delay_float = 0;
    }
  }

  // now run the callback
  logger_.debug("running callback");
  auto start_time = std::chrono::steady_clock::now();
  bool requested_stop = callback_();
  auto end = std::chrono::steady_clock::now();

  std::chrono::time_point<std::chrono::steady_clock> local_wakeup_time;
  std::chrono::microseconds local_period;
  float local_period_float;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    local_wakeup_time = wakeup_time_;
    local_period = period_;
    local_period_float = period_float;
  }

  if (requested_stop || local_period_float <= 0) {
    // stop the timer if requested or if the period is <= 0
    logger_.debug("callback requested stop or period is <= 0, stopping");
    running_ = false;
    return true;
  }

  if (local_wakeup_time <= end) {
    // if the callback took longer (or just as long) than the period (so it is
    // already past the next wakeup time), log a warning and ensure that the
    // next wakeup time is the closest multiple of the period after the current
    float elapsed = std::chrono::duration<float>(end - start_time).count();
    if (elapsed >= local_period_float) {
      logger_.warn_rate_limited("callback took ~longer ({:.3f} s) than period ({:.3f} s)", elapsed,
                                local_period_float);
    }
    if (local_period.count() <= 0) {
      // period changed to oneshot while running; stop after this callback
      running_ = false;
      return true;
    }
    // update the next wakeup time to the closest multiple of the period after the current time
    size_t n = (end - local_wakeup_time) / local_period + 1;
    // only log if we are skipping more than one period
    if (n > 1) {
      logger_.warn("Already passed expected wakeup time, skipping {} periods", n);
    }
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    wakeup_time_ += n * local_period;
    // return immediately to execute the next callback iteration
    return false;
  }

  // now wait for the period (taking into account the time it took to run
  // the callback)
  {
    std::unique_lock<std::mutex> lock(m);
    cv.wait_until(lock, local_wakeup_time, [&task_notified] { return task_notified; });
    // reset the task_notified flag
    task_notified = false;
  }

  // now that we've waited, make sure the next wakeup time is the next multiple
  // of the period after the last
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    wakeup_time_ += period_;
  }

  // keep the timer running
  return false;
}

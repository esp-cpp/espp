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
  delay_float = std::chrono::duration<float>(delay_).count();
  if (config.auto_start) {
    start();
  }
}

Timer::~Timer() { cancel(); }

void Timer::start() {
  logger_.info("starting with period {:.3f} s and delay {:.3f} s", period_float, delay_float);
  running_ = true;
  // start the task
  task_->start();
}

void Timer::start(const std::chrono::duration<float> &delay) {
  if (delay.count() < 0) {
    logger_.warn("delay cannot be negative, not starting");
    return;
  }
  if (is_running()) {
    logger_.info("restarting with delay {:.3f} s", delay.count());
    cancel();
  }
  delay_ = std::chrono::duration_cast<std::chrono::microseconds>(delay);
  delay_float = std::chrono::duration<float>(delay_).count();
  start();
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
  period_ = std::chrono::duration_cast<std::chrono::microseconds>(period);
  period_float = std::chrono::duration<float>(period_).count();
  logger_.info("setting period to {:.3f} s", period_float);
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
  if (delay_float > 0) {
    auto start_time = std::chrono::steady_clock::now();
    logger_.debug("waiting for delay {:.3f} s", delay_float);
    std::unique_lock<std::mutex> lock(m);
    cv.wait_until(lock, start_time + delay_, [&task_notified] { return task_notified; });
    // reset the task_notified flag
    task_notified = false;
    if (!running_) {
      logger_.debug("delay canceled, stopping");
      return true;
    }
    // now set the delay to 0
    delay_ = std::chrono::microseconds(0);
    delay_float = 0;
  }
  // now run the callback
  auto start_time = std::chrono::steady_clock::now();
  logger_.debug("running callback");
  bool requested_stop = callback_();
  if (requested_stop || period_float <= 0) {
    // stop the timer if requested or if the period is <= 0
    logger_.debug("callback requested stop or period is <= 0, stopping");
    running_ = false;
    return true;
  }
  auto end = std::chrono::steady_clock::now();
  float elapsed = std::chrono::duration<float>(end - start_time).count();
  if (elapsed > period_float) {
    // if the callback took longer than the period, then we should just
    // return and run the callback again immediately
    logger_.warn_rate_limited("callback took longer ({:.3f} s) than period ({:.3f} s)", elapsed,
                              period_float);
    return false;
  }
  // now wait for the period (taking into account the time it took to run
  // the callback)
  {
    std::unique_lock<std::mutex> lock(m);
    cv.wait_until(lock, start_time + period_, [&task_notified] { return task_notified; });
    // reset the task_notified flag
    task_notified = false;
  }
  // keep the timer running
  return false;
}

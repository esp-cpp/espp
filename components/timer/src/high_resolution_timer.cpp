#include "high_resolution_timer.hpp"

using namespace espp;

HighResolutionTimer::HighResolutionTimer(const Config &config)
    : BaseComponent(config.name, config.log_level)
    , skip_unhandled_events_(config.skip_unhandled_events)
    , dispatch_method_(config.dispatch_method)
    , callback_(config.callback) {
  using namespace std::chrono_literals;
  // set a default logger rate limit (can always be set later by caller)
  set_log_rate_limit(100ms);
}

HighResolutionTimer::~HighResolutionTimer() {
  stop_watchdog();
  stop();
  if (timer_handle_) {
    esp_timer_delete(timer_handle_);
    timer_handle_ = nullptr;
  }
}

bool HighResolutionTimer::start(uint64_t period_us, bool oneshot) {
  if (is_running()) {
    stop();
  }

  // store whether the timer is oneshot or periodic
  oneshot_ = oneshot;

  esp_err_t err = ESP_OK;

  if (timer_handle_ == nullptr) {
    esp_timer_create_args_t timer_args;
    timer_args.callback = timer_callback;
    timer_args.arg = this;
    timer_args.dispatch_method = dispatch_method_;
    timer_args.name = get_name().c_str();
    timer_args.skip_unhandled_events = skip_unhandled_events_;

    err = esp_timer_create(&timer_args, &timer_handle_);
    if (err != ESP_OK) {
      logger_.error("Failed to create timer: {}", esp_err_to_name(err));
      return false;
    }
  }

  if (oneshot) {
    err = esp_timer_start_once(timer_handle_, period_us);
  } else {
    err = esp_timer_start_periodic(timer_handle_, period_us);
  }

  if (err != ESP_OK) {
    logger_.error("Failed to start timer: {}", esp_err_to_name(err));
    return false;
  }
  return true;
}

bool HighResolutionTimer::start_watchdog() {
#if !CONFIG_ESP_TASK_WDT_EN
  logger_.debug("Watchdog timer not enabled in sdkconfig");
  return false;
#else
  if (wdt_handle_) {
    logger_.debug("Watchdog timer already running");
    return false;
  }
  auto err = esp_task_wdt_add_user(get_name().c_str(), &wdt_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to start watchdog timer: {}", esp_err_to_name(err));
    wdt_handle_ = nullptr;
    return false;
  }
  return true;
#endif // CONFIG_ESP_TASK_WDT_EN
}

bool HighResolutionTimer::stop_watchdog() {
#if !CONFIG_ESP_TASK_WDT_EN
  logger_.debug("Watchdog timer not enabled in sdkconfig");
  return false;
#else
  if (!wdt_handle_) {
    logger_.debug("Watchdog timer not running");
    return false;
  }
  auto err = esp_task_wdt_delete_user(wdt_handle_);
  if (err != ESP_OK) {
    logger_.error("Failed to stop watchdog timer: {}", esp_err_to_name(err));
    return false;
  }
  wdt_handle_ = nullptr;
  return true;
#endif // CONFIG_ESP_TASK_WDT_EN
}

bool HighResolutionTimer::oneshot(uint64_t timeout_us) { return start(timeout_us, true); }

bool HighResolutionTimer::periodic(uint64_t period_us) { return start(period_us, false); }

void HighResolutionTimer::stop() {
  if (is_running()) {
    esp_timer_stop(timer_handle_);
  }
}

bool HighResolutionTimer::is_running() const {
  return timer_handle_ && esp_timer_is_active(timer_handle_);
}

bool HighResolutionTimer::is_oneshot() const { return oneshot_; };

bool HighResolutionTimer::is_periodic() const { return !oneshot_; };

void HighResolutionTimer::set_period(uint64_t period_us) {
  esp_err_t err = ESP_OK;
  if (is_running()) {
    err = esp_timer_restart(timer_handle_, period_us);
  } else if (timer_handle_) {
    if (oneshot_) {
      err = esp_timer_start_once(timer_handle_, period_us);
    } else {
      err = esp_timer_start_periodic(timer_handle_, period_us);
    }
  } else {
    logger_.error("Cannot set period for timer, start() has not been called!");
  }
  if (err != ESP_OK) {
    logger_.error("Failed to set timer period: {}", esp_err_to_name(err));
  }
}

uint64_t HighResolutionTimer::get_period() {
  uint64_t period_us = 0;
  esp_err_t err = ESP_OK;
  if (oneshot_) {
    err = esp_timer_get_expiry_time(timer_handle_, &period_us);
  } else {
    err = esp_timer_get_period(timer_handle_, &period_us);
  }
  if (err != ESP_OK) {
    logger_.error("Failed to get timer period: {}", esp_err_to_name(err));
    return 0;
  }
  logger_.debug("Timer period for {} timer: {} us", oneshot_.load() ? "oneshot" : "periodic",
                period_us);
  return period_us;
}

void HighResolutionTimer::timer_callback(void *arg) {
  auto timer = static_cast<HighResolutionTimer *>(arg);
  if (!timer) {
    return;
  }
  timer->handle_timer_callback();
}

void HighResolutionTimer::handle_timer_callback() {
  logger_.debug_rate_limited("Timer expired, calling callback");
  if (callback_) {
    callback_();
  }
#if CONFIG_ESP_TASK_WDT_EN
  if (wdt_handle_) {
    esp_task_wdt_reset_user(wdt_handle_);
  }
#endif // CONFIG_ESP_TASK_WDT_EN
}

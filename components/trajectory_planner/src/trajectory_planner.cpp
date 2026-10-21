#include "trajectory_planner.hpp"

#include "detail/trajectory_planner_math.hpp"

namespace espp {

TrajectoryPlanner::TrajectoryPlanner(const Config &config)
    : BaseComponent("TrajectoryPlanner", config.log_level) {
  if (set_config(config, /*reset_state=*/true)) {
    start_task();
  }
}

TrajectoryPlanner::~TrajectoryPlanner() { stop_task(); }

bool TrajectoryPlanner::set_config(const Config &config, bool reset_state) {
  const float planning_ms =
      std::chrono::duration<float, std::milli>(config.planning_period).count();
  if (planning_ms <= 0.0f) {
    logger_.error("planning_period must be > 0");
    return false;
  } else if (planning_ms < 5.0f || planning_ms > 200.0f) {
    logger_.warn("planning_period {:.1f} ms is outside recommended 5–200 ms range", planning_ms);
  }
  if (config.max_linear_velocity <= 0.0f) {
    logger_.error("max_linear_velocity must be > 0");
    return false;
  }
  if (config.max_reverse_linear_velocity < 0.0f) {
    logger_.error("max_reverse_linear_velocity must be >= 0 (0 = same as max_linear_velocity)");
    return false;
  }
  if (config.max_angular_velocity <= 0.0f) {
    logger_.error("max_angular_velocity must be > 0");
    return false;
  }

  if (config.driving_profile.max_linear_acceleration <= 0.0f) {
    logger_.error("driving_profile.max_linear_acceleration must be > 0");
    return false;
  }
  if (config.driving_profile.max_angular_acceleration <= 0.0f) {
    logger_.error("driving_profile.max_angular_acceleration must be > 0");
    return false;
  }
  // a stopping profile with no acceleration could never bring the chassis to rest
  if (config.stopping_profile.max_linear_acceleration <= 0.0f) {
    logger_.error("stopping_profile.max_linear_acceleration must be > 0");
    return false;
  }
  if (config.stopping_profile.max_angular_acceleration <= 0.0f) {
    logger_.error("stopping_profile.max_angular_acceleration must be > 0");
    return false;
  }

  if (config.driving_profile.max_linear_jerk * config.planning_period.count() >
      config.driving_profile.max_linear_acceleration) {
    logger_.warn(
        "Driving profile linear jerk exceeds linear acceleration over the planning period, jerk "
        "acts like infinite jerk. Consider reducing the jerk or increasing the planning period.");
  }
  if (config.driving_profile.max_angular_jerk * config.planning_period.count() >
      config.driving_profile.max_angular_acceleration) {
    logger_.warn(
        "Driving profile angular jerk exceeds angular acceleration over the planning period, jerk "
        "acts like infinite jerk. Consider reducing the jerk or increasing the planning period.");
  }

  if (config.stopping_profile.max_linear_jerk * config.planning_period.count() >
      config.stopping_profile.max_linear_acceleration) {
    logger_.warn(
        "Stopping profile linear jerk exceeds linear acceleration over the planning period, jerk "
        "acts like infinite jerk. Consider reducing the jerk or increasing the planning period.");
  }
  if (config.stopping_profile.max_angular_jerk * config.planning_period.count() >
      config.stopping_profile.max_angular_acceleration) {
    logger_.warn(
        "Stopping profile angular jerk exceeds angular acceleration over the planning period, jerk "
        "acts like infinite jerk. Consider reducing the jerk or increasing the planning period.");
  }

  logger_.info("Updated config: {}", config);
  // Copy the configuration (and its callback) before taking mutex_, and let the
  // previous ones die after releasing it: copying or destroying a std::function
  // that wraps a python callable takes the GIL, and a thread holding the GIL may
  // be waiting for mutex_ (see output_callback_ in the header).
  Config new_config = config;
  auto new_callback = std::make_shared<const output_callback_t>(config.output_callback);
  {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    std::swap(config_, new_config);
    output_callback_.swap(new_callback);
    if (reset_state) {
      reset();
    }
  }
  return true;
}

float TrajectoryPlanner::reverse_velocity_limit_locked() const {
  return config_.max_reverse_linear_velocity > 0.0f ? config_.max_reverse_linear_velocity
                                                    : config_.max_linear_velocity;
}

const TrajectoryPlanner::Config &TrajectoryPlanner::get_config() const {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  return config_;
}

void TrajectoryPlanner::set_target(float linear, float angular) {

  if (std::isnan(linear) || std::isnan(angular)) {
    logger_.error("set_target() called with NaN values: linear={}, angular={}", linear, angular);
    return;
  }
  // clamp the input to [-1, +1] first
  linear = std::clamp(linear, -1.0f, 1.0f);
  angular = std::clamp(angular, -1.0f, 1.0f);
  // the target should be clamped to a vector of a unit circle
  // Motion envelope enforcement: ensure that the combined normalized linear and angular velocities
  // do not exceed the unit circle. This prevents commanding a motion that exceeds the robot's
  // maximum capabilities.
  bool enforce_motion_envelope = false;
  float max_centripetal_acceleration = 0.0f;
  float max_linear_velocity = 0.0f;
  float max_reverse_velocity = 0.0f;
  float max_angular_velocity = 0.0f;
  {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    enforce_motion_envelope = config_.enforce_motion_envelope;
    max_centripetal_acceleration = config_.max_centripetal_acceleration;
    max_linear_velocity = config_.max_linear_velocity;
    max_reverse_velocity = reverse_velocity_limit_locked();
    max_angular_velocity = config_.max_angular_velocity;
  }

  if (enforce_motion_envelope) {
    float magnitude = std::sqrt(linear * linear + angular * angular);
    if (magnitude > 1.0f) {
      linear /= magnitude;
      angular /= magnitude;
      logger_.warn("Motion envelope enforced: linear={}, angular={}, magnitude={}", linear, angular,
                   magnitude);
    }
  }

  // full forward stick = max_linear_velocity, full reverse stick = the reverse limit
  auto target_v = linear * (linear < 0.0f ? max_reverse_velocity : max_linear_velocity);
  auto target_w = angular * max_angular_velocity;

  // Centripetal acceleration limit: |v · w| ≤ a_c_max
  // Scale both v and w proportionally to preserve the turning radius.
  if (max_centripetal_acceleration > 0.0f) {
    float a_c = std::abs(target_v * target_w);
    if (a_c > max_centripetal_acceleration) {
      float scale = std::sqrt(max_centripetal_acceleration / a_c);
      target_v *= scale;
      target_w *= scale;
      logger_.warn(
          "Centripetal acceleration limit enforced: target_v={}, target_w={}, scale={}, a_c={}",
          target_v, target_w, scale, a_c);
    }
  }

  logger_.debug("Target: v={:.3f} m/s, w={:.3f} rad/s", target_v, target_w);
  {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    target_v_ = target_v;
    target_w_ = target_w;
  }
}

std::pair<float, float> TrajectoryPlanner::get_target() const {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  const float v_limit =
      target_v_ < 0.0f ? reverse_velocity_limit_locked() : config_.max_linear_velocity;
  return {target_v_ / v_limit, target_w_ / config_.max_angular_velocity};
}

void TrajectoryPlanner::update(float dt) {
  if (dt <= 0.0f) {
    logger_.warn("update() called with non-positive dt={:.3f}, skipping update", dt);
    return;
  }
  logger_.debug("Updating TP with dt={:.3f}", dt);
  {
    std::lock_guard<std::recursive_mutex> lk(mutex_);

    const bool is_stopping = (target_v_ == 0.0f && target_w_ == 0.0f);
    const MotionProfile &profile = is_stopping ? config_.stopping_profile : config_.driving_profile;

    // Each axis is stepped on its own: S-curve where its jerk limit is > 0,
    // trapezoidal where it is 0. The acceleration actually applied is kept
    // in the state either way, so a jerk-limited step after a trapezoidal
    // one (a stop, then driving again) resumes from the real value.
    using detail::trajectory_planner::AxisState;
    using detail::trajectory_planner::step_axis;
    AxisState lin{.v = state_.v, .a = state_.a_v};
    AxisState ang{.v = state_.w, .a = state_.a_w};
    step_axis(lin, target_v_, -reverse_velocity_limit_locked(), config_.max_linear_velocity,
              profile.max_linear_acceleration, profile.max_linear_jerk, dt);
    step_axis(ang, target_w_, -config_.max_angular_velocity, config_.max_angular_velocity,
              profile.max_angular_acceleration, profile.max_angular_jerk, dt);
    state_.v = lin.v;
    state_.a_v = lin.a;
    state_.w = ang.v;
    state_.a_w = ang.a;

    logger_.debug("State: {}, target_v={:.3f}, target_w={:.3f}", state_, target_v_, target_w_);
  } // mutex_ released here

  {
    std::lock_guard<std::mutex> lk(callback_wake_m_);
    callback_wake_flag_ = true;
  }
  callback_wake_cv_.notify_one();
}

TrajectoryPlanner::MotionCommand TrajectoryPlanner::output() const {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  return MotionCommand{.linear_velocity = state_.v, .angular_velocity = state_.w};
}

void TrajectoryPlanner::stop() { set_target(0.0f, 0.0f); }

bool TrajectoryPlanner::start_task() {
  if (timer_ && timer_->is_running()) {
    return false;
  }
  last_update_time_ = std::chrono::steady_clock::now();
  timer_ = std::make_unique<espp::Timer>(espp::Timer::AdvancedConfig{
      .period = config_.planning_period,
      .callback = [this]() -> bool {
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_update_time_).count();
        last_update_time_ = now;
        update(dt);
        return false;
      },
      .auto_start = true,
      .task_config = config_.planning_task_config,
  });
  callback_wake_flag_ = false;
  callback_stop_flag_ = false;
  callback_task_ = espp::Task::make_unique({
      .callback = [this]() -> bool {
        // Wait indefinitely for a notification from update() (no polling timeout needed).
        {
          std::unique_lock<std::mutex> lk(callback_wake_m_);
          callback_wake_cv_.wait(lk, [this] { return callback_wake_flag_ || callback_stop_flag_; });
          if (callback_stop_flag_)
            return true; // stop the task
          callback_wake_flag_ = false;
        }
        // snapshot the callback by shared_ptr (no std::function copy under mutex_,
        // see output_callback_) and call it with the mutex released
        std::shared_ptr<const output_callback_t> cb;
        MotionCommand cmd;
        {
          std::lock_guard<std::recursive_mutex> lk(mutex_);
          cb = output_callback_;
          cmd = MotionCommand{.linear_velocity = state_.v, .angular_velocity = state_.w};
        }
        if (cb && *cb)
          (*cb)(cmd);
        return false;
      },
      .task_config = config_.callback_task_config,
  });
  callback_task_->start();
  return timer_->is_running() && callback_task_->is_started();
}

bool TrajectoryPlanner::stop_task() {
  if (!timer_) {
    return false;
  }
  timer_->cancel();
  // Wake the callback task so it exits promptly instead of waiting for the timeout.
  {
    std::lock_guard<std::mutex> lk(callback_wake_m_);
    callback_stop_flag_ = true;
  }
  callback_wake_cv_.notify_one();
  if (callback_task_)
    callback_task_->stop();
  return true;
}

bool TrajectoryPlanner::is_running() const {
  return timer_ && timer_->is_running() && callback_task_ && callback_task_->is_started();
}

void TrajectoryPlanner::reset() {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  state_ = State{};
  target_v_ = 0.0f;
  target_w_ = 0.0f;
}

} // namespace espp

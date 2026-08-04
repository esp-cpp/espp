#include "trajectory_planner.hpp"

namespace espp {

TrajectoryPlanner::TrajectoryPlanner(const Config &config)
    : BaseComponent("TrajectoryPlanner", config.log_level) {
  set_config(config, /*reset_state=*/true);
  start_task();
}

TrajectoryPlanner::~TrajectoryPlanner() { stop_task(); }

void TrajectoryPlanner::set_config(const Config &config, bool reset_state) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  const float planning_ms =
      std::chrono::duration<float, std::milli>(config.planning_period).count();
  if (planning_ms <= 0.0f) {
    logger_.error("planning_period must be > 0");
    return;
  } else if (planning_ms < 5.0f || planning_ms > 200.0f) {
    logger_.warn("planning_period {:.1f} ms is outside recommended 5–200 ms range", planning_ms);
  }
  const float callback_ms =
      std::chrono::duration<float, std::milli>(config.callback_period).count();
  if (callback_ms <= 0.0f) {
    logger_.error("callback_period must be > 0");
    return;
  } else if (callback_ms < 2.0f * planning_ms) {
    logger_.warn("callback_period {:.1f} ms is less than 2× planning_period {:.1f} ms - repeated "
                 "outputs likely",
                 callback_ms, planning_ms);
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
  config_ = config;
  output_callback_ = config.output_callback;
  if (reset_state) {
    reset();
  }
}

const TrajectoryPlanner::Config &TrajectoryPlanner::get_config() const { return config_; }

void TrajectoryPlanner::set_target(float linear, float angular) {

  // clamp the input to [-1, +1] first
  linear = std::clamp(linear, -1.0f, 1.0f);
  angular = std::clamp(angular, -1.0f, 1.0f);
  // the target should be clamped to a vector of a unit circle
  // Motion envelope enforcement: ensure that the combined normalized linear and angular velocities
  // do not exceed the unit circle. This prevents commanding a motion that exceeds the robot's
  // maximum capabilities.
  if (config_.enforce_motion_envelope) {
    float magnitude = std::sqrt(linear * linear + angular * angular);
    if (magnitude > 1.0f) {
      linear /= magnitude;
      angular /= magnitude;
      logger_.warn("Motion envelope enforced: linear={}, angular={}, magnitude={}", linear, angular,
                   magnitude);
    }
  }

  std::lock_guard<std::recursive_mutex> lk(mutex_);
  target_v_ = linear * config_.max_linear_velocity;
  target_w_ = angular * config_.max_angular_velocity;

  // Centripetal acceleration limit: |v · w| ≤ a_c_max
  // Scale both v and w proportionally to preserve the turning radius.
  if (config_.max_centripetal_acceleration > 0.0f) {
    float a_c = std::abs(target_v_ * target_w_);
    if (a_c > config_.max_centripetal_acceleration) {
      float scale = std::sqrt(config_.max_centripetal_acceleration / a_c);
      target_v_ *= scale;
      target_w_ *= scale;
      logger_.warn(
          "Centripetal acceleration limit enforced: target_v={}, target_w={}, scale={}, a_c={}",
          target_v_, target_w_, scale, a_c);
    }
  }

  logger_.debug("Target: v={:.3f} m/s, w={:.3f} rad/s", target_v_, target_w_);
}

std::pair<float, float> TrajectoryPlanner::get_target() const {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  return {target_v_ / config_.max_linear_velocity, target_w_ / config_.max_angular_velocity};
}

void TrajectoryPlanner::update(float dt) {
  if (dt <= 0.0f) {
    return;
  }
  // logger_.info("Updating TP with dt={:.3f}", dt);
  {
    std::lock_guard<std::recursive_mutex> lk(mutex_);

    const bool is_stopping = (target_v_ == 0.0f && target_w_ == 0.0f);
    const MotionProfile &profile = is_stopping ? config_.stopping_profile : config_.driving_profile;

    const bool jerk_limited = (profile.max_linear_jerk > 0.0f) || (profile.max_angular_jerk > 0.0f);

    if (jerk_limited) {
      // Minimum velocity-change distance when decelerating |a| to 0 at max jerk.
      auto sd = [](float a_abs, float step, float dt_) -> float {
        int n = static_cast<int>(a_abs / step);
        return n * a_abs * dt_ - n * (n + 1) * 0.5f * step * dt_;
      };

      // Returns {d_now, d_maintain, d_increase} signed by sign(a). Matches Python
      // stopping_thresholds().
      auto stopping_thresh = [&sd](float a, float step,
                                   float dt_) -> std::tuple<float, float, float> {
        if (a == 0.0f)
          return {0.0f, 0.0f, step * dt_};
        float sign = a > 0.0f ? 1.0f : -1.0f;
        float a_abs = std::abs(a);
        float d_now = sd(a_abs, step, dt_);
        float d_maintain = a_abs * dt_ + d_now;
        float d_increase = (a_abs + step) * dt_ + sd(a_abs + step, step, dt_);
        return {sign * d_now, sign * d_maintain, sign * d_increase};
      };

      // Per-axis discrete optimal-control update. Matches the Python decision block.
      auto update_accel = [&stopping_thresh, &logger_ = logger_](float &a, float dist, float step,
                                                                 float a_max, float dt_) {
        auto [d_now, d_maintain, d_increase] = stopping_thresh(a, step, dt_);
        float a_prev = a;

        if (dist > 0.0f) {
          if (dist > d_increase)
            a += step;
          else if (dist <= d_maintain) {
            logger_.debug("Dist <= d_maintain: a={}, dist={}, step={}, dt_={}", a, dist, step, dt_);
            auto ideal_a = dist / dt_; // postive
            if (ideal_a <= a && ideal_a >= a - step && ideal_a < step) {
              a = ideal_a;
            } else {
              a -= step;
            }
          }
          // else: maintain current a
        } else if (dist < 0.0f) {
          if (dist < d_increase)
            a -= step;
          else if (dist >= d_maintain) {
            auto ideal_a = dist / dt_; // negative
            if (ideal_a >= a && ideal_a <= a + step && ideal_a > -step) {
              a = ideal_a;
            } else {
              a += step;
            }
          }
          // else: maintain current a
        } else {
          a = 0.0f;
        }

        // Safety: clamp Δa to the jerk limit even if the decision block over-stepped.
        float delta_a = a - a_prev;
        if (std::abs(delta_a) > step)
          a = a_prev + std::copysign(step, delta_a);

        a = std::clamp(a, -a_max, a_max);
      };

      const float step_lj =
          profile.max_linear_jerk > 0.0f ? profile.max_linear_jerk * dt : 1e9f * dt;
      const float step_aj =
          profile.max_angular_jerk > 0.0f ? profile.max_angular_jerk * dt : 1e9f * dt;

      update_accel(state_.a_v, target_v_ - state_.v, step_lj, profile.max_linear_acceleration, dt);
      update_accel(state_.a_w, target_w_ - state_.w, step_aj, profile.max_angular_acceleration, dt);

      state_.v = std::clamp(state_.v + state_.a_v * dt, -config_.max_linear_velocity,
                            config_.max_linear_velocity);
      state_.w = std::clamp(state_.w + state_.a_w * dt, -config_.max_angular_velocity,
                            config_.max_angular_velocity);

    } else {
      // Trapezoidal mode: rate-limit velocity directly by acceleration limit.
      auto rate_limit = [](float current, float target, float max_accel, float dt_) -> float {
        float max_delta = max_accel * dt_;
        return current + std::clamp(target - current, -max_delta, max_delta);
      };

      state_.v = std::clamp(rate_limit(state_.v, target_v_, profile.max_linear_acceleration, dt),
                            -config_.max_linear_velocity, config_.max_linear_velocity);
      state_.w = std::clamp(rate_limit(state_.w, target_w_, profile.max_angular_acceleration, dt),
                            -config_.max_angular_velocity, config_.max_angular_velocity);
    }

    logger_.debug(
        "State: v={:.3f}, w={:.3f}, a_v={:.3f}, a_w={:.3f}, target_v={:.3f}, target_w={:.3f}",
        state_.v, state_.w, state_.a_v, state_.a_w, target_v_, target_w_);
  } // mutex released here
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
  callback_timer_ = std::make_unique<espp::Timer>(espp::Timer::AdvancedConfig{
      .period = config_.callback_period,
      .callback = [this]() -> bool {
        output_callback_t cb;
        MotionCommand cmd;
        {
          std::lock_guard<std::recursive_mutex> lk(mutex_);
          cb = output_callback_;
          cmd = MotionCommand{.linear_velocity = state_.v, .angular_velocity = state_.w};
        }
        if (cb)
          cb(cmd);
        return false;
      },
      .auto_start = true,
      .task_config = config_.callback_task_config,
  });
  return timer_->is_running() && callback_timer_->is_running();
}

bool TrajectoryPlanner::stop_task() {
  if (!timer_) {
    return false;
  }
  timer_->cancel();
  if (callback_timer_)
    callback_timer_->cancel();
  return true;
}

bool TrajectoryPlanner::is_running() const {
  return timer_ && timer_->is_running() && callback_timer_ && callback_timer_->is_running();
}

void TrajectoryPlanner::reset() {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  state_ = State{};
  target_v_ = 0.0f;
  target_w_ = 0.0f;
}

} // namespace espp

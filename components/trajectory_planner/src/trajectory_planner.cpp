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
  logger_.info("Updated config: {}", config);
  config_ = config;
  output_callback_ = config.output_callback;
  if (reset_state) {
    reset();
  }
}

const TrajectoryPlanner::Config &TrajectoryPlanner::get_config() const { return config_; }

void TrajectoryPlanner::set_target(float linear, float angular) {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  target_v_ = std::clamp(linear, -1.0f, 1.0f) * config_.max_linear_velocity;
  target_w_ = std::clamp(angular, -1.0f, 1.0f) * config_.max_angular_velocity;
  logger_.debug("Target: v={:.3f} m/s, w={:.3f} rad/s", target_v_, target_w_);
}

void TrajectoryPlanner::update(float dt) {
  if (dt <= 0.0f) {
    return;
  }
  std::unique_lock<std::recursive_mutex> lk(mutex_);
  const bool is_stopping = (target_v_ == 0.0f && target_w_ == 0.0f);
  const float lin_accel = is_stopping && config_.max_linear_deceleration > 0.0f
                              ? config_.max_linear_deceleration
                              : config_.max_linear_acceleration;
  const float ang_accel = is_stopping && config_.max_angular_deceleration > 0.0f
                              ? config_.max_angular_deceleration
                              : config_.max_angular_acceleration;

  const bool jerk_limited = (config_.max_linear_jerk > 0.0f) || (config_.max_angular_jerk > 0.0f);

  if (jerk_limited) {
    // S-curve mode: rate-limit the acceleration (jerk limit), then integrate velocity.

    // Desired acceleration: close velocity error as fast as the accel limit allows.
    auto desired_accel = [](float target, float current, float dt_, float max_accel) -> float {
      return std::clamp((target - current) / dt_, -max_accel, max_accel);
    };

    float a_v_des = desired_accel(target_v_, state_.v, dt, lin_accel);
    float a_w_des = desired_accel(target_w_, state_.w, dt, ang_accel);

    // Rate-limit change in acceleration by jerk limit.
    auto apply_jerk_limit = [](float current, float desired, float max_jerk, float dt_) -> float {
      float max_delta = max_jerk * dt_;
      return current + std::clamp(desired - current, -max_delta, max_delta);
    };

    float max_lj = config_.max_linear_jerk > 0.0f ? config_.max_linear_jerk : 1e9f;
    float max_aj = config_.max_angular_jerk > 0.0f ? config_.max_angular_jerk : 1e9f;

    state_.a_v =
        std::clamp(apply_jerk_limit(state_.a_v, a_v_des, max_lj, dt), -lin_accel, lin_accel);
    state_.a_w =
        std::clamp(apply_jerk_limit(state_.a_w, a_w_des, max_aj, dt), -ang_accel, ang_accel);

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

    state_.v = std::clamp(rate_limit(state_.v, target_v_, lin_accel, dt),
                          -config_.max_linear_velocity, config_.max_linear_velocity);
    state_.w = std::clamp(rate_limit(state_.w, target_w_, ang_accel, dt),
                          -config_.max_angular_velocity, config_.max_angular_velocity);
  }

  // Motion envelope enforcement: (v/vmax)² + (w/wmax)² ≤ 1
  if (config_.enforce_motion_envelope) {
    float vmax = config_.max_linear_velocity;
    float wmax = config_.max_angular_velocity;
    if (vmax > 0.0f && wmax > 0.0f) {
      float nv = state_.v / vmax;
      float nw = state_.w / wmax;
      float scale = std::sqrt(nv * nv + nw * nw);
      if (scale > 1.0f) {
        state_.v /= scale;
        state_.w /= scale;
      }
    }
  }

  // Centripetal acceleration limit: |v · ω| ≤ a_c_max
  // Scale both v and ω proportionally to preserve the turning radius.
  if (config_.max_centripetal_acceleration > 0.0f) {
    float a_c = std::abs(state_.v * state_.w);
    if (a_c > config_.max_centripetal_acceleration) {
      float scale = std::sqrt(config_.max_centripetal_acceleration / a_c);
      state_.v *= scale;
      state_.w *= scale;
    }
  }

  logger_.debug("State: v={:.3f}, w={:.3f}, a_v={:.3f}, a_w={:.3f}", state_.v, state_.w, state_.a_v,
                state_.a_w);

  // Capture output before releasing the lock.
  MotionCommand cmd{.linear_velocity = state_.v, .angular_velocity = state_.w};
  lk.unlock(); // release before callback to avoid re-entrant deadlock

  if (output_callback_) {
    output_callback_(cmd);
  }
}

TrajectoryPlanner::MotionCommand TrajectoryPlanner::output() const {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  return MotionCommand{.linear_velocity = state_.v, .angular_velocity = state_.w};
}

void TrajectoryPlanner::stop() { set_target(0.0f, 0.0f); }

bool TrajectoryPlanner::start_task() {
  if (task_ && task_->is_started()) {
    return false;
  }
  last_update_time_ = std::chrono::steady_clock::now();
  const auto period = config_.update_period;
  task_ = espp::Task::make_unique({
      .callback = [this, period](std::mutex &m, std::condition_variable &cv,
                                 bool &notified) -> bool {
        {
          std::unique_lock<std::mutex> lk(m);
          cv.wait_for(lk, period, [&notified] { return notified; });
          if (notified) {
            notified = false;
            return true; // task requested to stop
          }
        }
        auto now = std::chrono::steady_clock::now();
        float dt = std::chrono::duration<float>(now - last_update_time_).count();
        last_update_time_ = now;
        update(dt);
        return false; // keep running
      },
      .task_config = config_.task_config,
  });
  return task_->start();
}

bool TrajectoryPlanner::stop_task() {
  if (!task_) {
    return false;
  }
  return task_->stop();
}

bool TrajectoryPlanner::is_running() const { return task_ && task_->is_started(); }

void TrajectoryPlanner::reset() {
  std::lock_guard<std::recursive_mutex> lk(mutex_);
  state_ = State{};
  target_v_ = 0.0f;
  target_w_ = 0.0f;
}

} // namespace espp

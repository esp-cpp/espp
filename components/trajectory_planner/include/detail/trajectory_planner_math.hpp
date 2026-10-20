#pragma once

// The per-axis maths of espp::TrajectoryPlanner, standard-library only so it
// can be unit-tested on the host (see test/trajectory_planner_host_test.cpp).
// TrajectoryPlanner::update() steps its linear and angular axes with
// step_axis(); nothing here knows about targets, profiles or threads.

#include <algorithm>
#include <cmath>
#include <tuple>

namespace espp {
namespace detail {
namespace trajectory_planner {

/// Minimum velocity-change distance when decelerating |a| to 0 at max jerk
/// (`step` = jerk * dt).
inline float stopping_distance(float a_abs, float step, float dt) {
  const int n = static_cast<int>(a_abs / step);
  return n * a_abs * dt - n * (n + 1) * 0.5f * step * dt;
}

/// {d_now, d_maintain, d_increase}, signed like `a`: the velocity change still
/// to come if the acceleration is wound down to zero now, kept for one more
/// step, or increased by one more step. Matches the Python model's
/// stopping_thresholds().
inline std::tuple<float, float, float> stopping_thresholds(float a, float step, float dt) {
  if (a == 0.0f)
    return {0.0f, 0.0f, step * dt};
  const float sign = a > 0.0f ? 1.0f : -1.0f;
  const float a_abs = std::abs(a);
  const float d_now = stopping_distance(a_abs, step, dt);
  const float d_maintain = a_abs * dt + d_now;
  const float d_increase = (a_abs + step) * dt + stopping_distance(a_abs + step, step, dt);
  return {sign * d_now, sign * d_maintain, sign * d_increase};
}

/**
 * @brief One jerk-limited acceleration update (discrete optimal control).
 * @param a Acceleration in / out: the value applied on the previous step, and
 *          the value to apply now.
 * @param dist Remaining velocity change to the target (target - v).
 * @param step Largest acceleration change per step (jerk * dt).
 * @param a_max Acceleration limit of the profile in force.
 * @param dt Step length.
 * @note The change of `a` is limited to `step` in every case, including when
 *       the incoming `a` is beyond `a_max` (the previous step belonged to a
 *       profile with a higher limit, e.g. a firmer stopping profile): it is
 *       then pulled back into range at the jerk limit, not clamped at once.
 */
inline void update_accel(float &a, float dist, float step, float a_max, float dt) {
  const auto [d_now, d_maintain, d_increase] = stopping_thresholds(a, step, dt);
  const float a_prev = a;

  if (dist > 0.0f) {
    if (dist > d_increase)
      a += step;
    else if (dist <= d_maintain) {
      const float ideal_a = dist / dt; // positive
      if (ideal_a <= a && ideal_a >= a - step && ideal_a < step)
        a = ideal_a;
      else
        a -= step;
    }
    // else: maintain the current a
  } else if (dist < 0.0f) {
    if (dist < d_increase)
      a -= step;
    else if (dist >= d_maintain) {
      const float ideal_a = dist / dt; // negative
      if (ideal_a >= a && ideal_a <= a + step && ideal_a > -step)
        a = ideal_a;
      else
        a += step;
    }
    // else: maintain the current a
  } else {
    a = 0.0f;
  }

  // Safety: limit the change to the jerk limit in case the decision block over-stepped.
  const float delta_a = a - a_prev;
  if (std::abs(delta_a) > step)
    a = a_prev + std::copysign(step, delta_a);

  // Clamp to the profile's acceleration limit; an acceleration inherited from
  // a profile with a higher limit comes back into range one jerk step at a time.
  const float lo = std::min(-a_max, a_prev + step);
  const float hi = std::max(a_max, a_prev - step);
  a = std::clamp(a, lo, hi);
}

/// Trapezoidal rate limiter: advance `current` toward `target` by at most max_accel * dt.
inline float rate_limit(float current, float target, float max_accel, float dt) {
  const float max_delta = max_accel * dt;
  return current + std::clamp(target - current, -max_delta, max_delta);
}

/// The motion state of one axis between steps.
struct AxisState {
  float v = 0.0f; ///< velocity
  float a = 0.0f; ///< acceleration applied on the last step
};

/**
 * @brief Advance one axis by one planning step toward `target`.
 *
 * With `jerk` > 0 the acceleration is planned by update_accel() (S-curve);
 * with `jerk` == 0 the velocity is rate-limited by `a_max` directly
 * (trapezoidal). Either way `state.a` ends up as the acceleration that was
 * actually applied -- (v_new - v_old) / dt, after the velocity clamp -- so
 * that a later jerk-limited step resumes from the real value: after a
 * trapezoidal stop has completed it is 0, not the last S-curve acceleration,
 * and if the stop is interrupted it is the deceleration in force.
 *
 * @param state Axis state in / out.
 * @param target Target velocity.
 * @param v_min Lowest velocity allowed (<= 0; e.g. minus the reverse limit).
 * @param v_max Highest velocity allowed (>= 0); the result is clamped to [v_min, v_max].
 * @param a_max Acceleration limit.
 * @param jerk Jerk limit; 0 selects the trapezoidal profile.
 * @param dt Step length (> 0).
 */
inline void step_axis(AxisState &state, float target, float v_min, float v_max, float a_max,
                      float jerk, float dt) {
  const float v_prev = state.v;
  if (jerk > 0.0f) {
    update_accel(state.a, target - state.v, jerk * dt, a_max, dt);
    state.v = std::clamp(state.v + state.a * dt, v_min, v_max);
  } else {
    state.v = std::clamp(rate_limit(state.v, target, a_max, dt), v_min, v_max);
  }
  state.a = (state.v - v_prev) / dt;
}

} // namespace trajectory_planner
} // namespace detail
} // namespace espp

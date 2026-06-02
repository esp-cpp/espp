#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

#include "base_component.hpp"

/**
 * @file adrc.hpp
 * @brief Active disturbance rejection control utilities and controllers.
 *
 * This header provides a Han tracking differentiator plus first-order and
 * second-order ADRC controllers in both linear and Han-style nonlinear forms.
 *
 * ADRC combines three ideas:
 * - a reference shaper / tracking differentiator,
 * - an extended state observer (ESO), and
 * - a feedback law that cancels the estimated disturbance.
 *
 * In practice, the observer lumps together plant uncertainty, friction, load
 * torque changes, unmodeled dynamics, and external disturbances into an extra
 * state (`z2` or `z3`). The controller then subtracts that estimate from the
 * commanded control effort.
 *
 * Typical robotics usage:
 * - use a first-order ADRC around speed, current, or other approximately
 *   first-order actuator dynamics;
 * - use a second-order ADRC around position, heading, or other loops where the
 *   commanded input primarily affects acceleration;
 * - use the Han nonlinear variants when you want gentler small-signal behavior
 *   and stronger response on large errors than the linear forms provide.
 *
 * @note `dt` is expressed in seconds and should be reasonably consistent from
 * update to update.
 * @note `b0` is the estimated plant input gain. Its sign should be correct and
 * its magnitude should be in the right ballpark, but ADRC does not require a
 * perfect plant model.
 */
namespace espp {
namespace detail {
inline constexpr float adrc_epsilon = 1e-6f;

inline float safe_nonzero(float value) {
  if (std::fabs(value) >= adrc_epsilon) {
    return value;
  }
  return value < 0.0f ? -adrc_epsilon : adrc_epsilon;
}

inline float sign(float value) { return (0.0f < value) - (value < 0.0f); }

inline float fal(float error, float alpha, float delta) {
  auto safe_delta = std::max(delta, adrc_epsilon);
  auto abs_error = std::fabs(error);
  if (abs_error <= safe_delta) {
    return error / std::pow(safe_delta, 1.0f - alpha);
  }
  return std::copysign(std::pow(abs_error, alpha), error);
}

inline float fsg(float value, float delta) {
  return 0.5f * (sign(value + delta) - sign(value - delta));
}

inline float fst(float x1, float x2, float r, float h) {
  auto safe_h = std::max(h, adrc_epsilon);
  auto safe_r = std::max(r, adrc_epsilon);
  auto d = safe_r * safe_h * safe_h;
  auto a0 = safe_h * x2;
  auto y = x1 + a0;
  auto a1 = std::sqrt(d * (d + 8.0f * std::fabs(y)));
  auto a2 = a0 + sign(y) * (a1 - d) * 0.5f;
  auto sy = fsg(y, d);
  auto a = (a0 + y - a2) * sy + a2;
  auto sa = fsg(a, d);
  return -safe_r * ((a / d - sign(a)) * sa + sign(a));
}
} // namespace detail

/**
 * @brief Han tracking differentiator for smoothing a reference trajectory and
 *        estimating its rate.
 *
 * The tracking differentiator (TD) is commonly used at the front of an ADRC
 * loop to turn a stepped reference into a smooth internal command. That reduces
 * jerk, limits observer shock, and provides an internally consistent reference
 * derivative for second-order loops.
 *
 * For robotics motor control, the TD is useful when a planner or user interface
 * produces abrupt setpoint changes but the plant should accelerate smoothly. In
 * a position loop, for example, the TD can convert a position step into a
 * smoother position and velocity command pair.
 *
 * @par Tuning
 * `tracking_bandwidth` controls how aggressively the TD chases the target.
 * Larger values follow the reference more tightly but can reintroduce abrupt
 * motion. `filter_factor` scales the internal `h0` term and can be increased to
 * further soften the commanded trajectory.
 *
 */
class HanTrackingDifferentiator : public BaseComponent {
public:
  /// Configuration for the Han tracking differentiator.
  struct Config {
    float tracking_bandwidth;  ///< Tracking aggressiveness parameter, commonly noted `r`.
    float filter_factor{5.0f}; ///< Multiplier applied to `dt` for the Han `h0` term.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// State of the tracking differentiator.
  struct State {
    float position{0.0f}; ///< Smoothed reference value.
    float rate{0.0f};     ///< Estimated first derivative of the smoothed reference.
  };

  /// Construct the tracking differentiator.
  /// @param config Tracking differentiator configuration.
  explicit HanTrackingDifferentiator(const Config &config)
      : BaseComponent("HanTrackingDifferentiator", config.log_level)
      , config_(config) {}

  /// Change the tracking differentiator configuration.
  /// @param config New configuration.
  /// @param reset_state If true, reset the differentiator state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    if (reset_state) {
      clear();
    }
  }

  /// Reset the differentiator state.
  /// @param value Initial position value after reset.
  void clear(float value = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.position = value;
    state_.rate = 0.0f;
  }

  /// Advance the tracking differentiator.
  /// @param target Target reference value to follow.
  /// @param dt Timestep in seconds.
  /// @return Updated tracking differentiator state.
  State update(float target, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_;
    }
    auto h0 = std::max(config_.filter_factor * dt, dt);
    auto accel = detail::fst(state_.position - target, state_.rate, config_.tracking_bandwidth, h0);
    state_.position += dt * state_.rate;
    state_.rate += dt * accel;
    return state_;
  }

  /// Advance the tracking differentiator.
  /// @param target Target reference value to follow.
  /// @param dt Timestep in seconds.
  /// @return Updated tracking differentiator state.
  State operator()(float target, float dt) { return update(target, dt); }

  /// Get the most recently computed state.
  /// @return Current differentiator state.
  const State &get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active differentiator configuration.
  /// @return Current configuration.
  const Config &get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

protected:
  Config config_;
  State state_;
  mutable std::recursive_mutex mutex_;
};

/**
 * @brief Linear first-order active disturbance rejection controller.
 *
 * This controller targets plants that can be approximated as a first-order
 * system in the controlled variable, such as a velocity loop whose command acts
 * roughly like torque or acceleration after inner current regulation.
 *
 * The linear ESO estimates:
 * - `z1`: the controlled output, and
 * - `z2`: the lumped disturbance and modeling error.
 *
 * The resulting control law is simple to tune with bandwidth parameters and is
 * often a good first ADRC choice when migrating from PI or PID speed control.
 *
 * @par Typical robotics use
 * Use this class for wheel speed control, conveyor speed control, or other
 * actuator loops where the measured state is dominated by a single pole. In a
 * cascaded servo, this is commonly the outer speed loop above a faster current
 * controller.
 *
 * @par Tuning
 * Start with a conservative `controller_bandwidth`, then choose
 * `observer_bandwidth` several times higher so the ESO settles faster than the
 * controller. `b0` should match the sign of the actuator path and roughly scale
 * command to output-rate change. If the controller saturates immediately,
 * reduce the bandwidth or improve the `b0` estimate before increasing gains.
 *
 * \section adrc_ex1 Linear First-Order ADRC Example
 * \snippet adrc_example.cpp adrc linear first order example
 */
class LinearAdrcFirstOrder : public BaseComponent {
public:
  /// Configuration for the first-order linear ADRC controller.
  struct Config {
    float b0;                   ///< Estimated control gain of the plant.
    float controller_bandwidth; ///< Closed-loop bandwidth used for the state error feedback gain.
    float observer_bandwidth;   ///< Observer bandwidth used for the linear extended state observer.
    float output_min;           ///< Minimum output command.
    float output_max;           ///< Maximum output command.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// Observer and controller state.
  struct State {
    float measurement{0.0f}; ///< Most recent measured output.
    float reference{0.0f};   ///< Most recent reference input.
    float z1{0.0f};          ///< Estimated plant output state.
    float z2{0.0f};          ///< Estimated total disturbance state.
    float output{0.0f};      ///< Most recent controller output.
  };

  /// Construct the first-order linear ADRC controller.
  /// @param config Controller configuration.
  explicit LinearAdrcFirstOrder(const Config &config)
      : BaseComponent("LinearAdrcFirstOrder", config.log_level)
      , config_(config) {}

  /// Change the controller configuration.
  /// @param config New controller configuration.
  /// @param reset_state If true, reset the observer and control state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    if (reset_state) {
      clear();
    }
  }

  /// Reset the observer and controller state.
  /// @param measurement Initial measured output for the observer state.
  void clear(float measurement = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.measurement = measurement;
    state_.reference = measurement;
    state_.z1 = measurement;
    state_.z2 = 0.0f;
    state_.output = 0.0f;
  }

  /// Update the controller.
  /// @param reference Desired output reference.
  /// @param measurement Measured output.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float measurement, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_.output;
    }

    auto wo = std::max(config_.observer_bandwidth, detail::adrc_epsilon);
    auto beta1 = 2.0f * wo;
    auto beta2 = wo * wo;
    auto b0 = detail::safe_nonzero(config_.b0);
    auto error = state_.z1 - measurement;

    state_.measurement = measurement;
    state_.reference = reference;
    state_.z1 += dt * (state_.z2 - beta1 * error + b0 * state_.output);
    state_.z2 += dt * (-beta2 * error);

    auto k1 = std::max(config_.controller_bandwidth, detail::adrc_epsilon);
    auto u0 = k1 * (reference - state_.z1);
    state_.output = std::clamp((u0 - state_.z2) / b0, config_.output_min, config_.output_max);
    return state_.output;
  }

  /// Update the controller.
  /// @param reference Desired output reference.
  /// @param measurement Measured output.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float operator()(float reference, float measurement, float dt) {
    return update(reference, measurement, dt);
  }

  /// Get the current controller state.
  /// @return Current state.
  const State &get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active configuration.
  /// @return Current configuration.
  const Config &get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

protected:
  Config config_;
  State state_;
  mutable std::recursive_mutex mutex_;
};

/**
 * @brief Linear second-order active disturbance rejection controller.
 *
 * This controller targets plants whose commanded input primarily affects the
 * second derivative of the controlled variable. Typical examples include motor
 * position loops, gimbal angle loops, and heading loops where the command acts
 * like torque or angular acceleration.
 *
 * The linear ESO estimates:
 * - `z1`: position,
 * - `z2`: rate, and
 * - `z3`: the lumped disturbance and unmodeled dynamics.
 *
 * @par Typical robotics use
 * Use this class when your feedback variable is position but your actuator
 * fundamentally produces acceleration. In a mobile robot or arm joint, this is
 * usually the position controller that sits above a faster velocity or torque
 * loop.
 *
 * @par Tuning
 * `controller_bandwidth` shapes the closed-loop stiffness and damping. The
 * observer is usually tuned faster than the controller so `z3` converges before
 * it dominates the motion. If a trajectory generator already provides a
 * velocity feedforward term, pass it through the four-argument `update()`
 * overload as `reference_rate`.
 *
 * \section adrc_ex2 Linear Second-Order ADRC Example
 * \snippet adrc_example.cpp adrc linear second order example
 */
class LinearAdrcSecondOrder : public BaseComponent {
public:
  /// Configuration for the second-order linear ADRC controller.
  struct Config {
    float b0;                   ///< Estimated control gain of the plant.
    float controller_bandwidth; ///< Closed-loop bandwidth used to derive the feedback gains.
    float observer_bandwidth;   ///< Observer bandwidth used for the linear extended state observer.
    float output_min;           ///< Minimum output command.
    float output_max;           ///< Maximum output command.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// Observer and controller state.
  struct State {
    float measurement{0.0f};    ///< Most recent measured position.
    float reference{0.0f};      ///< Most recent reference position.
    float reference_rate{0.0f}; ///< Most recent reference rate.
    float z1{0.0f};             ///< Estimated plant position state.
    float z2{0.0f};             ///< Estimated plant rate state.
    float z3{0.0f};             ///< Estimated total disturbance state.
    float output{0.0f};         ///< Most recent controller output.
  };

  /// Construct the second-order linear ADRC controller.
  /// @param config Controller configuration.
  explicit LinearAdrcSecondOrder(const Config &config)
      : BaseComponent("LinearAdrcSecondOrder", config.log_level)
      , config_(config) {}

  /// Change the controller configuration.
  /// @param config New controller configuration.
  /// @param reset_state If true, reset the observer and control state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    if (reset_state) {
      clear();
    }
  }

  /// Reset the observer and controller state.
  /// @param measurement Initial measured position for the observer state.
  /// @param measurement_rate Initial measured rate estimate for the observer state.
  void clear(float measurement = 0.0f, float measurement_rate = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.measurement = measurement;
    state_.reference = measurement;
    state_.reference_rate = 0.0f;
    state_.z1 = measurement;
    state_.z2 = measurement_rate;
    state_.z3 = 0.0f;
    state_.output = 0.0f;
  }

  /// Update the controller using an explicit reference rate.
  /// @param reference Desired position reference.
  /// @param reference_rate Desired reference rate.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float reference_rate, float measurement, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_.output;
    }

    auto wo = std::max(config_.observer_bandwidth, detail::adrc_epsilon);
    auto beta1 = 3.0f * wo;
    auto beta2 = 3.0f * wo * wo;
    auto beta3 = wo * wo * wo;
    auto b0 = detail::safe_nonzero(config_.b0);
    auto error = state_.z1 - measurement;

    state_.measurement = measurement;
    state_.reference = reference;
    state_.reference_rate = reference_rate;
    state_.z1 += dt * (state_.z2 - beta1 * error);
    state_.z2 += dt * (state_.z3 - beta2 * error + b0 * state_.output);
    state_.z3 += dt * (-beta3 * error);

    auto wc = std::max(config_.controller_bandwidth, detail::adrc_epsilon);
    auto kp = wc * wc;
    auto kd = 2.0f * wc;
    auto u0 = kp * (reference - state_.z1) + kd * (reference_rate - state_.z2);
    state_.output = std::clamp((u0 - state_.z3) / b0, config_.output_min, config_.output_max);
    return state_.output;
  }

  /// Update the controller assuming a zero reference-rate command.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float measurement, float dt) {
    return update(reference, 0.0f, measurement, dt);
  }

  /// Update the controller assuming a zero reference-rate command.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float operator()(float reference, float measurement, float dt) {
    return update(reference, measurement, dt);
  }

  /// Get the current controller state.
  /// @return Current state.
  const State &get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active configuration.
  /// @return Current configuration.
  const Config &get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

protected:
  Config config_;
  State state_;
  mutable std::recursive_mutex mutex_;
};

/**
 * @brief Han-style nonlinear first-order active disturbance rejection
 *        controller.
 *
 * This variant replaces the linear error terms with Han's nonlinear `fal()`
 * functions and can optionally prefilter the reference with a tracking
 * differentiator. The nonlinear feedback is often attractive when you want high
 * authority on large errors without making the loop excessively sharp around
 * the operating point.
 *
 * @par Typical robotics use
 * Use this class for speed loops that must reject large step disturbances but
 * remain well-behaved near zero speed, for example wheel or propeller speed
 * control with friction, stiction, or battery-voltage variation.
 *
 * @par Tuning
 * `controller_gain` is the main feedback gain. `observer_bandwidth` sets the
 * speed of the nonlinear ESO. `controller_alpha`, `observer_alpha`, and
 * `fal_delta` determine how strongly the loop transitions between linear
 * small-error behavior and nonlinear large-error behavior. Increase
 * `fal_delta` if the loop is too twitchy around the setpoint, especially with
 * noisy sensors.
 *
 * \section adrc_ex3 Han First-Order ADRC Example
 * \snippet adrc_example.cpp adrc han first order example
 */
class HanAdrcFirstOrder : public BaseComponent {
public:
  /// Configuration for the first-order nonlinear ADRC controller.
  struct Config {
    float b0;                     ///< Estimated control gain of the plant.
    float controller_gain;        ///< Nonlinear state error feedback gain.
    float observer_bandwidth;     ///< Observer bandwidth used to derive nonlinear ESO gains.
    float observer_alpha{0.5f};   ///< `fal()` exponent used by the observer disturbance state.
    float controller_alpha{0.8f}; ///< `fal()` exponent used by the nonlinear feedback term.
    float fal_delta{0.01f};       ///< Small-signal linear region width used by `fal()`.
    bool use_tracking_differentiator{true}; ///< Enable reference smoothing through the Han TD.
    HanTrackingDifferentiator::Config tracking_config{
        .tracking_bandwidth = 40.0f,
        .filter_factor = 5.0f}; ///< Tracking differentiator configuration.
    float output_min;           ///< Minimum output command.
    float output_max;           ///< Maximum output command.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// Observer and controller state.
  struct State {
    float measurement{0.0f};  ///< Most recent measured output.
    float reference{0.0f};    ///< Most recent reference input.
    float td_reference{0.0f}; ///< Smoothed reference from the tracking differentiator.
    float z1{0.0f};           ///< Estimated plant output state.
    float z2{0.0f};           ///< Estimated total disturbance state.
    float output{0.0f};       ///< Most recent controller output.
  };

  /// Construct the first-order nonlinear ADRC controller.
  /// @param config Controller configuration.
  explicit HanAdrcFirstOrder(const Config &config)
      : BaseComponent("HanAdrcFirstOrder", config.log_level)
      , config_(config)
      , tracking_differentiator_(config.tracking_config) {}

  /// Change the controller configuration.
  /// @param config New controller configuration.
  /// @param reset_state If true, reset the observer and control state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    tracking_differentiator_.set_config(config.tracking_config, reset_state);
    if (reset_state) {
      clear();
    }
  }

  /// Reset the observer, controller, and tracking differentiator state.
  /// @param measurement Initial measured output for the observer state.
  void clear(float measurement = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.measurement = measurement;
    state_.reference = measurement;
    state_.td_reference = measurement;
    state_.z1 = measurement;
    state_.z2 = 0.0f;
    state_.output = 0.0f;
    tracking_differentiator_.clear(measurement);
  }

  /// Update the controller.
  /// @param reference Desired output reference.
  /// @param measurement Measured output.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float measurement, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_.output;
    }

    auto td_state = config_.use_tracking_differentiator
                        ? tracking_differentiator_.update(reference, dt)
                        : HanTrackingDifferentiator::State{.position = reference, .rate = 0.0f};
    auto wo = std::max(config_.observer_bandwidth, detail::adrc_epsilon);
    auto beta1 = 2.0f * wo;
    auto beta2 = wo * wo;
    auto b0 = detail::safe_nonzero(config_.b0);
    auto error = state_.z1 - measurement;

    state_.measurement = measurement;
    state_.reference = reference;
    state_.td_reference = td_state.position;
    state_.z1 += dt * (state_.z2 - beta1 * error + b0 * state_.output);
    state_.z2 += dt * (-beta2 * detail::fal(error, config_.observer_alpha, config_.fal_delta));

    auto feedback =
        config_.controller_gain *
        detail::fal(td_state.position - state_.z1, config_.controller_alpha, config_.fal_delta);
    state_.output = std::clamp((feedback - state_.z2) / b0, config_.output_min, config_.output_max);
    return state_.output;
  }

  /// Update the controller.
  /// @param reference Desired output reference.
  /// @param measurement Measured output.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float operator()(float reference, float measurement, float dt) {
    return update(reference, measurement, dt);
  }

  /// Get the current controller state.
  /// @return Current state.
  const State &get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active configuration.
  /// @return Current configuration.
  const Config &get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

  /// Get the current tracking differentiator state.
  /// @return Tracking differentiator state.
  HanTrackingDifferentiator::State get_tracking_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return tracking_differentiator_.get_state();
  }

protected:
  Config config_;
  State state_;
  HanTrackingDifferentiator tracking_differentiator_;
  mutable std::recursive_mutex mutex_;
};

/**
 * @brief Han-style nonlinear second-order active disturbance rejection
 *        controller.
 *
 * This controller combines a second-order ESO with nonlinear state error
 * feedback and an optional Han tracking differentiator. It is the most flexible
 * controller in this component and is useful when a position-like loop must
 * reject significant disturbances while still handling large setpoint changes
 * gracefully.
 *
 * @par Typical robotics use
 * Use this class for position control of motor-driven joints, pan/tilt systems,
 * steering axes, or mobile-robot heading loops when load variation and
 * unmodeled friction are too large for simple PD or PID tuning to stay robust.
 *
 * @par Tuning
 * `position_gain` and `rate_gain` play roles similar to proportional and
 * derivative action in the nonlinear feedback law. `observer_bandwidth` should
 * usually be faster than the desired closed-loop motion, but if encoder noise
 * is significant you may need to back it off and rely more on the tracking
 * differentiator to keep the reference smooth. `controller_alpha1` and
 * `controller_alpha2` change how strongly large position and rate errors are
 * amplified relative to small errors.
 *
 * @note When `use_tracking_differentiator` is true, the explicit
 * `reference_rate` argument passed to the four-argument `update()` overload is
 * ignored. In that mode the controller uses the tracking differentiator's
 * internally generated rate estimate instead.
 *
 * \section adrc_ex4 Han Second-Order ADRC Example
 * \snippet adrc_example.cpp adrc han second order example
 */
class HanAdrcSecondOrder : public BaseComponent {
public:
  /// Configuration for the second-order nonlinear ADRC controller.
  struct Config {
    float b0;                      ///< Estimated control gain of the plant.
    float position_gain;           ///< Nonlinear feedback gain for position error.
    float rate_gain;               ///< Nonlinear feedback gain for rate error.
    float observer_bandwidth;      ///< Observer bandwidth used to derive nonlinear ESO gains.
    float observer_alpha1{0.5f};   ///< `fal()` exponent used for the ESO middle state.
    float observer_alpha2{0.25f};  ///< `fal()` exponent used for the ESO disturbance state.
    float controller_alpha1{0.8f}; ///< `fal()` exponent used for the position error.
    float controller_alpha2{1.5f}; ///< `fal()` exponent used for the rate error.
    float fal_delta{0.01f};        ///< Small-signal linear region width used by `fal()`.
    bool use_tracking_differentiator{true}; ///< Enable reference smoothing through the Han TD.
    HanTrackingDifferentiator::Config tracking_config{
        .tracking_bandwidth = 60.0f,
        .filter_factor = 5.0f}; ///< Tracking differentiator configuration.
    float output_min;           ///< Minimum output command.
    float output_max;           ///< Maximum output command.
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; ///< Verbosity for the internal logger.
  };

  /// Observer and controller state.
  struct State {
    float measurement{0.0f};       ///< Most recent measured position.
    float reference{0.0f};         ///< Most recent reference position.
    float td_reference{0.0f};      ///< Smoothed reference position.
    float td_reference_rate{0.0f}; ///< Smoothed reference rate.
    float z1{0.0f};                ///< Estimated plant position state.
    float z2{0.0f};                ///< Estimated plant rate state.
    float z3{0.0f};                ///< Estimated total disturbance state.
    float output{0.0f};            ///< Most recent controller output.
  };

  /// Construct the second-order nonlinear ADRC controller.
  /// @param config Controller configuration.
  explicit HanAdrcSecondOrder(const Config &config)
      : BaseComponent("HanAdrcSecondOrder", config.log_level)
      , config_(config)
      , tracking_differentiator_(config.tracking_config) {}

  /// Change the controller configuration.
  /// @param config New controller configuration.
  /// @param reset_state If true, reset the observer and control state after applying the config.
  void set_config(const Config &config, bool reset_state = true) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    config_ = config;
    tracking_differentiator_.set_config(config.tracking_config, reset_state);
    if (reset_state) {
      clear();
    }
  }

  /// Reset the observer, controller, and tracking differentiator state.
  /// @param measurement Initial measured position for the observer state.
  /// @param measurement_rate Initial measured rate estimate for the observer state.
  void clear(float measurement = 0.0f, float measurement_rate = 0.0f) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    state_.measurement = measurement;
    state_.reference = measurement;
    state_.td_reference = measurement;
    state_.td_reference_rate = 0.0f;
    state_.z1 = measurement;
    state_.z2 = measurement_rate;
    state_.z3 = 0.0f;
    state_.output = 0.0f;
    tracking_differentiator_.clear(measurement);
  }

  /// Update the controller using an externally supplied reference rate when the
  /// tracking differentiator is disabled.
  /// @note If `use_tracking_differentiator` is true, `reference_rate` is
  /// ignored and the tracking differentiator's internally estimated rate is
  /// used instead.
  /// @param reference Desired position reference.
  /// @param reference_rate Desired reference rate.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float reference_rate, float measurement, float dt) {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    if (dt <= 0.0f) {
      return state_.output;
    }

    auto td_state = config_.use_tracking_differentiator
                        ? tracking_differentiator_.update(reference, dt)
                        : HanTrackingDifferentiator::State{
                              .position = reference,
                              .rate = reference_rate,
                          };
    auto wo = std::max(config_.observer_bandwidth, detail::adrc_epsilon);
    auto beta1 = 3.0f * wo;
    auto beta2 = 3.0f * wo * wo;
    auto beta3 = wo * wo * wo;
    auto b0 = detail::safe_nonzero(config_.b0);
    auto error = state_.z1 - measurement;

    state_.measurement = measurement;
    state_.reference = reference;
    state_.td_reference = td_state.position;
    state_.td_reference_rate = td_state.rate;
    state_.z1 += dt * (state_.z2 - beta1 * error);
    state_.z2 +=
        dt * (state_.z3 - beta2 * detail::fal(error, config_.observer_alpha1, config_.fal_delta) +
              b0 * state_.output);
    state_.z3 += dt * (-beta3 * detail::fal(error, config_.observer_alpha2, config_.fal_delta));

    auto e1 = td_state.position - state_.z1;
    auto e2 = td_state.rate - state_.z2;
    auto u0 =
        config_.position_gain * detail::fal(e1, config_.controller_alpha1, config_.fal_delta) +
        config_.rate_gain * detail::fal(e2, config_.controller_alpha2, config_.fal_delta);
    state_.output = std::clamp((u0 - state_.z3) / b0, config_.output_min, config_.output_max);
    return state_.output;
  }

  /// Update the controller assuming a zero reference-rate command when the
  /// tracking differentiator is disabled.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float update(float reference, float measurement, float dt) {
    return update(reference, 0.0f, measurement, dt);
  }

  /// Update the controller assuming a zero reference-rate command when the
  /// tracking differentiator is disabled.
  /// @param reference Desired position reference.
  /// @param measurement Measured position.
  /// @param dt Timestep in seconds.
  /// @return Control output.
  float operator()(float reference, float measurement, float dt) {
    return update(reference, measurement, dt);
  }

  /// Get the current controller state.
  /// @return Current state.
  const State &get_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return state_;
  }

  /// Get the active configuration.
  /// @return Current configuration.
  const Config &get_config() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return config_;
  }

  /// Get the current tracking differentiator state.
  /// @return Tracking differentiator state.
  HanTrackingDifferentiator::State get_tracking_state() const {
    std::lock_guard<std::recursive_mutex> lk(mutex_);
    return tracking_differentiator_.get_state();
  }

protected:
  Config config_;
  State state_;
  HanTrackingDifferentiator tracking_differentiator_;
  mutable std::recursive_mutex mutex_;
};
} // namespace espp

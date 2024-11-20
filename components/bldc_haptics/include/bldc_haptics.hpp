#pragma once

#include <algorithm>
#include <deque>
#include <vector>

#include "base_component.hpp"
#include "bldc_motor.hpp"
#include "detent_config.hpp"
#include "haptic_config.hpp"
#include "task.hpp"

namespace espp {
/// @brief Concept for a motor that can be used for haptics
template <class FOO>
concept MotorConcept = requires {
  static_cast<void (FOO::*)(void)>(&FOO::enable);  ///< Enable the motor
  static_cast<void (FOO::*)(void)>(&FOO::disable); ///< Disable the motor
  static_cast<void (FOO::*)(float)>(
      &FOO::move); ///< Move the motor to a new target (position, velocity, or torque depending on
                   ///< the motor control type)
  static_cast<void (FOO::*)(detail::MotionControlType)>(
      &FOO::set_motion_control_type);                            ///< Set the motion control type
  static_cast<void (FOO::*)(void)>(&FOO::loop_foc);              ///< Run the FOC loop
  static_cast<float (FOO::*)(void)>(&FOO::get_shaft_angle);      ///< Get the shaft angle
  static_cast<float (FOO::*)(void)>(&FOO::get_shaft_velocity);   ///< Get the shaft velocity
  static_cast<float (FOO::*)(void)>(&FOO::get_electrical_angle); ///< Get the electrical angle
};

/// @brief Class which creates haptic feedback for the user by vibrating the
/// motor This class is based on the work at
/// https://github.com/scottbez1/smartknob to use a small BLDC gimbal motor as a
/// haptic feedback device. It does so by varying the control type, setpoints, and
/// gains of the motor to create a vibration. The motor is driven using the ESP32's MCPWM
/// peripheral. The motor is driven in a closed loop using the encoder feedback.
/// @note The motor is configured to be driven in an open-loop mode, so that the PID contained
/// in the motor controller does not interfere with the haptic feedback.
///
/// The haptics provided by this class enable configuration of:
/// - Positions (with non-magnetic detents) - evenly spaced across the allowed range of motion
/// - Number of magnetic detents - where the
/// - Width of the detents
/// - Strength of the detents
/// - Snap point (position at which the motor will snap to the next detent / position)
/// - Bounds on the rotation of the motor - can be unbounded, bounded within a
///   single revolution, or bounded within multiple revolutions
///
/// The haptics provided by this class provide the following functionality:
/// - Positions: Evenly spaced positions across the allowed range of motion
///   (specified by the min and max position) will have a detent.
/// - Detents: Manually specified detents will be placed at the specified
///   positions. The detents will have a specified width and strength.
/// - End Stops: The motor will vibrate when it is at the min or max position.
///   This is useful for providing feedback when the motor is at the end of its
///   range of motion. The end stops are configured by specifying the strength
///   of the end stops.
/// - Snap point: The snap point is the position at which the motor will snap
///   to the next detent / position. This is useful for providing feedback when
///   the motor is at a certain position. The snap point is configured by
///   specifying the snap point (percentage of the way through the detent) and
///   the snap point bias (percentage of the way through the detent to bias the
///   snap point).
///
/// Some example configurations are provided as static constexpr in
/// espp::detail. They are:
/// - UNBOUNDED_NO_DETENTS: No detents, no end stops, no snap point, no bounds
/// - BOUNDED_NO_DETENTS: No detents, no end stops, no snap point, bounded
///   within a single revolution
/// - MULTI_REV_NO_DETENTS: No detents, no end stops, no snap point, bounded
///   within multiple revolutions
/// - COARSE_VALUES_STRONG_DETENTS: detents, end stops, snap point, bounded
///   within a single revolution
/// - FINE_VALUES_NO_DETENTS: No detents, end stops, snap point, bounded
/// - FINE_VALUES_WITH_DETENTS: detents, end stops, snap point, bounded
/// - RETURN_TO_CENTER_WITH_DETENTS: 3 detents, end stops, snap point, bounded
///   within a single revolution
/// - RETURN_TO_CENTER_WITH_DETENTS_AND_MULTIPLE_REVOLUTIONS: 3 detents, end
///   stops, snap point, bounded within multiple revolutions
///
/// Some haptic behaviors that can be implemented with this library are:
/// - Unbounded with no detents
/// - Bounded with no detents
/// - Multiple revolutions
/// - On/off with strong detent
/// - Return to center without detents
/// - Return to center with detents
/// - Fine values with no detents
/// - Fine values with detents
/// - Coarse values with strong detents
/// - Coarse values with weak detents
///
/// \section bldc_haptics_ex1 Example 1: Basic usage
/// \snippet bldc_haptics_example.cpp bldc_haptics_example_1
/// \section bldc_haptics_ex2 Example 2: Playing a haptic click / buzz
/// \snippet bldc_haptics_example.cpp bldc_haptics_example_2
template <MotorConcept M> class BldcHaptics : public BaseComponent {
public:
  /// @brief Configuration for the haptic motor
  struct Config {
    std::reference_wrapper<M> motor; ///< Pointer to the motor to use for haptics
    float kp_factor{2}; ///< Factor to multiply the detent strength by to get kp (default 2). Used
                        ///< for both detents and end stops. \note Depending on the motor, this may
                        ///< need to be adjusted to get the desired behavior.
    float kd_factor_min{0.01}; ///< Min Factor to multiply the detent strength by to get kd (default
                               ///< 0.01). \note Depending on the motor, this may need to be
                               ///< adjusted to get the desired behavior.
    float kd_factor_max{0.04}; ///< Max Factor to multiply the detent strength by to get kd (default
                               ///< 0.04). \note Depending on the motor, this may need to be
                               ///< adjusted to get the desired behavior.
    Logger::Verbosity log_level; ///< Log level to use for the haptics
  };

  /// @brief Constructor for the haptic motor
  /// @param config Configuration for the haptic motor
  explicit BldcHaptics(const Config &config)
      : BaseComponent("BldcHaptics", config.log_level)
      , detent_pid_({.kp = 0,             // will be set later (motor_task)
                     .ki = 0,             // not configurable for now
                     .kd = 0,             // will be set later (update_detent_config)
                     .integrator_min = 0, // not configurable for now
                     .integrator_max = 0, // not configurable for now
                     .output_min = -1,    // go ahead and set some bounds (operates on current)
                     .output_max = 1})    // go ahead and set some bounds (operates on current)
      , kp_factor_(config.kp_factor)
      , kd_factor_min_(config.kd_factor_min)
      , kd_factor_max_(config.kd_factor_max)
      , motor_(config.motor) {
    logger_.set_rate_limit(std::chrono::milliseconds(100));
    logger_.info("Initializing haptic motor\n"
                 "\tkp_factor: {}\n"
                 "\tkd_factor_min: {}\n"
                 "\tkd_factor_max: {}",
                 kp_factor_, kd_factor_min_, kd_factor_max_);
    // set the motion control type to torque
    motor_.get().set_motion_control_type(detail::MotionControlType::TORQUE);
    // create the motor task
    motor_task_ = Task::make_unique(
        {.name = "haptic_motor",
         .callback = std::bind(&BldcHaptics::motor_task, this, std::placeholders::_1,
                               std::placeholders::_2, std::placeholders::_3),
         .stack_size_bytes = 1024 * 6,
         .log_level = Logger::Verbosity::WARN});
  }

  /// @brief Destructor for the haptic motor
  /// @note This will stop the motor if it is running
  ~BldcHaptics() {
    // stop the motor if it is running
    stop();
  }

  /// @brief Check if the haptic motor is running
  /// @return True if the haptic motor is running, false otherwise
  bool is_running() const { return motor_task_->is_running(); }

  /// @brief Start the haptic motor
  void start() {
    // enable the motor
    {
      std::unique_lock<std::mutex> lk(motor_mutex_);
      motor_.get().enable();
    }
    if (is_running()) {
      return;
    }
    motor_task_->start();
  }

  /// @brief Stop the haptic motor
  void stop() {
    // disable the motor
    {
      std::unique_lock<std::mutex> lk(motor_mutex_);
      motor_.get().disable();
    }
    if (!is_running()) {
      return;
    }
    motor_task_->stop();
  }

  /// @brief Get the current position of the haptic motor
  /// @return Current position of the haptic motor
  float get_position() const { return current_position_; }

  /// @brief Configure the detents for the haptic motor
  void update_detent_config(const detail::DetentConfig &config) {
    std::unique_lock<std::mutex> lk(detent_mutex_);
    // update the detent center
    current_detent_center_ = motor_.get().get_shaft_angle();

    if (current_position_ < config.min_position) {
      // if the current position is less than the min position, set the current
      // position to the min position
      current_position_ = config.min_position;
    } else if (current_position_ > config.max_position) {
      // if the current position is greater than the max position, set the
      // current position to the max position
      current_position_ = config.max_position;
    }

    // update the detent config
    detent_config_ = config;

    // Update derivative factor of torque controller based on detent width. If
    // the D factor is large on coarse detents, the motor ends up making noise
    // because the P&D factors amplify the noise from the sensor. This is a
    // piecewise linear function so that fine detents (small width) get a
    // higher D factor and coarse detents get a small D factor. Fine detents
    // need a nonzero D factor to artificially create "clicks" each time a new
    // value is reached (the P factor is small for fine detents due to the
    // smaller angular errors, and the existing P factor doesn't work well for
    // very small angle changes (easy to get runaway due to sensor noise &
    // lag)).
    // TODO: consider eliminating this D factor entirely and just "play" a
    // hardcoded haptic "click" (e.g. a quick burst of torque in each
    // direction) whenever the position changes when the detent width is too
    // small for the P factor to work well.
    const float derivative_lower_strength = config.detent_strength * kd_factor_max_;
    const float derivative_upper_strength = config.detent_strength * kd_factor_min_;
    const float derivative_position_width_lower = 3.0f * M_PI / 180.0f; // radians(3);
    const float derivative_position_width_upper = 8.0f * M_PI / 180.0f; // radians(8);
    const float raw = derivative_lower_strength +
                      (derivative_upper_strength - derivative_lower_strength) /
                          (derivative_position_width_upper - derivative_position_width_lower) *
                          (config.position_width - derivative_position_width_lower);
    // When there are intermittent detents (set via detent_positions), disable
    // derivative factor as this adds extra "clicks" when nearing a detent.
    float new_kd =
        config.detent_positions.size() > 0
            ? 0
            : std::clamp<float>(
                  raw, std::min<float>(derivative_lower_strength, derivative_upper_strength),
                  std::max<float>(derivative_lower_strength, derivative_upper_strength));
    // update the PID parameters
    auto pid_config = detent_pid_.get_config();
    pid_config.kd = new_kd;
    // we don't want to clear the PID state when we change the config, so we
    // pass false
    logger_.info("Updating detent PID config: {}", pid_config);
    detent_pid_.set_config(pid_config, false);
  }

  /// @brief Play haptic feedback
  /// @note Plays a somewhat-configurable haptic "buzz" / "click" for the user
  /// @note This is a blocking call that will wait for the haptic feedback to
  ///       finish before returning. It will also block the motor/detent task
  ///       from running until the haptic feedback is finished.
  /// @param config Configuration for the haptic feedback
  void play_haptic(const detail::HapticConfig &config) {
    std::unique_lock<std::mutex> lk(motor_mutex_);
    // TODO: use the config frequency
    // TODO: use the config duration
    // TODO: convert this to a non-blocking call (put data into a queue and perform the actions in
    // the task)
    // TODO: Use the PID controller to control the haptics
    // Play a hardcoded haptic "click"
    float strength = config.strength; // 5 or 1.5 were used in SmartKnob
    motor_.get().move(strength);
    using namespace std::chrono_literals;
    for (uint8_t i = 0; i < 3; i++) {
      motor_.get().loop_foc();
      std::this_thread::sleep_for(1ms);
    }
    motor_.get().move(-strength);
    for (uint8_t i = 0; i < 3; i++) {
      motor_.get().loop_foc();
      std::this_thread::sleep_for(1ms);
    }
    motor_.get().move(0);
    motor_.get().loop_foc();
  }

protected:
  /// @brief Task which runs the haptic motor
  /// @param m Mutex to use for the task
  /// @param cv Condition variable to use for the task
  /// @param task_notified True if the task has been notified, false otherwise
  /// @return True if the task should be stopped, false otherwise
  bool motor_task(std::mutex &m, std::condition_variable &cv, bool &task_notified) {
    auto start_time = std::chrono::steady_clock::now();
    // if we are not moving, and we're close to the center (but not exactly at
    // the center), slowly move back to the center

    // get the current detent config (copy it so we don't hold the mutex for too long)
    detail::DetentConfig detent_config;
    {
      std::unique_lock<std::mutex> lk(detent_mutex_);
      detent_config = detent_config_;
    }

    {
      std::unique_lock<std::mutex> lk(motor_mutex_);
      // manage the haptics (detents, positions, etc.)
      // apply motor torque based on angle to the nearest position (strength is
      // handled by the PID parameters)

      // The snap point determines the position at which we snap to the next
      // detent. If we're close enough to the snap point, we snap to the next
      // detent. If we're not close enough to the snap point, we apply a torque
      // to move us towards the snap point.
      // The snap point is a percentage of the detent width, and is relative to
      // the detent center. For example, if the snap point is 1.1, and the detent
      // width is 10 degrees, then the snap point is 1.1 * 10 = 11 degrees away
      // from the detent center. If the detent center is at 0 degrees, then the
      // snap point is at 11 degrees. If the detent center is at 5 degrees, then
      // the snap point is at 16 degrees.

      // check our position vs the nearest detent, and update our position if we're
      // close enough to snap to another detent
      float motor_angle = motor_.get().get_shaft_angle();
      float angle_to_detent_center = motor_angle - current_detent_center_;

      // Handle the snap point - if we're close enough to the snap point, snap
      // to the snap point
      float snap_point_radians = detent_config.position_width * detent_config.snap_point;
      // the bias is the amount of the detent width that we bias the snap point
      // by. For example, if the bias is 0.1, and the detent width is 10 degrees,
      // then the snap point is 11 degrees away from the detent center. If the
      // bias is 0.2, then the snap point is 12 degrees away from the detent
      // center.
      float bias_radians = detent_config.position_width * detent_config.snap_point_bias;
      float snap_point_radians_decrease =
          snap_point_radians + (current_position_ <= 0 ? bias_radians : -bias_radians);
      float snap_point_radians_increase =
          -snap_point_radians + (current_position_ >= 0 ? -bias_radians : bias_radians);

      int32_t num_positions = detent_config.max_position - detent_config.min_position + 1;
      // update our position if we're close enough to snap to another detent
      // (and we're not at the end of the range)
      if (angle_to_detent_center > snap_point_radians_decrease &&
          (num_positions <= 0 || current_position_ > detent_config.min_position)) {
        // we're past the snap point, so snap to the next detent
        current_detent_center_ += detent_config.position_width;
        angle_to_detent_center -= detent_config.position_width;
        current_position_--;
        logger_.info("Position: {}", current_position_);
      } else if (angle_to_detent_center < snap_point_radians_increase &&
                 (num_positions <= 0 || current_position_ < detent_config.max_position)) {
        // we're past the snap point, so snap to the next detent
        current_detent_center_ -= detent_config.position_width;
        angle_to_detent_center += detent_config.position_width;
        current_position_++;
        logger_.info("Position: {}", current_position_);
      }

      float dead_zone_adjustment =
          std::clamp(angle_to_detent_center,
                     std::max(-detent_config.position_width * detent_config.dead_zone_percent,
                              -detent_config.dead_zone_abs_max_radians),
                     std::min(detent_config.position_width * detent_config.dead_zone_percent,
                              detent_config.dead_zone_abs_max_radians));

      bool out_of_bounds =
          num_positions > 0 &&
          ((angle_to_detent_center > 0 && current_position_ == detent_config.min_position) ||
           (angle_to_detent_center < 0 && current_position_ == detent_config.max_position));

      logger_.debug_rate_limited("\n"
                                 "\tCurrent position:       {}\n"
                                 "\tcurrent detent center:  {}\n"
                                 "\tmotor angle:            {}\n"
                                 "\tangle_to_detent_center: {}\n"
                                 "\tdead_zone_adjustment:   {}\n"
                                 "\tout_of_bounds:          {}",
                                 current_position_, current_detent_center_, motor_angle,
                                 angle_to_detent_center, dead_zone_adjustment, out_of_bounds);

      // update the PID parameters based on our position
      Pid::Config pid_config = detent_pid_.get_config();
      pid_config.kp = out_of_bounds
                          ? detent_config.end_strength *
                                kp_factor_ // if we're out of bounds, then we apply end stop force
                          : detent_config.detent_strength *
                                kp_factor_; // if we're in bounds, then we apply detent force
      // we don't want to clear the PID state when we change the config, so we pass false
      detent_pid_.set_config(pid_config, false);

      // Apply motor torque based on our angle to the nearest detent (detent
      // strength, etc is handled by the pid parameters)
      if (std::abs(motor_.get().get_shaft_velocity()) > 60) {
        // Don't apply torque if velocity is too high (helps avoid positive
        // feedback loop/runaway)
        logger_.info_rate_limited("velocity too high, not applying torque");
        motor_.get().move(0);
      } else {
        // apply torque based on our angle to the nearest detent
        float input = -angle_to_detent_center + dead_zone_adjustment;
        // if we're out of bounds, then we apply torque regardless of our
        // position
        if (!out_of_bounds && detent_config.detent_positions.size() > 0) {
          // if there are manually specified detents, then we only apply torque
          // if we're in a detent
          bool in_detent = std::any_of(detent_config.detent_positions.begin(),
                                       detent_config.detent_positions.end(),
                                       [this](int y) { return current_position_ == y; });
          // if we're not in a detent, then we don't apply any torque
          if (!in_detent) {
            input = 0;
          }
        }
        // get the torque from the PID controller
        float torque = detent_pid_.update(input);
        logger_.debug_rate_limited("angle: {:0.3f}, input: {:0.3f}, torque: {:0.3f}", motor_angle,
                                   input, torque);
        // apply the torque to the motor
        motor_.get().move(torque);
      } // end if std::abs(motor_.get().get_shaft_velocity()) > 60
      motor_.get().loop_foc();
    } // end motor_mutex_

    // now sleep
    {
      using namespace std::chrono_literals;
      std::unique_lock<std::mutex> lk(m);
      cv.wait_until(lk, start_time + 1ms, [&task_notified] { return task_notified; });
      task_notified = false;
    }

    // don't want to stop the task
    return false;
  }

  Pid detent_pid_;                              ///< PID controller for the detents
  float kp_factor_{0};                          ///< kp factor for the PID controller
  float kd_factor_min_{0};                      ///< Minimum kd factor for the PID controller
  float kd_factor_max_{0};                      ///< Maximum kd factor for the PID controller
  std::atomic<int> current_position_{0};        ///< Current position of the motor
  std::atomic<float> current_detent_center_{0}; ///< Current center of the detent
  std::mutex detent_mutex_;                     ///< Mutex for accessing the detents
  detail::DetentConfig detent_config_;          ///< Configuration for the detents

  std::mutex motor_mutex_;          ///< Mutex for accessing the motor
  std::reference_wrapper<M> motor_; ///< Pointer to the motor to use for haptics

  std::unique_ptr<Task> motor_task_; ///< Task which runs the haptic motor
};
} // namespace espp

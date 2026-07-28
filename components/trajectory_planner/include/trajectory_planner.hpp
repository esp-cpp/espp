#pragma once

#include <algorithm>
#include <cmath>
#include <mutex>

#include "base_component.hpp"

namespace espp {

/**
 *  @brief Converts normalized joystick velocity commands into smooth,
 *         dynamically feasible chassis motion commands (v, ω).
 *
 *  The planner is drive-system independent — it does not know about wheel
 *  geometry or kinematics. It only enforces velocity, acceleration, and
 *  (optionally) jerk limits on chassis-level commands. The downstream
 *  kinematics layer converts (v_ref, ω_ref) into individual motor commands.
 *
 *  Two generation modes are selected automatically based on configuration:
 *  - **Acceleration-limited** (trapezoidal profile): set max_linear_jerk and
 *    max_angular_jerk to 0.
 *  - **Jerk-limited** (S-curve profile): set non-zero jerk limits.
 *
 *  This class is thread-safe: set_target(), update(), output(), stop(), and
 *  reset() may be called from different threads concurrently.
 *
 * \section trajectory_planner_ex1 Acceleration-Limited Example
 * \snippet trajectory_planner_example.cpp trajectory_planner example
 * \section trajectory_planner_ex2 Jerk-Limited Example
 * \snippet trajectory_planner_example.cpp trajectory_planner jerk example
 */
class TrajectoryPlanner : public BaseComponent {
public:
  /**
   * @brief Chassis motion command produced by the planner.
   */
  struct MotionCommand {
    float linear_velocity{0.0f};  /**< Linear velocity reference (m/s). */
    float angular_velocity{0.0f}; /**< Angular velocity reference (rad/s). */
  };

  /**
   * @brief Configuration for the TrajectoryPlanner.
   */
  struct Config {
    float max_linear_velocity;      /**< Maximum linear velocity magnitude (m/s). */
    float max_angular_velocity;     /**< Maximum angular velocity magnitude (rad/s). */
    float max_linear_acceleration;  /**< Maximum linear acceleration magnitude (m/s²). */
    float max_angular_acceleration; /**< Maximum angular acceleration magnitude (rad/s²). */
    float max_linear_jerk{0.0f};    /**< Maximum linear jerk (m/s³). 0 disables jerk limiting. */
    float max_angular_jerk{0.0f}; /**< Maximum angular jerk (rad/s³). 0 disables jerk limiting. */
    bool enforce_motion_envelope{false}; /**< When true, enforces (v/vmax)²+(ω/ωmax)²≤1 on
                                              output to prevent infeasible combined commands. */
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Construct the planner with the given configuration.
   * @param config Configuration parameters.
   */
  explicit TrajectoryPlanner(const Config &config);

  /**
   * @brief Update the planner configuration.
   * @param config New configuration parameters.
   * @param reset_state If true (default), resets velocity/acceleration state to zero.
   */
  void set_config(const Config &config, bool reset_state = true);

  /**
   * @brief Get the current configuration.
   * @return Const reference to the active Config.
   */
  const Config &get_config() const;

  /**
   * @brief Set the desired chassis velocity target.
   *
   * Inputs are physical velocities. The caller typically pre-scales:
   * @code
   *   planner.set_target(v_cmd * max_linear_velocity, ω_cmd * max_angular_velocity);
   * @endcode
   * Inputs are clamped to the configured velocity limits.
   *
   * @param linear_velocity  Desired linear velocity (m/s).
   * @param angular_velocity Desired angular velocity (rad/s).
   */
  void set_target(float linear_velocity, float angular_velocity);

  /**
   * @brief Advance the planner by one time step.
   *
   * Applies velocity, acceleration, and jerk limits to step the internal
   * state towards the current target. Call at a fixed control-loop rate, then
   * read the result via output().
   *
   * @param dt Time step in seconds. Must be > 0; ignored otherwise.
   */
  void update(float dt);

  /**
   * @brief Get the current smoothed motion command.
   * @return MotionCommand containing the trajectory-limited (v_ref, ω_ref).
   */
  MotionCommand output() const;

  /**
   * @brief Command the planner to decelerate to a full stop.
   *
   * Equivalent to set_target(0, 0). The planner ramps down respecting all
   * configured limits rather than cutting output immediately.
   */
  void stop();

  /**
   * @brief Reset velocity, acceleration state, and target to zero immediately.
   *
   * The next call to output() will return (0, 0). Use after an emergency stop
   * or before re-initialising with a new configuration.
   */
  void reset();

protected:
  /// Internal motion state tracked between update() calls.
  struct State {
    float v{0.0f};   /**< Current linear velocity (m/s). */
    float w{0.0f};   /**< Current angular velocity (rad/s). */
    float a_v{0.0f}; /**< Current linear acceleration (m/s²) — jerk-limited mode only. */
    float a_w{0.0f}; /**< Current angular acceleration (rad/s²) — jerk-limited mode only. */
  };

  Config config_;
  State state_{};
  float target_v_{0.0f};
  float target_w_{0.0f};
  mutable std::recursive_mutex mutex_;
};

} // namespace espp

#include "trajectory_planner_formatters.hpp"

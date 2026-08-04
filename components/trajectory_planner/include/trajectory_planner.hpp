#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <utility>

#include "base_component.hpp"
#include "timer.hpp"

namespace espp {

/**
 *  @brief Converts normalized joystick velocity commands into smooth,
 *         dynamically feasible chassis motion commands (v, w).
 *
 *  The planner is drive-system independent -- it does not know about wheel
 *  geometry or kinematics. It only enforces velocity, acceleration, and
 *  jerk limits on chassis-level commands. The downstream kinematics layer
 *  converts (v_ref, w_ref) into individual motor commands.
 *
 *  ### Algorithm
 *  The jerk-limited mode uses a discrete optimal-control approach: at each
 *  step the planner computes the minimum velocity-change distance needed to
 *  decelerate the current acceleration to zero, then decides whether to
 *  accelerate, maintain, or decelerate to land exactly on the target without
 *  overshoot -- equivalent to a time-optimal S-curve under jerk and
 *  acceleration constraints.
 *
 *  ### Profiles
 *  Motion limits are grouped into two MotionProfile objects inside Config:
 *  - **driving_profile** -- used whenever the target is non-zero.
 *  - **stopping_profile** -- used when the target is (0, 0). Setting jerk to 0
 *    gives a trapezoidal stop; higher acceleration gives faster, firmer braking.
 *
 *  A MotionProfile selects its mode automatically:
 *  - **Trapezoidal**: `max_linear_jerk == 0 && max_angular_jerk == 0`
 *  - **S-curve**: either jerk field is non-zero
 *
 *  ### Timing
 *  Two independent `espp::Timer` instances run internally:
 *  - **planning timer** -- calls `update()` at `planning_period` (default 20 ms / 50 Hz).
 *    Recommended range: 5-200 ms on microcontrollers.
 *  - **callback timer** -- fires `output_callback` at `callback_period` (default 40 ms).
 *    Should be >= 2x planning_period (Nyquist); faster rates repeat the same output.
 *
 *  This class is thread-safe: set_target(), get_target(), output(), stop(),
 *  and reset() may be called from different threads concurrently.
 *
 * \section trajectory_planner_ex0 Quick-Start: Full Public API
 * \snippet trajectory_planner_example.cpp trajectory_planner quickstart
 * \section trajectory_planner_ex1 S-Curve Driving / Trapezoidal Stop
 * \snippet trajectory_planner_example.cpp trajectory_planner example
 * \section trajectory_planner_ex2 High-Speed S-Curve with Centripetal Limiting
 * \snippet trajectory_planner_example.cpp trajectory_planner jerk example
 * \section trajectory_planner_ex3 Constraint Validation
 * \snippet trajectory_planner_example.cpp trajectory_planner validation
 */
class TrajectoryPlanner : public BaseComponent {
public:
  /**
   * @brief Chassis motion command produced by the planner.
   */
  struct MotionCommand {
    float linear_velocity = 0.0f;  /**< Linear velocity reference (m/s). */
    float angular_velocity = 0.0f; /**< Angular velocity reference (rad/s). */
  };

  /**
   * @brief Callback invoked at the end of every update() call with the latest
   *        smoothed MotionCommand. Use to feed the output directly into a
   *        kinematics layer without polling output().
   * @note  The callback is called without holding the internal mutex, so it is
   *        safe to call set_target(), stop(), or output() from within it.
   */
  typedef std::function<void(const MotionCommand &)> output_callback_t;

  /**
   * @brief Acceleration and jerk limits for one phase of motion.
   *
   * Set max_linear_jerk / max_angular_jerk to 0 for a trapezoidal (ramp)
   * profile, or to a positive value for an S-curve profile.
   *
   * @note Using a trapezoidal stopping profile (jerk = 0) is recommended to
   *       avoid S-curve overshoot past zero when the planner decelerates from
   *       a jerk-limited driving phase.
   */
  struct MotionProfile {
    float max_linear_acceleration = 0.0f;  /**< Linear acceleration limit (m/s^2). */
    float max_angular_acceleration = 0.0f; /**< Angular acceleration limit (rad/s^2). */
    float max_linear_jerk = 0.0f;          /**< Linear jerk limit (m/s^3). 0 = trapezoidal. */
    float max_angular_jerk = 0.0f;         /**< Angular jerk limit (rad/s^3). 0 = trapezoidal. */
  };

  /**
   * @brief Configuration for the TrajectoryPlanner.
   */
  struct Config {
    float max_linear_velocity;  /**< Maximum linear velocity magnitude (m/s). */
    float max_angular_velocity; /**< Maximum angular velocity magnitude (rad/s). */
    espp::TrajectoryPlanner::MotionProfile
        driving_profile; /**< Accel/jerk limits used when target != (0, 0). */
    espp::TrajectoryPlanner::MotionProfile stopping_profile; /**< Accel/jerk limits used when target
                                         == (0, 0). Set jerk to 0 here for a clean trapezoidal stop
                                         with no overshoot. Higher acceleration than the
                                         driving profile gives faster, firmer braking. */
    bool enforce_motion_envelope = false;      /**< When true, enforces (v/vmax)^2+(w/wmax)^2<=1 on
                                                   output to prevent infeasible combined commands. */
    float max_centripetal_acceleration = 0.1f; /**< Maximum centripetal acceleration |v*w| (m/s^2).
                                                   0 disables the limit. Both v and w are scaled
                                                   proportionally when the limit is exceeded. */
    espp::TrajectoryPlanner::output_callback_t output_callback =
        nullptr; /**< Optional callback invoked after each update()
with the latest MotionCommand output.
Leave as nullptr to disable. */
    // --- Periodic task configuration ---
    std::chrono::duration<float> planning_period = std::chrono::milliseconds(20); /**< Planner
                                                     update rate (default 50 Hz). Recommended: 5-200
                                                     ms on microcontrollers. */
    std::chrono::duration<float> callback_period = std::chrono::milliseconds(40); /**< Output
                                                     callback rate (default 50 Hz). Should be
                                                     >= 2x planning_period (Nyquist); a faster
                                                     rate will repeat the same output. */
    espp::Task::BaseConfig planning_task_config = {
        .name = "TP_planning",
        .stack_size_bytes = 4096,
        .priority = 0,
        .core_id = -1}; /**< Underlying task config for the timer. */
    espp::Task::BaseConfig callback_task_config = {
        .name = "TP_cb",
        .stack_size_bytes = 8192,
        .priority = 0,
        .core_id = -1}; /**< Underlying task config for the callback timer. */
    espp::Logger::Verbosity log_level = espp::Logger::Verbosity::WARN; /**< Logger verbosity. */
  };

  /**
   * @brief Construct the planner with the given configuration.
   *        The periodic update task is started automatically.
   * @param config Configuration parameters.
   */
  explicit TrajectoryPlanner(const Config &config);

  /**
   * @brief Destructor. Stops the periodic update task before destruction.
   */
  ~TrajectoryPlanner();

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
   * @brief Set the desired chassis velocity target using normalized joystick inputs.
   *
   * Both inputs are in the range [-1, +1] and are scaled internally:
   * @code
   *   v_target  = linear  * max_linear_velocity
   *   w_target  = angular * max_angular_velocity
   * @endcode
   * Values outside [-1, +1] are clamped before scaling.
   *
   * @param linear  Normalized linear velocity command  [-1, +1].
   *                +1 = full forward, -1 = full reverse.
   * @param angular Normalized angular velocity command [-1, +1].
   *                +1 = full left turn, -1 = full right turn.
   */
  void set_target(float linear, float angular);

  /**
   * @brief Get the current normalized velocity target.
   * @note  If enforce_motion_envelope is enabled the stored target may differ
   *        from the value passed to set_target() (it is projected onto the unit circle).
   * @return Pair of {linear, angular} in [-1, +1].
   */
  std::pair<float, float> get_target() const;

  /**
   * @brief Get the current smoothed motion command.
   * @return MotionCommand containing the trajectory-limited (v_ref, w_ref).
   */
  espp::TrajectoryPlanner::MotionCommand output() const;

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

  /**
   * @brief Check whether the periodic update task is currently running.
   * @return true if the task is running.
   */
  bool is_running() const;

protected:
  /// Internal motion state tracked between update() calls.
  struct State {
    float v = 0.0f;   /**< Current linear velocity (m/s). */
    float w = 0.0f;   /**< Current angular velocity (rad/s). */
    float a_v = 0.0f; /**< Current linear acceleration (m/s^2) -- jerk-limited mode only. */
    float a_w = 0.0f; /**< Current angular acceleration (rad/s^2) -- jerk-limited mode only. */
  };

  /**
   * @brief Start the periodic planning and callback timers.
   *
   * The planning timer calls update() at Config::planning_period and the
   * callback timer fires output_callback at Config::callback_period.
   * Has no effect if the timers are already running.
   *
   * @return true if both timers were started successfully.
   */
  bool start_task();

  /**
   * @brief Stop the periodic update task.
   *
   * @return true if the task was stopped, false if it was not running.
   */
  bool stop_task();

  /**
   * @brief Advance the planner by one time step.
   *
   * Called automatically by the periodic task. Can also be called manually
   * when not using the task (e.g. in a user-managed control loop).
   *
   * @param dt Time step in seconds. Must be > 0; ignored otherwise.
   */
  void update(float dt);

  Config config_;
  State state_ = {};
  float target_v_ = 0.0f;
  float target_w_ = 0.0f;
  espp::TrajectoryPlanner::output_callback_t output_callback_ = nullptr;
  std::unique_ptr<espp::Timer> timer_ = nullptr;
  std::unique_ptr<espp::Timer> callback_timer_ = nullptr;
  std::chrono::steady_clock::time_point last_update_time_ = std::chrono::steady_clock::now();
  mutable std::recursive_mutex mutex_;
};

} // namespace espp

#include "trajectory_planner_formatters.hpp"

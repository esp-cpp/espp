#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>

#include "base_component.hpp"
#include "task.hpp"

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
   * @brief Callback invoked at the end of every update() call with the latest
   *        smoothed MotionCommand. Use to feed the output directly into a
   *        kinematics layer without polling output().
   * @note  The callback is called without holding the internal mutex, so it is
   *        safe to call set_target(), stop(), or output() from within it.
   */
  typedef std::function<void(const MotionCommand &)> output_callback_t;

  /**
   * @brief Configuration for the TrajectoryPlanner.
   */
  struct Config {
    float max_linear_velocity;            /**< Maximum linear velocity magnitude (m/s). */
    float max_angular_velocity;           /**< Maximum angular velocity magnitude (rad/s). */
    float max_linear_acceleration;        /**< Maximum linear acceleration magnitude (m/s²). */
    float max_angular_acceleration;       /**< Maximum angular acceleration magnitude (rad/s²). */
    float max_linear_deceleration{0.0f};  /**< Braking deceleration for stops (m/s²).
                                               0 = fall back to max_linear_acceleration. */
    float max_angular_deceleration{0.0f}; /**< Braking deceleration for stops (rad/s²).
                                               0 = fall back to max_angular_acceleration. */
    float max_linear_jerk{0.0f};  /**< Maximum linear jerk (m/s³). 0 disables jerk limiting. */
    float max_angular_jerk{0.0f}; /**< Maximum angular jerk (rad/s³). 0 disables jerk limiting. */
    bool enforce_motion_envelope{false}; /**< When true, enforces (v/vmax)²+(ω/ωmax)²≤1 on
                                              output to prevent infeasible combined commands. */
    float max_centripetal_acceleration{0.1f}; /**< Maximum centripetal acceleration |v·ω| (m/s²).
                                                   0 disables the limit. Both v and ω are scaled
                                                   proportionally when the limit is exceeded. */
    output_callback_t output_callback{nullptr}; /**< Optional callback invoked after each update()
                                                     with the latest MotionCommand output.
                                                     Leave as nullptr to disable. */
    // --- Periodic task configuration ---
    std::chrono::duration<float> update_period{std::chrono::milliseconds(20)}; /**< Period between
                                                     automatic update() calls (default 50 Hz). */
    espp::Task::BaseConfig task_config{.name = "TrajectoryPlanner",
                                       .stack_size_bytes = 4096,
                                       .priority = 0,
                                       .core_id = -1}; /**< Task configuration (name, stack,
                                                            priority, core affinity). */
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
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
   *   ω_target  = angular * max_angular_velocity
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

  /**
   * @brief Check whether the periodic update task is currently running.
   * @return true if the task is running.
   */
  bool is_running() const;

protected:
  /// Internal motion state tracked between update() calls.
  struct State {
    float v{0.0f};   /**< Current linear velocity (m/s). */
    float w{0.0f};   /**< Current angular velocity (rad/s). */
    float a_v{0.0f}; /**< Current linear acceleration (m/s²) — jerk-limited mode only. */
    float a_w{0.0f}; /**< Current angular acceleration (rad/s²) — jerk-limited mode only. */
  };

  /**
   * @brief Start the periodic update task.
   *
   * The task calls update() at the rate configured by Config::update_period
   * and fires the output_callback after each step. Has no effect if the task
   * is already running.
   *
   * @return true if the task was started, false if it was already running.
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
  State state_{};
  float target_v_{0.0f};
  float target_w_{0.0f};
  output_callback_t output_callback_{nullptr};
  std::unique_ptr<espp::Task> task_;
  std::chrono::steady_clock::time_point last_update_time_;
  mutable std::recursive_mutex mutex_;
};

} // namespace espp

#include "trajectory_planner_formatters.hpp"

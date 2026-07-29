# Trajectory Planner Component

[![Badge](https://components.espressif.com/components/espp/trajectory_planner/badge.svg)](https://components.espressif.com/components/espp/trajectory_planner)

The `TrajectoryPlanner` component converts normalized joystick velocity commands
into smooth, dynamically feasible chassis motion commands `(v, ω)`. It is
drive-system independent — the kinematics layer downstream is responsible for
converting chassis commands into individual motor commands.

## Features

- **Normalized inputs** — `set_target(linear, angular)` accepts values in `[-1, +1]`;
  `+1` maps to the configured maximum speed.
- **Velocity limiting** — outputs are bounded by `max_linear_velocity` / `max_angular_velocity`.
- **Acceleration limiting** — trapezoidal (ramp) profiles via `max_linear_acceleration` /
  `max_angular_acceleration`.
- **Separate braking deceleration** — configurable `max_linear_deceleration` /
  `max_angular_deceleration` applied when the target is `(0, 0)`, allowing faster stops
  than normal driving acceleration.
- **Optional jerk limiting** — S-curve profiles when `max_linear_jerk` /
  `max_angular_jerk` are non-zero.
- **Motion envelope enforcement** — optionally constrains `(v/v_max)² + (ω/ω_max)² ≤ 1`
  to prevent infeasible combined commands.
- **Centripetal acceleration limiting** — constrains `|v · ω| ≤ max_centripetal_acceleration`
  (default 0.1 m/s²) to reduce wheel slip during turns.
- **Periodic update task** — built-in `espp::Task` that calls the planner at a fixed
  rate (`update_period`, default 50 Hz). Start with `start_task()`, stop with `stop_task()`.
- **Output callback** — optional `output_callback` fired after every update step, suitable
  for feeding a downstream kinematics layer directly from the control loop thread.
- **Thread-safe** — `set_target()`, `output()`, `stop()`, and `reset()` are safe to call
  from any thread concurrently.

## Usage

```cpp
espp::TrajectoryPlanner planner({
    .max_linear_velocity      = 1.0f,        // m/s
    .max_angular_velocity     = 3.14159f,    // rad/s
    .max_linear_acceleration  = 2.0f,        // m/s²
    .max_angular_acceleration = 6.28f,       // rad/s²
    .max_linear_deceleration  = 6.0f,        // faster braking
    .max_angular_deceleration = 12.0f,
    .enforce_motion_envelope  = true,
    .output_callback = [](const espp::TrajectoryPlanner::MotionCommand &cmd) {
        // forward to kinematics layer every tick
        kinematics.apply(cmd.linear_velocity, cmd.angular_velocity);
    },
    .update_period = std::chrono::milliseconds(10),  // 100 Hz
    .auto_start    = true,
    .priority      = 5,
    .core_id       = 1,
});

// Normalized joystick input: +1.0 = full forward, +0.5 = half left turn
planner.set_target(1.0f, 0.5f);

// Controlled deceleration to stop (respects deceleration limits)
planner.stop();

// Emergency stop — zeroes state immediately
planner.reset();
```

## Example

The [example](./example) shows both acceleration-limited (trapezoidal) and
jerk-limited (S-curve) modes with the output callback and the periodic task.

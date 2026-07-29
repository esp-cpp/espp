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
- **Dual motion profiles** — separate `driving_profile` and `stopping_profile`, each
  independently configuring acceleration and jerk limits:
  - **Trapezoidal** (ramp): set `max_linear_jerk = max_angular_jerk = 0`
  - **S-curve**: set non-zero jerk limits for smooth acceleration
  - Recommended: use an S-curve driving profile and a trapezoidal stopping
    profile (jerk = 0) to get smooth starts with clean, overshoot-free stops.
- **Motion envelope enforcement** — optionally constrains `(v/v_max)² + (ω/ω_max)² ≤ 1`
  to prevent infeasible combined commands.
- **Centripetal acceleration limiting** — constrains `|v · ω| ≤ max_centripetal_acceleration`
  (default 0.1 m/s²) to reduce wheel slip during turns.
- **Periodic update task** — built-in `espp::Task` starts automatically on construction.
  Rate is set via `update_period` (default 50 Hz); task config via `task_config`.
- **Output callback** — optional `output_callback` fired after every update step, suitable
  for feeding a downstream kinematics layer directly from the control loop thread.
- **Thread-safe** — `set_target()`, `output()`, `stop()`, and `reset()` are safe to call
  from any thread concurrently.

## Usage

```cpp
espp::TrajectoryPlanner planner({
    .max_linear_velocity  = 1.0f,       // m/s
    .max_angular_velocity = 3.14159f,   // rad/s

    // S-curve acceleration while driving
    .driving_profile  = {
        .max_linear_acceleration  = 2.0f,   // m/s²
        .max_angular_acceleration = 6.28f,  // rad/s²
        .max_linear_jerk          = 10.0f,  // m/s³
        .max_angular_jerk         = 30.0f,  // rad/s³
    },
    // Trapezoidal (no jerk) stop — fast and overshoot-free
    .stopping_profile = {
        .max_linear_acceleration  = 5.0f,   // m/s²
        .max_angular_acceleration = 10.0f,  // rad/s²
    },

    .enforce_motion_envelope     = true,
    .max_centripetal_acceleration = 0.3f,   // m/s²
    .output_callback = [](const espp::TrajectoryPlanner::MotionCommand &cmd) {
        kinematics.apply(cmd.linear_velocity, cmd.angular_velocity);
    },
    .update_period = std::chrono::milliseconds(10),  // 100 Hz
    .task_config   = {.name = "Planner", .priority = 5, .core_id = 1},
});

// Normalized joystick input: +1.0 = full forward, +0.5 = half left turn
planner.set_target(1.0f, 0.5f);

// Controlled deceleration to stop (uses stopping_profile)
planner.stop();

// Emergency stop — zeroes state immediately
planner.reset();
```

## Example

The [example](./example) shows three scenarios:
1. **Quick-start** — all public API in one place
2. **S-curve driving / trapezoidal stop** — smooth acceleration, clean braking
3. **High-speed S-curve** — combined jerk-limited motion with centripetal limiting

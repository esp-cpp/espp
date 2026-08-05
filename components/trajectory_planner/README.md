# Trajectory Planner Component

[![Badge](https://components.espressif.com/components/espp/trajectory_planner/badge.svg)](https://components.espressif.com/components/espp/trajectory_planner)

The `TrajectoryPlanner` component converts normalized joystick velocity commands
into smooth, dynamically feasible chassis motion commands `(v, w)`. It is
drive-system independent - the kinematics layer downstream is responsible for
converting chassis commands into individual motor commands.

## Algorithm

The jerk-limited mode uses a **discrete optimal-control** approach. At each step
the planner computes the minimum velocity-change distance needed to decelerate
the current acceleration to zero, then decides whether to accelerate, maintain,
or decelerate - landing exactly on the target velocity without overshoot. This
is equivalent to a time-optimal S-curve under joint jerk and acceleration
constraints.

## Features

- **Normalized inputs** - `set_target(linear, angular)` accepts `[-1, +1]`;
  `+/-1` maps to configured maximum speed. `get_target()` reads it back.
- **Velocity limiting** - outputs bounded by `max_linear_velocity` / `max_angular_velocity`.
- **Dual motion profiles** - separate `driving_profile` and `stopping_profile`:
  - **Trapezoidal**: `max_linear_jerk = max_angular_jerk = 0` (default)
  - **S-curve**: non-zero jerk limits; discrete optimal-control landing
  - Recommended: S-curve driving + trapezoidal stopping (jerk = 0) -> smooth starts, overshoot-free stops.
- **Motion envelope enforcement** - optionally constrains `(v/v_max)^2 + (w/w_max)^2 <= 1`.
- **Centripetal acceleration limiting** - constrains `|v * w| <= max_centripetal_acceleration`.
- **Planning timer** - `planning_period` drives `update()` (default 20 ms / 50 Hz);
  recommended range 5-200 ms on microcontrollers.
- **Output callback** - `output_callback` fires on a dedicated task each time `update()`
  produces a new output value (CV-notified); no separate callback rate to configure.
  Safe to call `set_target()` and `output()` from within it.
- **Thread-safe** - `set_target()`, `get_target()`, `output()`, `stop()`, and
  `reset()` are safe to call from any thread concurrently.

## Usage

```cpp
espp::TrajectoryPlanner planner({
    .max_linear_velocity  = 1.0f,       // m/s
    .max_angular_velocity = 3.14159f,   // rad/s

    // S-curve driving profile
    .driving_profile  = {
        .max_linear_acceleration  = 2.0f,   // m/s^2
        .max_angular_acceleration = 6.28f,  // rad/s^2
        .max_linear_jerk          = 10.0f,  // m/s^3  -- enables S-curve
        .max_angular_jerk         = 25.0f,  // rad/s^3
    },
    // Trapezoidal stop (jerk = 0) -- fast, overshoot-free
    .stopping_profile = {
        .max_linear_acceleration  = 5.0f,   // m/s^2
        .max_angular_acceleration = 10.0f,  // rad/s^2
    },

    .enforce_motion_envelope      = true,
    .max_centripetal_acceleration = 0.3f,
    .output_callback = [](const espp::TrajectoryPlanner::MotionCommand &cmd) {
        kinematics.apply(cmd.linear_velocity, cmd.angular_velocity);
    },
    .planning_period      = std::chrono::milliseconds(20),  // 50 Hz
    .planning_task_config = {.name = "tp_plan", .priority = 5, .core_id = 1},
    .callback_task_config = {.name = "tp_cb",   .priority = 4, .core_id = 1},
});

// Normalized joystick input: +1.0 = full forward, +0.5 = half left turn
planner.set_target(1.0f, 0.5f);

// Read back the stored (possibly envelope-projected) target
auto [lin, ang] = planner.get_target();

// Controlled deceleration to stop (uses stopping_profile)
planner.stop();

// Emergency stop -- zeroes state immediately
planner.reset();
```

## Example

The [example](./example) shows four scenarios:
1. **Quick-start** - all public API in one place
2. **S-curve driving / trapezoidal stop** - smooth acceleration, clean braking
3. **High-speed S-curve** - combined jerk-limited motion with centripetal limiting
4. **Constraint validation** - automated checks for speed, acceleration, jerk,
   and centripetal limits across driving and stopping phases separately
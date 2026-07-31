# Trajectory Planner Example

This example demonstrates how to use `espp::TrajectoryPlanner` to smooth
normalized joystick velocity commands `(v_cmd, w_cmd)` into dynamically
feasible chassis motion commands `(v_ref, w_ref)`.

Four scenarios are covered:

1. **Quick-start** - walks through every public API call in sequence
   (`set_target`, `get_target`, `output`, `set_config`, `stop`, `reset`, `is_running`).
2. **S-curve driving / trapezoidal stop** - jerk-limited ramp-up at `planning_period = 50 ms`
   followed by a trapezoidal (no-jerk) stop.
3. **High-speed S-curve** - combined linear + angular jerk-limited motion with
   motion envelope and centripetal acceleration enforcement.
4. **Constraint validation** - automated pass/fail checks on every output tick:
   - Speed vs. `max_linear_velocity` / `max_angular_velocity`
   - Acceleration vs. `driving_profile` or `stopping_profile` limits
   - Jerk vs. configured jerk limits
   - Centripetal acceleration `|v * w|`
   - Results reported separately for driving and stopping phases.

## Building

```bash
idf.py build
```

## Running

```bash
idf.py flash monitor
```

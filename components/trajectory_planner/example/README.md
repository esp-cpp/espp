# Trajectory Planner Example

This example demonstrates how to use `espp::TrajectoryPlanner` to smooth
normalized joystick velocity commands `(v_cmd, ω_cmd)` into dynamically
feasible chassis motion commands `(v_ref, ω_ref)`.

Three scenarios are covered:

1. **Quick-start** — walks through every public API call in sequence
2. **S-curve driving / trapezoidal stop** — smooth acceleration, clean braking
3. **High-speed S-curve** — combined jerk-limited motion with centripetal limiting

## Building

```bash
idf.py build
```

## Running

```bash
idf.py flash monitor
```

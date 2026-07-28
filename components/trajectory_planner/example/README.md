# Trajectory Planner Example

This example demonstrates how to use `espp::TrajectoryPlanner` to smooth
normalized joystick velocity commands `(v_cmd, ω_cmd)` into dynamically
feasible chassis motion commands `(v_ref, ω_ref)`.

## Building

```bash
idf.py build
```

## Running

```bash
idf.py flash monitor
```

# Trajectory Planner Component

[![Badge](https://components.espressif.com/components/espp/trajectory_planner/badge.svg)](https://components.espressif.com/components/espp/trajectory_planner)

The `TrajectoryPlanner` component converts normalized joystick velocity commands
into smooth, dynamically feasible chassis motion commands `(v, ω)`. It is
drive-system independent — the kinematics layer downstream is responsible for
converting chassis commands into individual motor commands.

Key features:
- Velocity limiting
- Acceleration limiting
- Optional jerk limiting
- Smooth start/stop and direction changes

## Example

The [example](./example) shows how to use the `espp::TrajectoryPlanner` class
to smooth joystick inputs before passing them to a kinematics layer.

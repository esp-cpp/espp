Trajectory Planner APIs
***********************

Trajectory Planner
------------------

The `TrajectoryPlanner` component converts normalized joystick velocity commands
``(v_cmd, w_cmd)`` into smooth, dynamically feasible chassis motion commands
``(v_ref, w_ref)``. It is drive-system independent -- the downstream kinematics
layer is responsible for converting chassis commands into individual motor commands.

The jerk-limited mode uses a **discrete optimal-control** approach: at each step
the planner computes the minimum velocity-change distance needed to decelerate
the current acceleration to zero, then decides whether to accelerate, maintain,
or decelerate -- landing exactly on the target without overshoot, equivalent to
a time-optimal S-curve under joint jerk and acceleration constraints.

Code examples are provided in the `trajectory_planner` example folder.

.. ------------------------------- Example -------------------------------------

.. toctree::

   trajectory_planner_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/trajectory_planner.inc

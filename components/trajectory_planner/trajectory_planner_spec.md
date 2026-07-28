# Generic Joystick Trajectory Planner Specification

## Purpose

The Trajectory Planner converts joystick velocity commands into smooth, dynamically feasible chassis motion commands.

It is drive-system independent and does not know whether the robot uses:

- Differential drive
- Ackermann steering
- Tricycle drive
- Mecanum wheels
- Omni wheels
- Swerve drive
- Tracked vehicles
- Custom multi-motor configurations

The planner's sole responsibility is generating a smooth chassis motion command:

```text
(v, ω)
```

The motor controller / kinematics layer is responsible for converting these chassis commands into individual motor commands.

---

# Architecture

```text
Joystick
    ↓
Desired Motion Command
(v_cmd, ω_cmd)
    ↓
Trajectory Planner
    ↓
Smoothed Motion Command
(v_ref, ω_ref)
    ↓
Kinematics Layer
    ↓
Wheel / Motor Targets
    ↓
Motor Controllers
    ↓
Motors
```

## Responsibility Split

### Trajectory Planner

Responsible for:

- Velocity limiting
- Acceleration limiting
- Jerk limiting (optional)
- Smooth start/stop
- Direction changes
- Command ramping
- Feasibility enforcement

### Kinematics Layer

Responsible for converting:

```text
(v_ref, ω_ref)
        →
motor commands
```

Examples:

- Differential drive → left/right wheel speeds
- Ackermann → steering angle + wheel speed
- Swerve → wheel angles + wheel speeds

---

# Inputs

## Command Input

Normalized joystick commands:

```cpp
v_cmd ∈ [-1, +1]
ω_cmd ∈ [-1, +1]
```

Application-level scaling:

```cpp
v_target = v_cmd * max_linear_velocity;
ω_target = ω_cmd * max_angular_velocity;
```

## Configuration Parameters

### Velocity Limits

```cpp
max_linear_velocity
max_angular_velocity
```

### Acceleration Limits

```cpp
max_linear_acceleration
max_angular_acceleration
```

### Optional Jerk Limits

```cpp
max_linear_jerk
max_angular_jerk
```

### Update Period

```cpp
dt
```

---

# Outputs

```cpp
struct MotionCommand
{
    float linearVelocity;   // m/s
    float angularVelocity;  // rad/s
};
```

Output:

```text
v_ref
ω_ref
```

---

# Internal States

## Minimum State Set

```cpp
struct State
{
    float v;
    float ω;
};
```

## Extended State Set (Jerk-Limited)

```cpp
struct State
{
    float v;
    float ω;

    float a_v;
    float a_ω;
};
```

---

# Features

## 1. Velocity Limiting

```text
|v| ≤ vmax
|ω| ≤ ωmax
```

## 2. Acceleration Limiting

```text
|dv/dt| ≤ amax
|dω/dt| ≤ αmax
```

Benefits:

- Smoother driving
- Reduced wheel slip
- Lower motor stress
- Better operator feel

## 3. Jerk Limiting (Optional)

```text
|da/dt| ≤ jmax
```

Benefits:

- Reduced vibration
- Lower current spikes
- Improved drivetrain life
- More natural response

## 4. Smooth Start/Stop

Transforms:

```text
0 → 100%
```

into:

```text
0 → 10 → 20 → 30 → ...
```

## 5. Direction Reversal Handling

Safely transitions:

```text
+V
↓
0
↓
−V
```

while respecting configured limits.

## 6. Emergency Stop Ramp

```cpp
planner.stop();
```

Generates a controlled deceleration trajectory to zero.

## 7. Motion Envelope Enforcement

Example feasibility constraint:

```text
(v/vmax)² + (ω/ωmax)² ≤ 1
```

Prevents impossible combinations of linear and angular velocity.

---

# Trajectory Generation Modes

## Mode 1: Acceleration-Limited

State:

```text
v
ω
```

Characteristics:

- Trapezoidal profile
- Computationally lightweight
- Suitable for most robots

## Mode 2: Jerk-Limited

State:

```text
v
ω
a_v
a_ω
```

Characteristics:

- S-curve profile
- Very smooth response
- Preferred for high-performance motion systems

---

# Public API

## Configuration

```cpp
struct PlannerConfig
{
    float maxLinearVelocity;
    float maxAngularVelocity;

    float maxLinearAcceleration;
    float maxAngularAcceleration;

    float maxLinearJerk;
    float maxAngularJerk;
};
```

## Core Interface

```cpp
class TrajectoryPlanner
{
public:

    void configure(const PlannerConfig& cfg);

    void setTarget(
        float linearVelocity,
        float angularVelocity);

    void update(float dt);

    MotionCommand output() const;

    void stop();

    void reset();
};
```

### Runtime Commands

```cpp
setTarget(v_target, ω_target);
```

```cpp
update(dt);
```

```cpp
MotionCommand cmd = output();
```

```cpp
stop();
```

```cpp
reset();
```

---

# Design Principles

1. Platform-independent
2. Real-time safe
3. Deterministic
4. Reusable
5. Smooth motion generation
6. Physically feasible output

---

# Final Interface Contract

## Input

```cpp
(v_cmd, ω_cmd)
```

Desired chassis velocity from joystick.

## Output

```cpp
(v_ref, ω_ref)
```

Trajectory-limited chassis velocity reference.

## Not Responsible For

- Wheel speed calculation
- Steering angle calculation
- Motor control loops
- Encoder feedback
- Drive-specific kinematics

These functions belong to the downstream kinematics and motor-control layers. The trajectory planner remains a pure chassis motion planner that transforms operator intent into smooth, achievable robot motion commands.

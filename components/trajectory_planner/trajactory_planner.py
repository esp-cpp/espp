import numpy as np
import matplotlib.pyplot as plt


def _sd(a_abs, step, dt):
    """Minimum velocity-change distance when decelerating |a| → 0 at max jerk."""
    n = int(a_abs / step)
    return n * a_abs * dt - n * (n + 1) * 0.5 * step * dt


def stopping_thresholds(a, step, dt):
    """Return (d_now, d_maintain, d_increase) with sign of a."""
    if a == 0.0:
        return 0.0, 0.0, step * dt  # special case: first step in either direction
    sign = 1 if a > 0 else -1
    a_abs = abs(a)
    d_now = _sd(a_abs, step, dt)
    d_maintain = a_abs * dt + d_now
    d_increase = (a_abs + step) * dt + _sd(a_abs + step, step, dt)
    return sign * d_now, sign * d_maintain, sign * d_increase

def jerk_limited_velocity_profile(
    v0,
    v_target,
    a_max,
    j_max,
    dt=0.001,
    tolerance=2e-1
):
    """
    Generate a jerk-limited velocity profile.

    Parameters
    ----------
    v0 : float
        Initial velocity
    v_target : float
        Target velocity
    a_max : float
        Maximum acceleration
    j_max : float
        Maximum jerk
    dt : float
        Simulation timestep
    tolerance : float
        Stop condition on velocity error

    Returns
    -------
    t, v, a, j, target : numpy arrays
    """

    v = v0
    a = 0.0

    t_hist = [0.0]
    v_hist = [v]
    a_hist = [a]
    j_hist = [0.0]
    target_hist = [v_target]

    t = 0.0
    pre_t = t
    settled_at = None  # time when tolerance was first reached

    while t < 20.0:

        distance_to_stop = v_target - v
        a_prev = a
        step = j_max * dt

        d_now, d_maintain, d_increase = stopping_thresholds(a, step, dt)
        print (f"distance_to_stop: {distance_to_stop:.4f}, d_now: {d_now:.4f}, d_increase: {d_increase:.4f}, d_maintain: {d_maintain:.4f}")

        if distance_to_stop > 0:
            if distance_to_stop > d_increase:
                a += step
            elif distance_to_stop <= d_maintain:
                a = distance_to_stop/dt if abs(a) < step else a - step
                print(f"Decreasing acceleration: a={a:.4f}")
            else:
                print(f"Maintaining acceleration: a={a:.4f}")
        elif distance_to_stop < 0:
            if distance_to_stop < d_increase:
                a -= step
            elif distance_to_stop >= d_maintain:
                a = distance_to_stop/dt if abs(a) < step else a + step
                print(f"Increasing acceleration: a={a:.4f}")
            else:
                print(f"Maintaining acceleration: a={a:.4f}")
        else:
            a = 0.0
            print(f"Distance to stop is zero, set acceleration to zero: a={a:.4f}")
        # clip acceleration based on the jerk 
        delta_a = a - a_prev
        if abs(delta_a) > step:
            a = a_prev + np.sign(delta_a) * step


        a = np.clip(a, -a_max, a_max)

        v += a * dt
        pre_t = t
        t += dt
        if (pre_t <= 4 and t > 4):
            v_target +=5
        if (pre_t <= 8 and t > 8):
            v_target +=5
        if (pre_t <= 12 and t > 12):
            v_target -=5
        if (pre_t <= 13 and t > 13):  # new target before reaching previous target
            v_target +=5
        if (pre_t <= 17 and t > 17):
            v_target -=5
        if (pre_t <= 18 and t > 18): # set new target to current speed
            v_target =v
                
        target_hist.append(v_target)

        t_hist.append(t)
        v_hist.append(v)
        a_hist.append(a)
        j_hist.append((a - a_prev) / dt)  # true applied jerk after a_max clamp

    return (
        np.array(t_hist),
        np.array(v_hist),
        np.array(a_hist),
        np.array(j_hist),
        np.array(target_hist),
    )


# Example
v_target = -3.0
t, v, a, j, target = jerk_limited_velocity_profile(
    v0=0.0,
    v_target=v_target,
    a_max=2.0,
    j_max=2.0,
    dt=0.051
)


fig, ax = plt.subplots(3, 1, figsize=(10, 8))

ax[0].plot(t, v)
ax[0].plot(t, target, 'r--', label='target')
ax[0].legend()
ax[0].set_ylabel("Velocity")

ax[1].plot(t, a)
ax[1].set_ylabel("Acceleration")

ax[2].plot(t, j)
ax[2].set_ylabel("Jerk")
ax[2].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
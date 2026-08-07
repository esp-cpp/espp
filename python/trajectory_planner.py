"""trajectory_planner.py - demonstrates espp.TrajectoryPlanner in Python.

Run:
    python trajectory_planner.py
"""
import sys
import time
import math
import threading
import espp
import matplotlib.pyplot as plt

# ── data collection (thread-safe) ────────────────────────────────────────────
_lock   = threading.Lock()
t_hist   = []
v_hist   = []
w_hist   = []
a_v_hist = []
a_w_hist = []
j_v_hist = []
j_w_hist = []
ev_hist  = []          # (time, label) for scenario markers
dt_hist   = []

_prev_t  = [None]
_prev_v  = [0.0]; _prev_w  = [0.0]
_prev_av = [0.0]; _prev_aw = [0.0]

start = time.time()

def on_output(cmd):
    now = time.time()
    with _lock:
        t    = now - start
        dt   = (t - _prev_t[0]) if _prev_t[0] is not None else 0.0
        _prev_t[0] = t
        v, w = cmd.linear_velocity, cmd.angular_velocity
        a_v  = (v - _prev_v[0]) / dt if dt > 0 else 0.0
        a_w  = (w - _prev_w[0]) / dt if dt > 0 else 0.0
        j_v  = (a_v - _prev_av[0]) / dt if dt > 0 else 0.0
        j_w  = (a_w - _prev_aw[0]) / dt if dt > 0 else 0.0
        _prev_v[0], _prev_w[0]   = v, w
        _prev_av[0], _prev_aw[0] = a_v, a_w
        t_hist.append(t); v_hist.append(v); w_hist.append(w)
        a_v_hist.append(a_v); a_w_hist.append(a_w)
        j_v_hist.append(j_v); j_w_hist.append(j_w)
        dt_hist.append(dt)

def mark(label):
    with _lock:
        ev_hist.append((time.time() - start, label))
    print(f"\n-- {label} --")

# ── build config ─────────────────────────────────────────────────────────────
driving = espp.TrajectoryPlanner.MotionProfile()
driving.max_linear_acceleration  = 2.0
driving.max_angular_acceleration = 6.28
driving.max_linear_jerk          = 10.0
driving.max_angular_jerk         = 25.0

stopping = espp.TrajectoryPlanner.MotionProfile()
stopping.max_linear_acceleration  = 5.0
stopping.max_angular_acceleration = 10.0
stopping.max_linear_jerk          = 20.0
stopping.max_angular_jerk         = 35.0

cfg = espp.TrajectoryPlanner.Config()
cfg.max_linear_velocity           = 2.0
cfg.max_angular_velocity          = math.pi
cfg.driving_profile               = driving
cfg.stopping_profile              = stopping
cfg.enforce_motion_envelope       = True
cfg.max_centripetal_acceleration  = 0.4
cfg.output_callback               = on_output
cfg.planning_period               = 0.02
cfg.planning_task_config          = espp.Task.BaseConfig("tp_py_plan")
cfg.callback_task_config          = espp.Task.BaseConfig("tp_py_cb")
cfg.log_level                     = espp.Logger.Verbosity.warn

# ── run scenarios ─────────────────────────────────────────────────────────────
planner = espp.TrajectoryPlanner(cfg)

mark("Forward ramp (+1, 0)")
planner.set_target(1.0, 0.0)
time.sleep(1.5)

mark("Right curve (0.5, -0.4)")
planner.set_target(0.5, -0.4)
time.sleep(1.5)

mark("Combined (0.8, 0.6)")
planner.set_target(0.8, 0.6)
time.sleep(1.5)

mark("Stop")
planner.stop()
time.sleep(0.8)

mark("Reverse (-0.6, 0.0)")
planner.set_target(-0.6, 0.0)
time.sleep(1.5)

mark("Stop")
planner.stop()
time.sleep(0.8)
mark("Reset")
planner.reset()
time.sleep(0.1)
cfg.output_callback = None  # stop collecting data
planner.set_config(cfg)  # apply new config without callback
time.sleep(0.1)  # wait for planner to process config change



# ── plot ──────────────────────────────────────────────────────────────────────
fig, axes = plt.subplots(4, 1, figsize=(12, 10), sharex=True)
fig.suptitle("TrajectoryPlanner output")

datasets = [
    (axes[0], [(v_hist, "v (m/s)", "C0"),   (w_hist,   "w (rad/s)", "C1")],
     "Velocity",     [cfg.max_linear_velocity, -cfg.max_linear_velocity,
                      cfg.max_angular_velocity, -cfg.max_angular_velocity]),
    (axes[1], [(a_v_hist, "a_v (m/s^2)", "C2"), (a_w_hist, "a_w (rad/s^2)", "C3")],
     "Acceleration",  [driving.max_linear_acceleration,  -driving.max_linear_acceleration,
                       driving.max_angular_acceleration, -driving.max_angular_acceleration,
                       stopping.max_linear_acceleration, -stopping.max_linear_acceleration,
                       stopping.max_angular_acceleration, -stopping.max_angular_acceleration]),
    (axes[2], [(j_v_hist, "j_v (m/s^3)", "C4"), (j_w_hist, "j_w (rad/s^3)", "C5")],
     "Jerk",          [driving.max_linear_jerk,  -driving.max_linear_jerk,
                       driving.max_angular_jerk, -driving.max_angular_jerk,
                       stopping.max_linear_jerk, -stopping.max_linear_jerk,
                       stopping.max_angular_jerk, -stopping.max_angular_jerk]),
    (axes[3], [([v * w for v, w in zip(v_hist, w_hist)], "|v*w| centripetal", "C6")],
     "Centripetal (m/s^2)", [cfg.max_centripetal_acceleration, -cfg.max_centripetal_acceleration]),
]

for ax, series, ylabel, limits in datasets:
    for data, label, color in series:
        ax.plot(t_hist, data, label=label, color=color, linewidth=1.2)
    # draw limit lines
    for lim in limits:
        ax.axhline(lim, linestyle="--", linewidth=0.7, color="gray", alpha=0.6)
    # scenario markers
    for t_ev, lbl in ev_hist:
        ax.axvline(t_ev, linestyle=":", linewidth=0.8, color="black", alpha=0.5)
    ax.set_ylabel(ylabel)
    ax.legend(loc="upper right", fontsize=8)
    ax.grid(True, alpha=0.3)

# shared event labels on top axis
for t_ev, lbl in ev_hist:
    axes[0].text(t_ev, axes[0].get_ylim()[1], lbl, rotation=45,
                 fontsize=7, va="top", ha="right", alpha=0.7)
axes[3].set_xlabel("Time (s)")

plt.tight_layout()
plt.show()
sys.exit(0)


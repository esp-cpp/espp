"""Blocking-destructor test for the espp python bindings.

Objects whose C++ destructor joins a thread (Task, Timer, TrajectoryPlanner,
ThreadPool) used to deadlock when deleted from python while their thread was
waiting for the GIL to run its python callback, and to hang or crash the
interpreter when still alive at exit. Their pybind11 holder now releases the
GIL around the delete and the module's atexit hook stops them before
finalisation (lib/python_bindings/stop_on_exit_holder.h).

Each case runs in a child interpreter under a timeout, so a regression shows
up as a failed case rather than a hung test.

Exit code 0 on full pass, 1 on any failure.
"""

import os
import subprocess
import sys
import time

TIMEOUT_S = 15.0

CASES = {
    # delete a running Task whose callback is python: must return promptly
    "task_del": """
import time, espp
calls = [0]
def cb():
    calls[0] += 1
    time.sleep(0.005)
    return False  # keep running
t = espp.Task(cb, espp.Task.BaseConfig("py_task"))
t.start()
time.sleep(0.2)
t0 = time.time(); del t; dt = time.time() - t0
assert calls[0] > 5, calls
assert dt < 2.0, dt
print("task_del ok", calls[0], round(dt, 3))
""",
    # the same through a Timer (its own task + a python callback)
    "timer_del": """
import time, espp
calls = [0]
def cb():
    calls[0] += 1
    time.sleep(0.003)
    return False
t = espp.Timer(espp.Timer.Config("py_timer", 0.005, 0.0, cb, True))
time.sleep(0.2)
t0 = time.time(); del t; dt = time.time() - t0
assert calls[0] > 5, calls
assert dt < 2.0, dt
print("timer_del ok", calls[0], round(dt, 3))
""",
    # a running TrajectoryPlanner with a python output callback
    "planner_del": """
import time, espp
outputs = [0]
def on_output(cmd):
    outputs[0] += 1
    time.sleep(0.003)
cfg = espp.TrajectoryPlanner.Config()
cfg.driving_profile.max_linear_acceleration = 1.0
cfg.driving_profile.max_angular_acceleration = 1.0
cfg.stopping_profile.max_linear_acceleration = 1.0
cfg.stopping_profile.max_angular_acceleration = 1.0
cfg.output_callback = on_output
cfg.planning_period = 0.005
cfg.planning_task_config = espp.Task.BaseConfig("tp_plan")
cfg.callback_task_config = espp.Task.BaseConfig("tp_cb")
p = espp.TrajectoryPlanner(cfg)
p.set_target(1.0, 0.0)
time.sleep(0.2)
t0 = time.time(); del p; dt = time.time() - t0
assert outputs[0] > 5, outputs
assert dt < 2.0, dt
print("planner_del ok", outputs[0], round(dt, 3))
""",
    # leave running objects alive at interpreter exit: the process must end cleanly
    "exit_with_live_objects": """
import time, espp
def cb():
    time.sleep(0.002)
    return False
task = espp.Task(cb, espp.Task.BaseConfig("py_task")); task.start()
timer = espp.Timer(espp.Timer.Config("py_timer", 0.005, 0.0, cb, True))
outputs = [0]
def on_output(cmd):
    outputs[0] += 1
    time.sleep(0.003)
cfg = espp.TrajectoryPlanner.Config()
cfg.driving_profile.max_linear_acceleration = 1.0
cfg.driving_profile.max_angular_acceleration = 1.0
cfg.stopping_profile.max_linear_acceleration = 1.0
cfg.stopping_profile.max_angular_acceleration = 1.0
cfg.output_callback = on_output
cfg.planning_period = 0.005
cfg.planning_task_config = espp.Task.BaseConfig("tp_plan")
cfg.callback_task_config = espp.Task.BaseConfig("tp_cb")
planner = espp.TrajectoryPlanner(cfg); planner.set_target(1.0, 0.0)
time.sleep(0.2)
print("exit_with_live_objects: leaving", outputs[0] > 0)
# no stop, no del: the interpreter exits with all three running
""",
    # an exception with a live planner must not turn into a hang or a crash either
    "exception_with_live_planner": """
import time, espp
def on_output(cmd):
    time.sleep(0.003)
cfg = espp.TrajectoryPlanner.Config()
cfg.driving_profile.max_linear_acceleration = 1.0
cfg.driving_profile.max_angular_acceleration = 1.0
cfg.stopping_profile.max_linear_acceleration = 1.0
cfg.stopping_profile.max_angular_acceleration = 1.0
cfg.output_callback = on_output
cfg.planning_period = 0.005
cfg.planning_task_config = espp.Task.BaseConfig("tp_plan")
cfg.callback_task_config = espp.Task.BaseConfig("tp_cb")
planner = espp.TrajectoryPlanner(cfg); planner.set_target(1.0, 0.0)
time.sleep(0.1)
raise RuntimeError("expected: unhandled exception with a live planner")
""",
}
EXPECTED_EXIT = {"exception_with_live_planner": 1}


def run_case(name: str, code: str) -> bool:
    t0 = time.time()
    try:
        proc = subprocess.run([sys.executable, "-u", "-c", code], capture_output=True, text=True,
                              timeout=TIMEOUT_S, env=os.environ.copy())
    except subprocess.TimeoutExpired as e:
        print(f"FAIL {name}: timed out after {TIMEOUT_S:.0f} s (hang)\n{(e.stdout or b'')}")
        return False
    dt = time.time() - t0
    expected = EXPECTED_EXIT.get(name, 0)
    if proc.returncode != expected:
        sig = f" (signal {-proc.returncode})" if proc.returncode < 0 else ""
        print(f"FAIL {name}: exit {proc.returncode}{sig}, expected {expected}\n{proc.stdout}{proc.stderr}")
        return False
    print(f"PASS {name} ({dt:.2f} s): {proc.stdout.strip().splitlines()[-1] if proc.stdout.strip() else ''}")
    return True


def main() -> int:
    results = [run_case(name, code) for name, code in CASES.items()]
    passed = sum(results)
    print(f"{passed}/{len(results)} cases passed")
    return 0 if all(results) else 1


if __name__ == "__main__":
    sys.exit(main())

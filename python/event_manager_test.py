"""EventManager test for the espp python bindings.

Covers the pub/sub API and its misuse, plus the cases that need the GIL
handled right: remove_subscriber() / clear() wait for a callback running on
the topic's thread (so they release the GIL, or a python callback could never
finish), a callback that removes its own subscription, and exiting the
interpreter with live subscribers (the module's atexit hook clears the
EventManager while python can still run and destroy the callbacks).

Each case runs in a child interpreter under a timeout, so a regression shows
up as a failed case rather than a hung test.

Exit code 0 on full pass, 1 on any failure.
"""

import os
import subprocess
import sys
import time

TIMEOUT_S = 15.0

# Shared by the cases: wait for a condition without hanging the case.
PRELUDE = """
import threading, time, espp
em = espp.EventManager.get()
def wait_for(pred, timeout=5.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pred():
            return True
        time.sleep(0.001)
    return pred()
"""

CASES = {
    "basics": """
assert em.publish("t", [1, 2, 3]) is False  # no subscribers yet
got = []
assert em.add_subscriber("t", "a", lambda data: got.append(("a", list(data)))) is True
assert em.add_subscriber("t", "a", lambda data: None) is False  # already subscribed
assert em.add_subscriber("t", "none", None) is False  # empty callback
cfg = espp.Task.BaseConfig("t subscriber", stack_size_bytes=8192)
assert em.add_subscriber("t", "b", lambda data: got.append(("b", list(data))), cfg) is True
assert em.add_subscriber("u", "c", lambda data: got.append(("c", list(data))), stack_size_bytes=8192)
assert em.publish("t", [1, 2, 3]) is True
assert em.publish("t", [9]) is True
assert em.publish("u", []) is True
assert wait_for(lambda: len(got) == 5), got
t_events = [g for g in got if g[0] != "c"]
assert t_events == [("a", [1, 2, 3]), ("b", [1, 2, 3]), ("a", [9]), ("b", [9])], t_events
assert ("c", []) in got
assert em.remove_subscriber("t", "nobody") is False
assert em.remove_subscriber("nowhere", "a") is False
assert em.add_publisher("t", "p") is True
assert em.add_publisher("t", "p") is False
assert em.remove_publisher("t", "p") is True
assert em.remove_publisher("t", "p") is False
assert em.remove_subscriber("t", "a") is True
assert em.remove_subscriber("t", "b") is True
assert em.publish("t", [1]) is False  # last subscriber gone
assert em.remove_subscriber("u", "c") is True
print("basics ok")
""",
    "misuse": """
def raises_type_error(fn):
    try:
        fn()
    except TypeError:
        return True
    return False
assert raises_type_error(lambda: em.publish("t", "not a list of ints"))
assert raises_type_error(lambda: em.publish("t", [1, 2, 300]))  # not a byte
assert raises_type_error(lambda: em.publish("t"))
assert raises_type_error(lambda: em.add_subscriber("t", "c", 5))  # not callable
assert raises_type_error(lambda: em.add_subscriber("t", "c"))
assert raises_type_error(lambda: em.remove_subscriber(1, 2))
assert raises_type_error(lambda: espp.EventManager())  # singleton: get() only
assert em is espp.EventManager.get()
# a callback that raises: the error is reported by python, the subscriber thread keeps going
calls = [0]
def flaky(data):
    calls[0] += 1
    if calls[0] == 1:
        raise ValueError("expected: first delivery raises")
assert em.add_subscriber("t", "flaky", flaky)
em.publish("t", [1])
em.publish("t", [2])
assert wait_for(lambda: calls[0] == 2), calls
print("misuse ok")
""",
    # remove_subscriber() from the main thread while the python callback runs: it waits for
    # the callback, which needs the GIL to finish, so the binding must release the GIL.
    "remove_while_callback_runs": """
started = threading.Event(); state = {"done": False, "calls": 0}
def slow(data):
    state["calls"] += 1
    started.set()
    time.sleep(0.3)
    state["done"] = sum(range(1000)) > 0  # python work after the sleep needs the GIL
assert em.add_subscriber("t", "slow", slow)
assert em.add_subscriber("t", "other", lambda data: None)
em.publish("t", [1])
assert started.wait(5)
t0 = time.time()
assert em.remove_subscriber("t", "slow") is True
dt = time.time() - t0
assert state["done"], "remove_subscriber returned while the callback was still running"
em.publish("t", [2])
time.sleep(0.1)
assert state["calls"] == 1, state
print("remove_while_callback_runs ok", round(dt, 3))
""",
    "remove_last_while_callback_runs": """
started = threading.Event(); state = {"done": False}
def slow(data):
    started.set()
    time.sleep(0.3)
    state["done"] = True
assert em.add_subscriber("t", "slow", slow)
em.publish("t", [1])
assert started.wait(5)
assert em.remove_subscriber("t", "slow") is True
assert state["done"]
assert em.publish("t", [2]) is False
print("remove_last_while_callback_runs ok")
""",
    "clear_while_callback_runs": """
started = threading.Event(); state = {"done": False}
def slow(data):
    started.set()
    time.sleep(0.3)
    state["done"] = True
assert em.add_subscriber("t", "slow", slow)
assert em.add_subscriber("u", "idle", lambda data: None)
em.add_publisher("t", "p")
em.publish("t", [1])
assert started.wait(5)
em.clear()
assert state["done"]
assert em.publish("t", [1]) is False and em.publish("u", [1]) is False
assert em.remove_publisher("t", "p") is False
got = []
assert em.add_subscriber("t", "again", lambda data: got.append(list(data)))
em.publish("t", [7])
assert wait_for(lambda: got == [[7]]), got
print("clear_while_callback_runs ok")
""",
    "callback_removes_itself": """
state = {"calls": 0, "removed": None}
def once(data):
    state["calls"] += 1
    state["removed"] = em.remove_subscriber("t", "once")
assert em.add_subscriber("t", "once", once)
em.publish("t", [1])
assert wait_for(lambda: state["removed"] is not None)
assert state["removed"] is True
assert em.publish("t", [2]) is False
assert state["calls"] == 1
print("callback_removes_itself ok")
""",
    # leave subscribers registered at exit, one of them idle and one mid-callback
    "exit_with_live_subscribers": """
started = threading.Event()
def slow(data):
    started.set()
    time.sleep(0.2)
assert em.add_subscriber("idle", "i", lambda data: None)
assert em.add_subscriber("busy", "b", slow)
em.publish("idle", [1])
em.publish("busy", [1])
assert started.wait(5)
print("exit_with_live_subscribers ok")
""",
    # an unhandled exception with live subscribers must still exit (with status 1)
    "exception_with_live_subscribers": """
assert em.add_subscriber("t", "s", lambda data: time.sleep(0.01))
for i in range(10):
    em.publish("t", [i])
raise RuntimeError("expected: unhandled exception with live subscribers")
""",
}
EXPECTED_EXIT = {"exception_with_live_subscribers": 1}


def run_case(name: str, code: str) -> bool:
    t0 = time.time()
    try:
        proc = subprocess.run([sys.executable, "-u", "-c", PRELUDE + code], capture_output=True,
                              text=True, timeout=TIMEOUT_S, env=os.environ.copy())
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

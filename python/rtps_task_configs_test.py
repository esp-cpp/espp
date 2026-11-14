#!/usr/bin/env python3
"""The participant's task configuration and the action server's execute workers,
from Python. Two participants in one process (discovery on the local interface):

 1. a participant built with every task configured (stack sizes, priorities,
    names) starts and stops;
 2. an action server with execute_workers=2 and max_goals_in_flight=4 runs
    two blocking goals at once and parks the other two (the C++ test
    pc/tests/rtps_action_worker_pool.cpp, through the binding);
 3. the native server takes the same settings.

Exits 0 on success, 1 on the first failed check (the participants are stopped
on every exit path: torn down at interpreter exit instead, one would join its
execute workers with the GIL held while the Python callback waits for it).
Needs a working loopback / local-interface multicast like the other python
rtps scripts; the dockerised interop harness runs it after rtps_rpc_demo.py.
The multicast-free checks of the same interface (construction, validation,
misuse) are python/rtps_bindings_misuse_test.py, which the wheel build runs.
"""

import sys
import threading
import time

import espp

ACTION = "/py_pool"
TYPE = "espp_test::action::dds_::PyPool"
EMPTY = bytes([0, 1, 0, 0, 0])  # an empty ROS 2 message, CDR-encapsulated


def check(cond, msg):
    if not cond:
        print(f"FAIL: {msg}", file=sys.stderr)
        sys.exit(1)


def wait_until(pred, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.005)
    return True


def main():
    task = espp.Task.BaseConfig
    # 1. every task configured
    server = espp.RtpsParticipant(
        espp.RtpsParticipant.Config(
            log_level=espp.Logger.Verbosity.error,
            receive_workers=2,
            receive_worker_task=task(name="py_rx", stack_size_bytes=8192, priority=5),
            reactor_task=task(name="py_reactor", stack_size_bytes=8192),
            job_retry_task=task(name="py_retry", stack_size_bytes=4096),
            protocol_task=task(name="py_proto", stack_size_bytes=6144),
            reply_hold_task=task(name="py_hold", stack_size_bytes=4096),
        )
    )
    client = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=espp.Logger.Verbosity.error))
    check(server.start() and client.start(), "start")
    try:
        return run_goals(server, client, task)
    finally:
        # Stopped on EVERY exit: a participant torn down at interpreter exit
        # joins its execute workers with the GIL held, while the Python execute
        # callback needs the GIL to see its cancellation - a hang, not a FAIL.
        client.stop()
        server.stop()


def run_goals(server, client, task):

    # 2. two workers, four goals
    executing = 0
    peak = 0
    finished = 0
    lock = threading.Lock()

    def execute(handle):
        nonlocal executing, peak, finished
        with lock:
            executing += 1
            peak = max(peak, executing)
        while not handle.is_canceling():
            time.sleep(0.005)
        with lock:
            executing -= 1
        handle.canceled(EMPTY)
        with lock:
            finished += 1

    check(
        server.add_action_server(
            ACTION, TYPE, lambda goal: True, execute,
            max_goals_in_flight=4, execute_workers=2,
            execute_task=task(name="py_exec", stack_size_bytes=8192),
        ),
        "add_action_server",
    )
    act = client.add_action_client(ACTION, TYPE)
    check(act is not None and act.wait_for_server(timeout=10.0), "wait_for_server")
    time.sleep(0.3)  # feedback/status topics match alongside

    results = []
    goal_ids = []
    for _ in range(4):
        gid = act.send_goal(EMPTY, lambda fb: None, lambda status, result: results.append(status))
        check(gid is not None, "send_goal accepted")
        goal_ids.append(gid)
    check(wait_until(lambda: executing == 2 and peak == 2, 2.0), "two goals execute")
    # the invariant, watched over 300 ms: two run, never a third
    until = time.monotonic() + 0.3
    while time.monotonic() < until:
        with lock:
            check(executing == 2 and peak == 2, "no more than two goals execute at once")
        time.sleep(0.005)
    for gid in goal_ids:
        check(act.cancel_goal(gid), "cancel_goal sent")
    check(wait_until(lambda: finished == 4, 5.0), "every goal ran and finished")
    check(peak == 2, "still no more than two at once")
    check(wait_until(lambda: len(results) == 4, 5.0), "every result arrived")
    check(all(status == 5 for status in results), "every result is CANCELED (5)")

    # 3. the native server takes the same settings
    check(
        server.add_native_action_server(
            "/py_pool_native", TYPE, lambda goal: True, lambda h: h.succeed(EMPTY),
            None, max_goals_in_flight=2, execute_workers=1,
            execute_task=task(name="py_nexec", stack_size_bytes=8192),
        ),
        "add_native_action_server",
    )

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

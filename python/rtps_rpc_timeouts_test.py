#!/usr/bin/env python3
"""The RPC timeout knobs of the RTPS binding, observed from Python. Two
participants in one process (discovery on the local interface), a server that
never answers in time and a client with a short request_timeout, then a fresh
pair (the first pair's writer / reader capacity is spent by then):

 1. ROS 2 service client, request_timeout=0.3 against a handler that sleeps
    2 s: call_future() resolves to None and pending_count() returns to 0 (the
    expiry is enforced when the pending table is touched - a send or a count -
    so the test counts); call_async() gets on_abandon() and never on_reply();
 2. the native service client does the same, with the same kwargs;
 3. ROS 2 action client, request_timeout=0.3 against an on_goal that blocks
    2 s: the goal ends with on_result(6 ABORTED, b'') once the client's pending
    table is touched again (the next send_goal here); the second goal, still
    unanswered when the participant stops, ends the same way;
 4. result_timeout=0 and result_timeout=0.2 servers complete a goal normally
    (on_result(4 SUCCEEDED)) with an unknown_goal_response set; a late
    get_result is not reachable from Python, so retirement itself is only
    covered by the C++ loopback tests;
 5. stop() abandons a still-pending goal (on_result(ABORTED) once).

Exits 0 on success, 1 on the first failed check; ~15 s. Needs a working
loopback / local-interface multicast like the other python rtps scripts.
"""

import sys
import threading
import time

import espp

TYPE_SRV = "example_interfaces::srv::dds_::AddTwoInts"
TYPE_ACT = "espp_test::action::dds_::Timeouts"
EMPTY = bytes([0, 1, 0, 0, 0])  # an empty ROS 2 message, CDR-encapsulated
ABORTED = 6
SUCCEEDED = 4


def check(cond, msg):
    if not cond:
        print(f"FAIL: {msg}", file=sys.stderr)
        sys.exit(1)


def wait_until(pred, timeout_s):
    deadline = time.monotonic() + timeout_s
    while not pred():
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.01)
    return True


def slow_handler(req):
    time.sleep(2.0)
    return EMPTY


def main():
    log = espp.Logger.Verbosity.error
    # the server blocks a receive worker per slow request: give it spares
    server = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=log, receive_workers=4))
    client = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=log))
    check(server.start() and client.start(), "start")
    try:
        # 1. ROS 2 service: the request is abandoned past request_timeout
        check(server.add_service_server("/slow", TYPE_SRV, slow_handler), "add_service_server")
        svc = client.add_service_client("/slow", TYPE_SRV, request_timeout=0.3, max_pending_requests=4)
        check(svc is not None and svc.wait_for_server(10.0), "service wait_for_server")
        t0 = time.monotonic()
        fut = svc.call_future(EMPTY)
        abandoned = []
        replied = []
        check(svc.call_async(EMPTY, lambda r: replied.append(r), lambda: abandoned.append(True)),
              "call_async queued")
        check(svc.pending_count() == 2, "two requests pending")
        check(wait_until(lambda: svc.pending_count() == 0, 2.0), "the pending table empties past request_timeout")
        elapsed = time.monotonic() - t0
        check(0.3 <= elapsed < 1.5, f"abandoned after about request_timeout, not {elapsed:.2f} s")
        check(fut.result(timeout=1.0) is None, "call_future() resolved to None on abandonment")
        check(abandoned == [True] and replied == [], "call_async: on_abandon once, on_reply never")
        time.sleep(2.0)  # the late reply arrives; nothing may run on it
        check(abandoned == [True] and replied == [], "a late reply does not run on_reply")
        # a blocking call() honours its own timeout
        t0 = time.monotonic()
        check(svc.call(EMPTY, timeout=0.2) is None, "call() with a short timeout returns None")
        check(time.monotonic() - t0 < 1.0, "call() returned at its timeout")
        check(wait_until(lambda: svc.pending_count() == 0, 2.0), "the pending table is empty again")

        # 2. the native service client does the same
        check(server.add_native_service_server("/nslow", TYPE_SRV, slow_handler), "add_native_service_server")
        nsvc = client.add_native_service_client("/nslow", TYPE_SRV, request_timeout=0.3, max_pending_requests=4)
        check(nsvc is not None, "add_native_service_client")
        time.sleep(1.0)  # native services match through discovery only
        nabandoned = []
        nreplied = []
        nfut = nsvc.call_future(EMPTY)
        check(nsvc.call_async(EMPTY, lambda r: nreplied.append(r), lambda: nabandoned.append(True)),
              "native call_async queued")
        check(nsvc.pending_count() == 2, "two native requests pending")
        check(wait_until(lambda: nsvc.pending_count() == 0, 2.0), "the native pending table empties")
        check(nfut.result(timeout=1.0) is None, "native call_future() resolved to None")
        check(nabandoned == [True] and nreplied == [], "native call_async: on_abandon once, on_reply never")

        # 3. ROS 2 action client: an unanswered send_goal ends the goal ABORTED
        def slow_on_goal(goal):
            time.sleep(2.0)
            return False  # too late anyway

        check(server.add_action_server("/slow_act", TYPE_ACT, slow_on_goal, lambda h: None),
              "add_action_server (slow on_goal)")
        act = client.add_action_client("/slow_act", TYPE_ACT, request_timeout=0.3)
        check(act is not None and act.wait_for_server(10.0), "action wait_for_server")
        time.sleep(0.3)  # feedback/status topics match alongside
        results = []
        lock = threading.Lock()

        def on_result(status, result):
            with lock:
                results.append((status, result))

        gid1 = act.send_goal(EMPTY, lambda fb: None, on_result)
        check(gid1 is not None, "first goal sent")
        time.sleep(0.6)  # past request_timeout; the expiry runs on the next send
        check(results == [], "nothing is reported until the pending table is touched")
        gid2 = act.send_goal(EMPTY, lambda fb: None, on_result)
        check(gid2 is not None, "second goal sent (it expires the first)")
        check(wait_until(lambda: len(results) == 1, 2.0), "the first goal ended once its request expired")
        with lock:
            check(results[0] == (ABORTED, b""), f"an abandoned send_goal reports ABORTED with no result: {results}")

        # 4. a result_timeout=0 server completes a goal normally (this pair's
        # last endpoints: a ROS 2 action server is 5 writers, a client 5 readers,
        # and the host profile allows 13 of each per participant)
        quick_goal(server, client, "/quick0", 0.0)
    finally:
        client.stop()  # abandons the second slow goal: on_result(ABORTED) once more
        server.stop()
    check(wait_until(lambda: len(results) == 2, 2.0) and results[1] == (ABORTED, b""),
          f"stop() abandons the still-pending goal as ABORTED: {results}")

    # 4b. and a result_timeout=0.2 one, on a fresh pair (two participants at a
    # time: more than that on one host contend for the user unicast ports)
    server = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=log))
    client = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=log))
    check(server.start() and client.start(), "start (second pair)")
    try:
        quick_goal(server, client, "/quick1", 0.2)
    finally:
        client.stop()
        server.stop()
    print("PASS")
    return 0


def quick_goal(server, client, name, result_timeout):
    check(server.add_action_server(name, TYPE_ACT, lambda g: True, lambda h: h.succeed(EMPTY),
                                   result_timeout=result_timeout, unknown_goal_response=EMPTY),
          f"add_action_server({name}, result_timeout={result_timeout})")
    qact = client.add_action_client(name, TYPE_ACT)
    check(qact is not None and qact.wait_for_server(10.0), f"{name} wait_for_server")
    time.sleep(0.3)  # feedback/status topics match alongside
    done = []
    check(qact.send_goal(EMPTY, lambda fb: None, lambda s, r: done.append(s)) is not None, f"{name} goal")
    check(wait_until(lambda: len(done) == 1, 5.0) and done[0] == SUCCEEDED,
          f"{name} (result_timeout={result_timeout}) goal SUCCEEDED: {done}")


if __name__ == "__main__":
    sys.exit(main())

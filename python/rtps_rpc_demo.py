#!/usr/bin/env python3
"""Showcase + self-test of the espp RTPS RMI/AMI APIs from Python.

Demonstrates, in one process (two participants), every request/reply mechanism
the espp RtpsParticipant exposes, using the typed ``espp.rtps`` wrappers and
pycdr2 message schemas - so you work with message objects, never CDR bytes:

  1. ROS 2-interoperable service (RMI)   - espp.rtps.ServiceServer / ServiceClient
  2. ROS 2-interoperable action  (AMI)   - espp.rtps.ActionServer  / ActionClient
  3. Native (espp<->espp) service (RMI)   - the same wrappers with native=True
  4. Native (espp<->espp) action  (AMI)   - the same wrappers with native=True

Message types are defined with pycdr2's ``make_idl_struct`` (the functional form
works on every Python, incl. 3.14; the ``@dataclass`` form does not). pycdr2 emits
ROS 2 / classic-CDR, so the AddTwoInts / Fibonacci schemas below match
example_interfaces on the wire.

Requires pycdr2 (see python/requirements.txt).
Run:  python/env/bin/python python/rtps_rpc_demo.py
Exit 0 iff every mechanism round-trips correctly.
"""

import sys
import threading
import time

import espp
import espp.rtps as rtps
from pycdr2 import make_idl_struct
from pycdr2.types import int32, int64, sequence

# --- pycdr2 message schemas (fields map straight to CDR / the ROS 2 wire) -----
# ROS 2 example_interfaces/srv/AddTwoInts.
AddReq = make_idl_struct("AddTwoInts_Request", "example_interfaces/srv/AddTwoInts_Request",
                         {"a": int64, "b": int64})
AddResp = make_idl_struct("AddTwoInts_Response", "example_interfaces/srv/AddTwoInts_Response",
                          {"sum": int64})
# ROS 2 example_interfaces/action/Fibonacci (Result + Feedback share the shape).
FibGoal = make_idl_struct("Fibonacci_Goal", "example_interfaces/action/Fibonacci_Goal",
                          {"order": int32})
FibSeq = make_idl_struct("Fibonacci_Seq", "example_interfaces/action/Fibonacci_Seq",
                         {"sequence": sequence[int32]})
# Native (espp<->espp) demo types - any consistent schema/name works.
MulResp = make_idl_struct("Mul_Response", "espp_examples/srv/Mul_Response", {"product": int64})
CountGoal = make_idl_struct("CountUp_Goal", "espp_examples/action/CountUp_Goal", {"n": int32})
CountVal = make_idl_struct("CountUp_Value", "espp_examples/action/CountUp_Value", {"value": int32})


def main():
    server = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=espp.Logger.Verbosity.warn))
    client = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=espp.Logger.Verbosity.warn))
    # Explicit check (not assert: asserts are skipped under `python -O`).
    if not server.start() or not client.start():
        print("FAIL: participants failed to start", file=sys.stderr)
        server.stop()
        client.stop()
        return 1

    results = {}

    # -- 1. ROS 2 service (RMI): sum = a + b --------------------------------
    rtps.ServiceServer(server, "/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts",
                       AddReq, AddResp, lambda req: AddResp(sum=req.a + req.b))
    svc = rtps.ServiceClient(client, "/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts",
                             AddReq, AddResp)

    # -- 2. ROS 2 action (AMI): Fibonacci -----------------------------------
    def fib_execute(handle):
        seq = [0, 1]
        for i in range(1, handle.goal.order):
            seq.append(seq[i] + seq[i - 1])
            handle.publish_feedback(FibSeq(sequence=seq))
            time.sleep(0.05)
        handle.succeed(FibSeq(sequence=seq))

    rtps.ActionServer(server, "/fibonacci", "example_interfaces::action::dds_::Fibonacci",
                      FibGoal, FibSeq, FibSeq, lambda goal: goal.order > 0, fib_execute)
    act = rtps.ActionClient(client, "/fibonacci", "example_interfaces::action::dds_::Fibonacci",
                            FibGoal, FibSeq, FibSeq)

    # -- 3. Native service (RMI): product = a * b ---------------------------
    rtps.ServiceServer(server, "/mul", "espp::native::Mul", AddReq, MulResp,
                       lambda req: MulResp(product=req.a * req.b), native=True)
    nsvc = rtps.ServiceClient(client, "/mul", "espp::native::Mul", AddReq, MulResp, native=True)

    # -- 4. Native action (AMI): count up to n ------------------------------
    def count_execute(handle):
        for i in range(1, handle.goal.n + 1):
            if handle.is_canceling():  # cooperative cancel (native protocol too)
                handle.canceled(CountVal(value=i - 1))
                return
            handle.publish_feedback(CountVal(value=i))
            time.sleep(0.03)
        handle.succeed(CountVal(value=handle.goal.n))

    rtps.ActionServer(server, "/countup", "espp::native::CountUp", CountGoal, CountVal, CountVal,
                      lambda goal: goal.n > 0, count_execute, native=True)
    nact = rtps.ActionClient(client, "/countup", "espp::native::CountUp", CountGoal, CountVal,
                             CountVal, native=True)

    time.sleep(2.0)  # SEDP discovery for every endpoint

    # === 1. Service: blocking, async, and future (the three call styles) ===
    reply = svc.call(AddReq(a=7, b=35), timeout=5.0)
    results["ros_service_sync"] = reply is not None and reply.sum == 42

    ev = threading.Event()
    got = {}
    svc.call_async(AddReq(a=100, b=23), lambda r: (got.update(v=r.sum), ev.set()))
    results["ros_service_async"] = ev.wait(5.0) and got.get("v") == 123

    fut = svc.call_future(AddReq(a=40, b=2))
    fut_reply = fut.result(timeout=5.0)
    results["ros_service_future"] = fut_reply is not None and fut_reply.sum == 42

    # === 2. Action: feedback + result ===
    done = threading.Event()
    fib = {"fb": 0, "status": 0, "seq": None}
    act.send_goal(
        FibGoal(order=5),
        lambda f: fib.__setitem__("fb", fib["fb"] + 1),
        lambda status, res: (fib.update(status=status, seq=res.sequence if res else None), done.set()))
    ok = done.wait(10.0)
    results["ros_action"] = (ok and fib["status"] == 4 and fib["seq"] == [0, 1, 1, 2, 3, 5]
                             and fib["fb"] > 0)

    # === 3. Native service ===
    nreply = nsvc.call(AddReq(a=6, b=7), timeout=5.0)
    results["native_service"] = nreply is not None and nreply.product == 42

    # === 4. Native action ===
    ndone = threading.Event()
    cnt = {"fb": 0, "status": 0, "n": None}
    nact.send_goal(
        CountGoal(n=5),
        lambda f: cnt.__setitem__("fb", cnt["fb"] + 1),
        lambda status, res: (cnt.update(status=status, n=res.value if res else None), ndone.set()))
    nok = ndone.wait(10.0)
    results["native_action"] = nok and cnt["status"] == 4 and cnt["n"] == 5 and cnt["fb"] > 0

    # === 5. Native action cancel: cancel a goal mid-flight ===
    # n is bounded (~6s if never canceled) so a rare lost cancel can't wall-clock
    # block stop()'s join of the execute thread. Cancel triggers off the FIRST
    # feedback (goal is accepted + executing, so cancel_goal() has its handle)
    # rather than a fixed sleep - deterministic under CI load.
    ncdone = threading.Event()
    nexecuting = threading.Event()
    ncancel = {"status": 0}
    nact.send_goal(
        CountGoal(n=200),
        lambda f: nexecuting.set(),
        lambda status, res: (ncancel.update(status=status), ncdone.set()))
    nexecuting.wait(5.0)  # goal is executing (and accepted) -> cancel is effective
    nact.cancel_goal()
    results["native_cancel"] = ncdone.wait(10.0) and ncancel["status"] == 5  # CANCELED

    server.stop()
    client.stop()

    print("\n=== espp RTPS RMI/AMI demo results ===")
    all_ok = True
    for name, passed in results.items():
        print(f"  {'PASS' if passed else 'FAIL'}  {name}")
        all_ok = all_ok and passed
    print("=== %s ===" % ("ALL PASS" if all_ok else "FAILURES"))
    return 0 if all_ok else 1


if __name__ == "__main__":
    sys.exit(main())

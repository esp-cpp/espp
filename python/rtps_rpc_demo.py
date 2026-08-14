#!/usr/bin/env python3
"""Showcase + self-test of the espp RTPS RMI/AMI APIs from Python.

Demonstrates, in one process (two participants), every request/reply mechanism
the espp RtpsParticipant exposes:

  1. ROS 2-interoperable service (RMI)   - add_service_server / add_service_client
  2. ROS 2-interoperable action  (AMI)   - add_action_server  / add_action_client
  3. Native (espp<->espp) service (RMI)  - add_native_service_*  (lean, in-band header)
  4. Native (espp<->espp) action  (AMI)  - add_native_action_*   (lean, ~3 endpoints)

Payloads are CDR-encapsulated bytes: a 4-byte little-endian encapsulation header
{00 01 00 00} followed by the CDR body. We hand-pack simple int fields with
struct; for real ROS 2 types use the `cdr` component or pycdr2 (see rtps_messages.py).

Run:  python/env/bin/python python/rtps_rpc_demo.py
Exit code 0 iff every mechanism round-trips correctly.
"""

import struct
import sys
import threading
import time

import espp

ENCAP = b"\x00\x01\x00\x00"  # CDR_LE encapsulation header


# --- CDR helpers for the demo message shapes -------------------------------
def enc_two_ints(a, b):  # example_interfaces/srv/AddTwoInts Request
    return ENCAP + struct.pack("<qq", a, b)


def enc_int64(x):  # AddTwoInts Response / generic
    return ENCAP + struct.pack("<q", x)


def dec_int64(b, off=4):
    return struct.unpack_from("<q", b, off)[0]


def enc_int32(x):
    return ENCAP + struct.pack("<i", x)


def dec_int32(b, off=4):
    return struct.unpack_from("<i", b, off)[0]


def enc_fib_goal(order):  # Fibonacci Goal { order: int32 }
    return ENCAP + struct.pack("<i", order)


def enc_seq(values):  # { sequence: int32[] } - uint32 length + elements
    return ENCAP + struct.pack("<I", len(values)) + b"".join(struct.pack("<i", v) for v in values)


def dec_seq(b):
    n = struct.unpack_from("<I", b, 4)[0]
    return list(struct.unpack_from("<%di" % n, b, 8))


def main():
    server = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=espp.Logger.Verbosity.warn))
    client = espp.RtpsParticipant(espp.RtpsParticipant.Config(log_level=espp.Logger.Verbosity.warn))
    assert server.start() and client.start(), "participants failed to start"

    results = {}

    # -- 1. ROS 2 service (RMI) ---------------------------------------------
    server.add_service_server(
        "/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts",
        lambda req: enc_int64(dec_int64(req, 4) + dec_int64(req, 12)))
    svc = client.add_service_client("/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts")

    # -- 2. ROS 2 action (AMI): Fibonacci -----------------------------------
    def fib_execute(handle):
        order = dec_int32(handle.goal())
        seq = [0, 1]
        for i in range(1, order):
            seq.append(seq[i] + seq[i - 1])
            handle.publish_feedback(enc_seq(seq))
            time.sleep(0.05)
        handle.succeed(enc_seq(seq))

    server.add_action_server(
        "/fibonacci", "example_interfaces::action::dds_::Fibonacci",
        lambda goal: dec_int32(goal) > 0, fib_execute)
    act = client.add_action_client("/fibonacci", "example_interfaces::action::dds_::Fibonacci")

    # -- 3. Native service (RMI) --------------------------------------------
    server.add_native_service_server(
        "/mul", "espp::native::Mul",
        lambda req: enc_int64(dec_int64(req, 4) * dec_int64(req, 12)))
    nsvc = client.add_native_service_client("/mul", "espp::native::Mul")

    # -- 4. Native action (AMI): countup ------------------------------------
    def count_execute(handle):
        n = dec_int32(handle.goal())
        for i in range(1, n + 1):
            handle.publish_feedback(enc_int32(i))
            time.sleep(0.03)
        handle.succeed(enc_int32(n))

    server.add_native_action_server(
        "/countup", "espp::native::CountUp",
        lambda goal: dec_int32(goal) > 0, count_execute)
    nact = client.add_native_action_client("/countup", "espp::native::CountUp")

    time.sleep(2.0)  # SEDP discovery for every endpoint

    # === 1. Service: blocking + async ===
    reply = svc.call(enc_two_ints(7, 35), timeout=5.0)
    results["ros_service_sync"] = reply is not None and dec_int64(reply) == 42

    ev = threading.Event()
    got = {}
    svc.call_async(enc_two_ints(100, 23), lambda r: (got.update(v=dec_int64(r)), ev.set()))
    results["ros_service_async"] = ev.wait(5.0) and got.get("v") == 123

    # === 2. Action: feedback + result ===
    done = threading.Event()
    fib = {"fb": 0, "status": 0, "seq": None}
    act.send_goal(
        enc_fib_goal(5),  # Fibonacci(5) = [0, 1, 1, 2, 3, 5]
        lambda f: fib.__setitem__("fb", fib["fb"] + 1),
        lambda status, res: (fib.update(status=status, seq=dec_seq(res)), done.set()))
    ok = done.wait(15.0)
    results["ros_action"] = (ok and fib["status"] == 4 and fib["seq"] == [0, 1, 1, 2, 3, 5]
                             and fib["fb"] > 0)

    # === 3. Native service ===
    nreply = nsvc.call(enc_two_ints(6, 7), timeout=5.0)
    results["native_service"] = nreply is not None and dec_int64(nreply) == 42

    # === 4. Native action ===
    ndone = threading.Event()
    cnt = {"fb": 0, "status": 0, "n": None}
    nact.send_goal(
        enc_int32(5),
        lambda f: cnt.__setitem__("fb", cnt["fb"] + 1),
        lambda status, res: (cnt.update(status=status, n=dec_int32(res)), ndone.set()))
    nok = ndone.wait(10.0)
    results["native_action"] = nok and cnt["status"] == 4 and cnt["n"] == 5 and cnt["fb"] > 0

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

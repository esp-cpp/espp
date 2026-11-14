#!/usr/bin/env python3
"""Behaviour and misuse of the RTPS Python binding, without any network.

Run by every wheel build (pyproject.toml [tool.cibuildwheel] test-command), on
every platform the wheel targets: what the binding must do when it is used
wrongly or at the edges, which the multicast-dependent functional scripts
(python/rtps_rpc_demo.py, python/rtps_task_configs_test.py in the dockerised
interop harness) cannot cover everywhere. No participant is started here, so no
socket is opened and nothing depends on the machine's interfaces.

 1. Config construction: defaults, every keyword argument, attributes round-trip
    (the optional task configs read back by VALUE: a read-back object outlives
    the field being set to None - a reference into the optional would dangle),
    positional backward compatibility of the first arguments;
 2. validation: a negative or oversized integer where the binding takes a
    size_t / uint8 raises TypeError and leaves the Config untouched; a wrong
    type for a task config raises TypeError;
 3. a participant that was never started: add_writer / add_reader / publish /
    add_service_server / add_action_server return False (not an exception, no
    crash), the client factories return None, stop() is harmless, and the
    participant destructs cleanly; start() after stop() on an unstarted
    participant is not required to work here (it needs an interface);
 4. the RPC timeout knobs (request_timeout / max_pending_requests on both
    service clients and the ROS 2 action client, result_timeout /
    unknown_goal_response on the ROS 2 action server): every one is accepted
    (negative seconds clamp to 0 = never / retire at once, so they must not
    raise), a str where bytes are due and a wrong type are TypeErrors, and the
    knobs a protocol does not have (the native action server's result_timeout,
    the native action client's request_timeout) are TypeErrors rather than
    silently ignored. What they DO is python/rtps_rpc_timeouts_test.py's job
    (it needs a network);
 5. signatures: the keyword arguments of the extended methods stay in their
    first-line signatures, with their documented defaults.

Exits 0 on success, 1 on the first failed check.
"""

import re
import sys

import espp

Rtps = espp.RtpsParticipant
Task = espp.Task.BaseConfig
EMPTY = bytes([0, 1, 0, 0, 0])


def check(cond, msg):
    if not cond:
        print(f"FAIL: {msg}", file=sys.stderr)
        sys.exit(1)


def raises(exc, fn):
    try:
        fn()
    except exc:
        return True
    except Exception as e:  # noqa: BLE001 - the test wants the exact type
        print(f"  (raised {type(e).__name__} instead of {exc.__name__}: {e})", file=sys.stderr)
        return False
    return False


def signature(obj):
    doc = obj.__doc__ or ""
    return doc.splitlines()[0] if doc else ""


def main():
    # 1. Config construction and round-trip
    c = Rtps.Config()
    check(c.receive_workers == 0, "receive_workers defaults to 0 (the transport's default)")
    for name in ("receive_worker_task", "reactor_task", "job_retry_task", "protocol_task", "reply_hold_task"):
        check(getattr(c, name) is None, f"{name} defaults to None")
    check(c.service_reply_match_timeout_ms == 1000, "service_reply_match_timeout_ms defaults to 1000")

    long_name = "a task name far longer than any small-string buffer holds"
    c = Rtps.Config(
        interface_address="",
        receive_workers=3,
        receive_worker_task=Task(name=long_name, stack_size_bytes=8192, priority=7, core_id=1),
        reactor_task=Task(name="r"),
        job_retry_task=None,
        protocol_task=Task(stack_size_bytes=6144),
        reply_hold_task=Task(name="ignored", stack_size_bytes=4096),
    )
    check(c.receive_workers == 3, "receive_workers round-trips")
    rx = c.receive_worker_task
    check(rx is not None and rx.name == long_name and rx.stack_size_bytes == 8192 and rx.priority == 7
          and rx.core_id == 1, "receive_worker_task round-trips every field")
    check(c.reactor_task.name == "r" and c.job_retry_task is None and c.protocol_task.stack_size_bytes == 6144,
          "the other task configs round-trip")
    # the read-back value must outlive the field: set it to None, then read the copy
    c.receive_worker_task = None
    check(c.receive_worker_task is None, "a task config can be cleared with None")
    check(rx.name == long_name and rx.stack_size_bytes == 8192, "a read-back task config is a copy, not a view")
    c.receive_worker_task = Task(name="again")
    check(c.receive_worker_task.name == "again" and rx.name == long_name, "re-assigning does not alias the copy")
    # mutating the copy does not write through (value semantics)
    c.reactor_task.name = "changed"
    check(c.reactor_task.name == "r", "the read-back task config is a copy (no write-through)")
    # attribute assignment of every field
    c.receive_workers = 2
    c.protocol_task = None
    check(c.receive_workers == 2 and c.protocol_task is None, "attribute assignment works")

    # positional callers of the original leading arguments are unaffected
    c = Rtps.Config("", None, None, espp.Logger.Verbosity.error)
    check(c.interface_address == "" and c.log_level == espp.Logger.Verbosity.error and c.receive_workers == 0,
          "positional leading arguments still work and the new ones default")

    # 2. validation
    check(raises(TypeError, lambda: Rtps.Config(receive_workers=-1)), "negative receive_workers is a TypeError")
    check(raises(TypeError, lambda: Rtps.Config(receive_workers=2**64)), "oversized receive_workers is a TypeError")
    check(raises(TypeError, lambda: Rtps.Config(reactor_task="not a BaseConfig")),
          "a non-BaseConfig task config is a TypeError")
    check(Rtps.Config(max_prioritized_endpoint_ports=255).max_prioritized_endpoint_ports == 255,
          "the uint8 field takes 255")
    check(raises(TypeError, lambda: Rtps.Config(max_prioritized_endpoint_ports=256)),
          "an out-of-range uint8 is a TypeError")
    c = Rtps.Config(receive_workers=1)

    def set_negative():
        c.receive_workers = -5

    check(raises(TypeError, set_negative) and c.receive_workers == 1,
          "a rejected attribute assignment leaves the value untouched")

    # 3. a participant that was never started
    p = Rtps(Rtps.Config(log_level=espp.Logger.Verbosity.none))
    check(not p.add_writer("rt/x", "std_msgs::msg::dds_::String_"), "add_writer before start() returns False")
    check(not p.add_reader("rt/x", "std_msgs::msg::dds_::String_", on_sample=lambda b: None),
          "add_reader before start() returns False")
    check(not p.publish("rt/x", EMPTY), "publish before start() returns False")
    check(not p.add_service_server("/s", "example_interfaces::srv::dds_::AddTwoInts", lambda req: EMPTY),
          "add_service_server before start() returns False")
    check(p.add_service_client("/s", "example_interfaces::srv::dds_::AddTwoInts") is None,
          "add_service_client before start() returns None")
    check(not p.add_action_server("/a", "example_interfaces::action::dds_::Fibonacci", lambda g: True,
                                  lambda h: None, execute_workers=2, max_goals_in_flight=0,
                                  execute_task=Task(stack_size_bytes=8192)),
          "add_action_server before start() returns False (with every new keyword argument)")
    check(p.add_action_client("/a", "example_interfaces::action::dds_::Fibonacci") is None,
          "add_action_client before start() returns None")
    check(not p.add_native_action_server("/n", "T", lambda g: True, lambda h: None, None, execute_workers=0),
          "add_native_action_server before start() returns False (execute_workers=0 accepted)")
    check(raises(TypeError, lambda: p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                                        execute_workers=-1)),
          "a negative execute_workers is a TypeError")
    check(raises(TypeError, lambda: p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                                        execute_task=42)),
          "a non-BaseConfig execute_task is a TypeError")
    # 4. the RPC timeout knobs, on the same unstarted participant
    for kwargs in ({"request_timeout": 0.3, "max_pending_requests": 4}, {"request_timeout": -1.0},
                   {"request_timeout": 0}, {"max_pending_requests": 0}):
        check(p.add_service_client("/s", "T", **kwargs) is None,
              f"add_service_client({kwargs}) is accepted (None before start)")
        check(p.add_native_service_client("/s", "T", **kwargs) is None,
              f"add_native_service_client({kwargs}) is accepted (None before start)")
    for kwargs in ({"request_timeout": 0.3}, {"request_timeout": -5.0}, {"request_timeout": 0}):
        check(p.add_action_client("/a", "T", **kwargs) is None,
              f"add_action_client({kwargs}) is accepted (None before start)")
    for kwargs in ({"result_timeout": 0.2}, {"result_timeout": 0}, {"result_timeout": -1.0},
                   {"unknown_goal_response": b"\x00\x01\x00\x00\x00"}, {"unknown_goal_response": b""}):
        check(not p.add_action_server("/a", "T", lambda g: True, lambda h: None, **kwargs),
              f"add_action_server({kwargs}) is accepted (False before start)")
    for bad in ("0.3", None, [1]):
        check(raises(TypeError, lambda: p.add_native_service_client("/s", "T", request_timeout=bad)),
              f"add_native_service_client(request_timeout={bad!r}) is a TypeError")
        check(raises(TypeError, lambda: p.add_action_client("/a", "T", request_timeout=bad)),
              f"add_action_client(request_timeout={bad!r}) is a TypeError")
    check(raises(TypeError, lambda: p.add_native_service_client("/s", "T", max_pending_requests=-1)),
          "a negative max_pending_requests is a TypeError")
    check(raises(TypeError, lambda: p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                                        unknown_goal_response="not bytes")),
          "a str unknown_goal_response is a TypeError (bytes are due)")
    check(raises(TypeError, lambda: p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                                        result_timeout="60")),
          "a str result_timeout is a TypeError")
    # huge timeouts are accepted (and mean never / keep forever: the facade
    # saturates its deadlines - the functional test pins the behaviour); NaN is
    # refused rather than read as 0
    for huge in (float("inf"), 1e300, 1e10):
        check(p.add_service_client("/s", "T", request_timeout=huge) is None,
              f"add_service_client(request_timeout={huge}) is accepted")
        check(p.add_native_service_client("/s", "T", request_timeout=huge) is None,
              f"add_native_service_client(request_timeout={huge}) is accepted")
        check(p.add_action_client("/a", "T", request_timeout=huge) is None,
              f"add_action_client(request_timeout={huge}) is accepted")
        check(not p.add_action_server("/a", "T", lambda g: True, lambda h: None, result_timeout=huge),
              f"add_action_server(result_timeout={huge}) is accepted")
    nan = float("nan")
    check(raises(ValueError, lambda: p.add_service_client("/s", "T", request_timeout=nan)),
          "add_service_client(request_timeout=nan) is a ValueError")
    check(raises(ValueError, lambda: p.add_native_service_client("/s", "T", request_timeout=nan)),
          "add_native_service_client(request_timeout=nan) is a ValueError")
    check(raises(ValueError, lambda: p.add_action_client("/a", "T", request_timeout=nan)),
          "add_action_client(request_timeout=nan) is a ValueError")
    check(raises(ValueError, lambda: p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                                         result_timeout=nan)),
          "add_action_server(result_timeout=nan) is a ValueError")
    # an unknown_goal_response that does not fit one DATA (the reply budget,
    # 65391 bytes) is refused before the started check, so it shows here too:
    # accepted, it would be dropped on every get_result for an unknown goal and
    # a ROS 2 client asking for a retired goal would hang
    check(not p.add_action_server("/a", "T", lambda g: True, lambda h: None,
                                  unknown_goal_response=bytes(70000)),
          "an oversize unknown_goal_response is refused")
    # knobs a protocol does not have are refused, not silently ignored
    check(raises(TypeError, lambda: p.add_native_action_server("/n", "T", lambda g: True, lambda h: None,
                                                               result_timeout=1.0)),
          "the native action server has no result_timeout (a goal is forgotten when it terminates)")
    check(raises(TypeError, lambda: p.add_native_action_client("/n", "T", request_timeout=1.0)),
          "the native action client has no request_timeout")
    p.stop()  # harmless on a participant that never started
    p.stop()
    del p

    # 5. signatures and defaults
    def has_default(sig, arg, default):
        return re.search(rf"\b{arg}: [^,)]*= {re.escape(default)}(?=[,)])", sig) is not None

    sig = signature(Rtps.Config.__init__)
    for arg, default in (("receive_workers", "0"), ("receive_worker_task", "None"), ("reactor_task", "None"),
                         ("job_retry_task", "None"), ("protocol_task", "None"), ("reply_hold_task", "None")):
        check(has_default(sig, arg, default), f"Config({arg}=...) defaults to {default} in the signature")
    for method in ("add_action_server", "add_native_action_server"):
        sig = signature(getattr(Rtps, method))
        for arg, default in (("max_goals_in_flight", "8"), ("execute_workers", "1"), ("execute_task", "None")):
            check(has_default(sig, arg, default), f"{method}({arg}=...) defaults to {default} in the signature")
    for method in ("add_service_client", "add_native_service_client"):
        sig = signature(getattr(Rtps, method))
        for arg, default in (("request_timeout", "30.0"), ("max_pending_requests", "64")):
            check(has_default(sig, arg, default), f"{method}({arg}=...) defaults to {default} in the signature")
    check(has_default(signature(Rtps.add_action_client), "request_timeout", "30.0"),
          "add_action_client(request_timeout=...) defaults to 30.0 in the signature")
    sig = signature(Rtps.add_action_server)
    check(has_default(sig, "result_timeout", "60.0"), "add_action_server(result_timeout=...) defaults to 60.0")
    check(has_default(sig, "unknown_goal_response", "b''"), "add_action_server(unknown_goal_response=...) defaults to b''")

    # 6. positional order frozen: the released leading arguments keep their
    # places and every later addition is appended; an argument inserted in the
    # middle would silently rebind positional callers, so the ORDER is pinned
    def param_names(method):
        first = signature(getattr(Rtps, method))
        inner = first[first.index("(") + 1:first.rindex(")")]
        return [part.split(":")[0].strip() for part in re.split(r",(?![^\[\]]*\])", inner)
                if part.strip() and not part.strip().startswith("*") and part.split(":")[0].strip() != "self"]

    expected = {
        "add_service_client": ["service", "type_name", "band", "dscp", "request_timeout", "max_pending_requests"],
        "add_native_service_client": ["service", "type_name", "band", "dscp", "request_timeout",
                                      "max_pending_requests"],
        "add_action_client": ["action", "type_name", "band", "dscp", "request_timeout"],
        "add_native_action_client": ["action", "type_name", "band", "dscp"],
        "add_action_server": ["action", "type_name", "on_goal", "execute", "band", "dscp", "max_goals_in_flight",
                              "execute_workers", "execute_task", "result_timeout", "unknown_goal_response"],
        "add_native_action_server": ["action", "type_name", "on_goal", "execute", "on_cancel", "band", "dscp",
                                     "max_goals_in_flight", "execute_workers", "execute_task"],
    }
    for method, names in expected.items():
        got = param_names(method)
        check(got == names, f"{method} parameter order is {got}, expected {names}")

    print("PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())

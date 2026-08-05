"""SocketReactor Python test.

Exercises the espp.SocketReactor Python binding: lifecycle, UDP receiver
registration + echo round-trip, sender info, callbacks returning None (no
response), multiple receivers, dynamic remove, and input validation. A plain
Python UDP socket is used as the client.

Exit code 0 on full pass, 1 on any failure.
"""

import socket
import sys
import time
from typing import List, Optional, Tuple

import espp

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

results: List[Tuple[str, bool]] = []


def check(test: str, condition: bool, desc: str) -> bool:
    ok = bool(condition)
    print(f"  {'PASS' if ok else 'FAIL'} [{test}]: {desc}")
    results.append((test, ok))
    return ok


def udp_send_recv(port: int, payload: bytes, timeout: float = 1.0) -> Optional[bytes]:
    """Send `payload` to 127.0.0.1:port and return the reply bytes (or None on timeout)."""
    cli = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cli.settimeout(timeout)
    try:
        cli.sendto(payload, ("127.0.0.1", port))
        reply, _ = cli.recvfrom(4096)
        return reply
    except socket.timeout:
        return None
    finally:
        cli.close()


def wait_until(predicate, timeout: float = 1.0, interval: float = 0.01) -> bool:
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


VERB = espp.Logger.Verbosity.warn

# ---------------------------------------------------------------------------
# 1. Lifecycle: auto_start=False -> start() -> is_running() -> stop()
# ---------------------------------------------------------------------------
name = "lifecycle"
print(f"--- {name} ---")
reactor = espp.SocketReactor(worker_count=2, auto_start=False, log_level=VERB)
check(name, not reactor.is_running(), "not running before start()")
check(name, reactor.start(), "start() returns True")
check(name, reactor.is_running(), "running after start()")
check(name, reactor.num_registered() == 0, "no registrations initially")

# ---------------------------------------------------------------------------
# 2. UDP echo: a Python callback reverses the payload; the reactor sends it back
# ---------------------------------------------------------------------------
name = "udp echo"
print(f"--- {name} ---")
server_a = espp.UdpSocket(espp.UdpSocket.Config(VERB))


def reverse_cb(data, sender):
    return bytes(reversed(data))


id_a = reactor.add_udp_receiver(server_a, 5111, 1500, reverse_cb)
check(name, id_a != espp.SocketReactor.INVALID_ID, "add_udp_receiver returns a valid id")
check(name, reactor.num_registered() == 1, "one registration")
payload = bytes(range(40))
reply = udp_send_recv(5111, payload)
check(name, reply == bytes(reversed(payload)), "reversed echo received via the reactor")

# ---------------------------------------------------------------------------
# 3. Sender info + a callback that returns None sends no response
# ---------------------------------------------------------------------------
name = "no response / sender info"
print(f"--- {name} ---")
server_b = espp.UdpSocket(espp.UdpSocket.Config(VERB))
seen = {}


def sink_cb(data, sender):
    seen["address"] = sender.address
    seen["length"] = len(data)
    return None  # -> reactor sends nothing back


id_b = reactor.add_udp_receiver(server_b, 5112, 1500, sink_cb)
check(name, id_b != espp.SocketReactor.INVALID_ID, "second receiver registered")
check(name, reactor.num_registered() == 2, "two registrations")
no_reply = udp_send_recv(5112, b"hello", timeout=0.4)
check(name, no_reply is None, "no reply when the callback returns None")
check(name, wait_until(lambda: seen.get("length") == 5), "callback observed the 5-byte payload")
check(name, seen.get("address") == "127.0.0.1", f"sender address seen: {seen.get('address')}")

# ---------------------------------------------------------------------------
# 4. Dynamic remove()
# ---------------------------------------------------------------------------
name = "remove"
print(f"--- {name} ---")
check(name, reactor.remove(id_a) is True, "remove() returns True for a valid id")
check(name, wait_until(lambda: reactor.num_registered() == 1), "count drops to 1 after remove()")
check(name, reactor.remove(999999) is False, "remove() of an unknown id returns False")
check(name, udp_send_recv(5111, b"x", timeout=0.3) is None, "removed receiver no longer echoes")

# ---------------------------------------------------------------------------
# 5. A raising callback must not crash the worker; the reactor keeps serving.
# ---------------------------------------------------------------------------
name = "callback exception"
print(f"--- {name} ---")
raiser = espp.UdpSocket(espp.UdpSocket.Config(VERB))


def raising_cb(data, sender):
    raise ValueError("boom")


check(name, reactor.add_udp_receiver(raiser, 5113, 1500, raising_cb) != espp.SocketReactor.INVALID_ID,
      "raising receiver registered")
# The datagram triggers the exception (reported via sys.unraisablehook); no reply,
# no crash.
check(name, udp_send_recv(5113, b"boom", timeout=0.4) is None, "raising callback yields no reply")
# The reactor is still alive: the earlier echo receiver on 5112's sibling keeps working.
echo_srv = espp.UdpSocket(espp.UdpSocket.Config(VERB))
reactor.add_udp_receiver(echo_srv, 5114, 1500, reverse_cb)
payload2 = bytes(range(24))
check(name, udp_send_recv(5114, payload2) == bytes(reversed(payload2)),
      "reactor still serves other receivers after a callback exception")

# ---------------------------------------------------------------------------
# 6. Stop
# ---------------------------------------------------------------------------
name = "stop"
print(f"--- {name} ---")
reactor.stop()
check(name, not reactor.is_running(), "not running after stop()")

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
passed = sum(1 for _, ok in results if ok)
total = len(results)
print()
print(f"======== SocketReactor python test: {passed}/{total} checks passed ========")
if passed != total:
    for test_name, ok in results:
        if not ok:
            print(f"  FAILED: {test_name}")
    sys.exit(1)
print("ALL CHECKS PASSED")
sys.exit(0)

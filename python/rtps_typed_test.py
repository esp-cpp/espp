#!/usr/bin/env python3
"""End-to-end test of the espp.rtps typed pub/sub wrapper with pycdr2.

Two participants in one process exchange a complex nested message (string +
scalars + nested struct + fixed array + variable sequence) using
espp.rtps.Publisher / Subscriber. Verifies the fields round-trip through real
SEDP/DATA over the espp transport (not just a count). Exits 0 on success.

Requires pycdr2 (see python/requirements.txt).
"""

import sys
import threading
import time

import espp

from rtps_messages import SensorSample, Vector3

REQUIRED = 5
TOPIC = "rt/imu_test"


def main() -> int:
    received = []
    lock = threading.Lock()

    def on_message(msg) -> None:
        with lock:
            received.append(msg)

    R = espp.RtpsParticipant
    pub_p = R(R.Config(log_level=espp.Logger.Verbosity.warn))
    sub_p = R(R.Config(log_level=espp.Logger.Verbosity.warn))
    if not pub_p.start() or not sub_p.start():
        print("FAIL: start")
        return 1

    pub = espp.rtps.Publisher(pub_p, TOPIC, SensorSample, reliable=True)
    sub = espp.rtps.Subscriber(sub_p, TOPIC, SensorSample, on_message=on_message, reliable=True)
    if not pub.valid or not sub.valid:
        print("FAIL: registration")
        return 1

    time.sleep(2.0)  # let SEDP match

    sent = 0
    deadline = time.monotonic() + 20.0
    while len(received) < REQUIRED and time.monotonic() < deadline:
        pub.publish(
            SensorSample(
                frame_id="imu_link",
                stamp_ns=1000 + sent,
                seq=sent,
                angular_velocity=Vector3(x=float(sent), y=0.0, z=-float(sent)),
                orientation_covariance=[0.0] * 9,
                samples=[float(sent), float(sent) + 0.5],
            )
        )
        sent += 1
        time.sleep(0.1)

    with lock:
        n = len(received)
        # Field-level check: the typed round-trip preserved nested + string fields.
        fields_ok = any(
            m.frame_id == "imu_link"
            and m.angular_velocity.z == -float(m.seq)
            and list(m.samples) == [float(m.seq), float(m.seq) + 0.5]
            for m in received
        )
    pub_p.stop()
    sub_p.stop()

    print(f"sent={sent} received={n} fields_ok={fields_ok}")
    if n >= REQUIRED and fields_ok:
        print("PASS")
        return 0
    print("FAIL")
    return 1


if __name__ == "__main__":
    sys.exit(main())

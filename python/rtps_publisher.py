#!/usr/bin/env python3
"""Standalone RTPS publisher using the espp Python library (embeddedRTPS engine).

Publishes a complex nested message (espp_examples/msg/SensorSample demo message: string + scalars +
nested struct + fixed array + variable sequence) using pycdr2 for CDR
(de)serialization and the light espp.rtps.Publisher wrapper. Pair it with
rtps_subscriber.py, a FastDDS peer, or a ROS 2 node (rmw_fastrtps).

Requires pycdr2 (see python/requirements.txt).

Usage: python rtps_publisher.py [topic] [interface_ipv4] [period_seconds]
"""

import sys
import time

import espp

from rtps_messages import SensorSample, Vector3


def main() -> int:
    topic = sys.argv[1] if len(sys.argv) > 1 else "rt/imu"
    interface = sys.argv[2] if len(sys.argv) > 2 else ""  # "" -> auto-detect
    period = float(sys.argv[3]) if len(sys.argv) > 3 else 0.5

    R = espp.RtpsParticipant
    participant = R(R.Config(interface_address=interface, log_level=espp.Logger.Verbosity.info))
    if not participant.start():
        print("failed to start participant")
        return 1

    # Publisher<SensorSample>: the DDS type name is derived from the pycdr2 typename
    # ("espp_examples/msg/SensorSample" -> "espp_examples::msg::dds_::SensorSample_").
    pub = espp.rtps.Publisher(participant, topic, SensorSample, reliable=True)
    if not pub.valid:
        print("failed to add writer")
        return 1
    print(f"publishing '{topic}' (espp_examples/msg/SensorSample) every {period}s; ctrl-c to stop")

    count = 0
    try:
        while True:
            msg = SensorSample(
                frame_id="imu_link",
                stamp_ns=time.time_ns(),
                seq=count,
                angular_velocity=Vector3(x=0.01 * count, y=0.0, z=-0.01 * count),
                orientation_covariance=[0.0] * 9,
                samples=[float(count), float(count) + 0.5],
            )
            if pub.publish(msg):  # pycdr2 serializes msg -> CDR bytes for you
                count += 1
                print(f"sent {count}")
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        participant.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())

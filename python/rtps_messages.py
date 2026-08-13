#!/usr/bin/env python3
"""Shared pycdr2 message definitions for the RTPS python examples.

pycdr2 turns a message schema into a CDR codec (``.serialize()`` /
``.deserialize()``) that speaks the same ROS 2 / classic-CDR wire format as the
espp C++ side, so these types interoperate with FastDDS and ROS 2.

Two ways to define a message with pycdr2:

* On Python <= 3.12, the idiomatic dataclass form works::

      from dataclasses import dataclass
      from pycdr2 import IdlStruct
      from pycdr2.types import float64

      @dataclass
      class Vector3(IdlStruct, typename="geometry_msgs/msg/Vector3"):
          x: float64 = 0.0
          y: float64 = 0.0
          z: float64 = 0.0

* The functional ``make_idl_struct`` form below works on every Python (including
  3.14, where the dataclass form currently trips over CPython's dataclass
  changes). It is otherwise equivalent.
"""

from pycdr2 import make_idl_struct
from pycdr2.types import array, float64, sequence, uint32, uint64

# geometry_msgs/msg/Vector3 — a nested sub-message.
Vector3 = make_idl_struct(
    "Vector3",
    "geometry_msgs/msg/Vector3",
    {"x": float64, "y": float64, "z": float64},
)

# A composite message showing the pieces you actually hit in real message types:
# a string, scalars, a NESTED struct, a FIXED array, and a variable SEQUENCE.
Imu = make_idl_struct(
    "Imu",
    "sensor_msgs/msg/Imu",
    {
        "frame_id": str,  # string
        "stamp_ns": uint64,  # scalar
        "seq": uint32,  # scalar
        "angular_velocity": Vector3,  # nested struct
        "orientation_covariance": array[float64, 9],  # fixed-size array
        "samples": sequence[float64],  # variable-length sequence
    },
)

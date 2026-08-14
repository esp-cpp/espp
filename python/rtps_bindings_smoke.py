#!/usr/bin/env python3
"""Wheel smoke test: import espp and assert the RTPS RMI/AMI binding surface is
exposed. Used by cibuildwheel (see pyproject.toml [tool.cibuildwheel]).

This is a binding-presence check, not a functional round-trip - a full in-process
demo needs multicast discovery and lives in the dockerised interop harness (see
python/rtps_rpc_demo.py). Catches a dropped/renamed binding after a build change.
"""

import sys

import espp

p = espp.RtpsParticipant

methods = [
    # pub/sub
    "add_writer", "add_reader", "publish",
    # services (RMI) - ROS 2 + native
    "add_service_server", "add_service_client",
    "add_native_service_server", "add_native_service_client",
    # actions (AMI) - ROS 2 + native
    "add_action_server", "add_action_client",
    "add_native_action_server", "add_native_action_client",
]
classes = [
    "ServiceClient", "ActionClient", "ActionGoalHandle",
    "NativeServiceClient", "NativeActionClient", "NativeGoalHandle",
]

# Methods on the client / goal-handle classes (the full client call surface).
class_methods = {
    "ServiceClient": ["call", "call_async", "call_future"],
    "NativeServiceClient": ["call", "call_async", "call_future"],
    "ActionClient": ["send_goal"],
    "ActionGoalHandle": ["goal", "publish_feedback", "succeed", "abort", "canceled", "is_canceling"],
    "NativeGoalHandle": ["goal", "publish_feedback", "succeed", "abort"],
}

missing = [m for m in methods if not hasattr(p, m)] + [c for c in classes if not hasattr(p, c)]
for cls, method_names in class_methods.items():
    handle = getattr(p, cls, None)
    missing += [f"{cls}.{m}" for m in method_names if handle is None or not hasattr(handle, m)]

if missing:
    print("espp RTPS binding surface INCOMPLETE, missing:", missing, file=sys.stderr)
    sys.exit(1)

print(f"espp {espp.__version__}: RTPS pub/sub + RMI/AMI (services, actions, native) bindings ok")

"""Lightweight typed pub/sub over :class:`espp.RtpsParticipant`.

``espp.RtpsParticipant`` transports CDR-encapsulated bytes. This module pairs it
with a runtime CDR codec so you publish and receive *message objects* instead of
raw bytes -- the Python counterpart to the C++ ``espp::Publisher<T>`` /
``espp::Subscriber<T>`` typed layer (templates can't be bound generically, so the
Python equivalent is this small pure-Python wrapper).

Any message type works as long as it supplies a CDR codec:

* an instance ``.serialize() -> bytes`` and a classmethod ``.deserialize(bytes)``
  (this is exactly what `pycdr2 <https://pypi.org/project/pycdr2/>`_ dataclasses
  provide, and pycdr2 emits ROS 2 / DDS-compatible CDR, so espp interoperates
  with FastDDS and ROS 2 out of the box), **or**
* explicit ``serialize`` / ``deserialize`` callables you pass in, for any other
  codec.

Example (pycdr2)::

    from pycdr2 import make_idl_struct
    from pycdr2.types import float64
    import espp

    Vector3 = make_idl_struct("Vector3", "geometry_msgs/msg/Vector3",
                              {"x": float64, "y": float64, "z": float64})

    participant = espp.RtpsParticipant(espp.RtpsParticipant.Config())
    participant.start()
    pub = espp.rtps.Publisher(participant, "rt/vec", Vector3, reliable=True)
    pub.publish(Vector3(x=1.0, y=2.0, z=3.0))
    sub = espp.rtps.Subscriber(participant, "rt/vec", Vector3,
                               on_message=lambda v: print(v), reliable=True)
"""

from __future__ import annotations

from typing import Any, Callable, Optional


def ros2_dds_type_name(name: str) -> str:
    """Map a ROS 2 type name to its DDS wire type name.

    ``"std_msgs/msg/String"`` -> ``"std_msgs::msg::dds_::String_"``. A name that
    already looks like a DDS type name (contains ``"::"``) is returned unchanged.
    """
    if "::" in name:
        return name
    parts = [p for p in name.split("/") if p]
    if len(parts) < 2:
        return name
    *namespace, type_name = parts
    return "::".join(namespace) + "::dds_::" + type_name + "_"


def _resolve_type_name(message_type: Any, type_name: Optional[str]) -> str:
    if type_name is not None:
        return type_name
    # pycdr2 stores the ROS 2 typename on the generated class.
    idl = getattr(message_type, "__idl_typename__", None)
    if idl:
        return ros2_dds_type_name(idl)
    raise ValueError(
        "type_name is required: the message type has no __idl_typename__ to derive it from"
    )


class Publisher:
    """Publish message objects on a topic, serializing them to CDR bytes.

    :param participant: a started :class:`espp.RtpsParticipant`.
    :param topic: DDS topic name.
    :param message_type: message class supplying ``.serialize()`` (and, for a
        pycdr2 type, ``__idl_typename__`` so ``type_name`` can be derived).
    :param type_name: DDS type name; derived from ``message_type`` if omitted.
    :param reliable: use RELIABLE QoS (default best-effort).
    :param serialize: optional ``callable(msg) -> bytes`` overriding
        ``message_type.serialize``.
    """

    def __init__(
        self,
        participant: Any,
        topic: str,
        message_type: Any = None,
        *,
        type_name: Optional[str] = None,
        reliable: bool = False,
        serialize: Optional[Callable[[Any], bytes]] = None,
    ) -> None:
        self._participant = participant
        self._topic = topic
        self._serialize = serialize if serialize is not None else (lambda m: m.serialize())
        self.valid = participant.add_writer(
            topic=topic, type_name=_resolve_type_name(message_type, type_name), reliable=reliable
        )

    def publish(self, message: Any) -> bool:
        """Serialize ``message`` and publish it. Returns True on success."""
        return self._participant.publish(self._topic, self._serialize(message))


class Subscriber:
    """Receive message objects from a topic, deserializing CDR bytes for you.

    :param participant: a started :class:`espp.RtpsParticipant`.
    :param topic: DDS topic name.
    :param message_type: message class supplying ``.deserialize(bytes)`` (and,
        for a pycdr2 type, ``__idl_typename__``).
    :param on_message: ``callable(msg)`` invoked for each received sample. Runs
        on an engine worker thread -- return quickly, do not block.
    :param type_name: DDS type name; derived from ``message_type`` if omitted.
    :param reliable: use RELIABLE QoS (default best-effort).
    :param deserialize: optional ``callable(bytes) -> msg`` overriding
        ``message_type.deserialize``.
    """

    def __init__(
        self,
        participant: Any,
        topic: str,
        message_type: Any = None,
        *,
        on_message: Callable[[Any], None],
        type_name: Optional[str] = None,
        reliable: bool = False,
        deserialize: Optional[Callable[[bytes], Any]] = None,
    ) -> None:
        deser = deserialize if deserialize is not None else (lambda data: message_type.deserialize(data))

        def _on_sample(data: bytes) -> None:
            try:
                message = deser(data)
            except Exception:
                # A malformed / wrong-type sample must not kill the reader thread.
                return
            on_message(message)

        self.valid = participant.add_reader(
            topic=topic,
            type_name=_resolve_type_name(message_type, type_name),
            reliable=reliable,
            on_sample=_on_sample,
        )


# ---------------------------------------------------------------------------
# Services (RMI) and actions (AMI) - the typed Python counterparts to the C++
# espp::ServiceServer/Client and espp::ActionServer/Client. Each pairs the
# byte-level RtpsParticipant RPC methods with a pycdr2 (or compatible) codec, so
# you deal in message objects instead of CDR bytes. Pass native=True to use the
# lean espp<->espp protocol instead of the ROS 2-interoperable one.
# ---------------------------------------------------------------------------


class ServiceServer:
    """Answer requests with responses, (de)serializing message objects for you.

    :param participant: a started :class:`espp.RtpsParticipant`.
    :param service: service name, e.g. ``"/add_two_ints"``.
    :param type_name: base DDS service type, e.g.
        ``"example_interfaces::srv::dds_::AddTwoInts"`` (any matching name for
        native).
    :param request_type: message class with ``.deserialize(bytes)``.
    :param response_type: message class with ``.serialize()`` (documentary; the
        handler returns an instance whose ``.serialize()`` is used).
    :param handler: ``callable(request_msg) -> response_msg``. Runs on an engine
        worker thread -- return promptly.
    :param native: use the lean native protocol instead of ROS 2.
    """

    def __init__(self, participant, service, type_name, request_type, response_type, handler,
                 *, native: bool = False) -> None:
        def _byte_handler(request_bytes: bytes) -> bytes:
            try:
                req = request_type.deserialize(request_bytes)
            except Exception:
                return b""
            return handler(req).serialize()

        add = participant.add_native_service_server if native else participant.add_service_server
        self.valid = add(service, type_name, _byte_handler)


class ServiceClient:
    """Call a service with a request object and get a response object back.

    Blocking :meth:`call` (RMI) and callback :meth:`call_async` (AMI). See
    :class:`ServiceServer` for the constructor parameters.
    """

    def __init__(self, participant, service, type_name, request_type, response_type,
                 *, native: bool = False) -> None:
        add = participant.add_native_service_client if native else participant.add_service_client
        self._client = add(service, type_name)
        self._response_type = response_type
        self.valid = self._client is not None

    def call(self, request, timeout: float = 5.0):
        """Blocking call. Returns the response object, or ``None`` on timeout."""
        reply = self._client.call(request.serialize(), timeout)
        return None if reply is None else self._response_type.deserialize(reply)

    def call_async(self, request, on_response) -> bool:
        """Async call: ``on_response(response_msg)`` when the reply arrives."""
        rt = self._response_type
        return self._client.call_async(request.serialize(), lambda b: on_response(rt.deserialize(b)))

    def call_future(self, request):
        """Async call returning a :class:`concurrent.futures.Future` of the
        response object (result is ``None`` if the request could not be queued).
        Use ``fut.result(timeout=...)``. Works for both protocols."""
        import concurrent.futures

        fut: concurrent.futures.Future = concurrent.futures.Future()
        if not self.call_async(request, fut.set_result):
            fut.set_result(None)
        return fut


class GoalHandle:
    """Typed view of a server-side goal handle, passed to an action ``execute``
    callback. Publish feedback and terminate the goal with message objects."""

    def __init__(self, handle, goal_type, result_type, feedback_type) -> None:
        self._handle = handle
        self._result_type = result_type
        self._feedback_type = feedback_type
        self.goal = goal_type.deserialize(handle.goal())

    def publish_feedback(self, feedback) -> None:
        self._handle.publish_feedback(feedback.serialize())

    def succeed(self, result) -> None:
        self._handle.succeed(result.serialize())

    def abort(self, result) -> None:
        self._handle.abort(result.serialize())

    def canceled(self, result) -> None:
        """Terminate the goal CANCELED (in response to a cancel request). Works on
        both the ROS 2 and native protocols."""
        canceled = getattr(self._handle, "canceled", None)
        if canceled is not None:
            canceled(result.serialize())

    def is_canceling(self) -> bool:
        """True if the client has requested cancellation of this goal (both the
        ROS 2 and native protocols). Poll this in a long execute() and wind the
        goal down - calling canceled() - when it becomes true."""
        return getattr(self._handle, "is_canceling", lambda: False)()


class ActionServer:
    """Run long goals with typed goal / result / feedback messages.

    :param on_goal: ``callable(goal_msg) -> bool`` (accept/reject).
    :param execute: ``callable(GoalHandle)`` run on its own thread.

    Other parameters mirror :class:`ServiceServer` (``action`` name + base
    ``type_name`` + the three message classes).
    """

    def __init__(self, participant, action, type_name, goal_type, result_type, feedback_type,
                 on_goal, execute, *, native: bool = False) -> None:
        def _byte_on_goal(goal_bytes: bytes) -> bool:
            try:
                return bool(on_goal(goal_type.deserialize(goal_bytes)))
            except Exception:
                return False

        def _byte_execute(handle) -> None:
            execute(GoalHandle(handle, goal_type, result_type, feedback_type))

        add = participant.add_native_action_server if native else participant.add_action_server
        self.valid = add(action, type_name, _byte_on_goal, _byte_execute)


class ActionClient:
    """Send typed goals and receive typed feedback + result."""

    def __init__(self, participant, action, type_name, goal_type, result_type, feedback_type,
                 *, native: bool = False) -> None:
        add = participant.add_native_action_client if native else participant.add_action_client
        self._client = add(action, type_name)
        self._native = native
        self._result_type = result_type
        self._feedback_type = feedback_type
        self._last_goal = None  # native: int handle; ROS 2: bytes goal id
        self.valid = self._client is not None

    def send_goal(self, goal, on_feedback, on_result) -> bool:
        """Send ``goal``; ``on_feedback(feedback_msg)`` per feedback,
        ``on_result(status:int, result_msg)`` once at the end (result is ``None``
        if the goal was rejected). Remembers the accepted goal so cancel_goal()
        can target it."""
        ft, rt = self._feedback_type, self._result_type

        def _fb(b: bytes) -> None:
            try:
                on_feedback(ft.deserialize(b))
            except Exception:
                # Runs on an engine worker thread: a malformed / wrong-type
                # feedback sample (or a raising user callback) must not kill it.
                pass

        def _res(status: int, b: bytes) -> None:
            on_result(status, rt.deserialize(b) if b else None)

        if self._native:
            return self._client.send_goal(
                goal.serialize(), _fb, _res, lambda handle: setattr(self, "_last_goal", handle))
        gid = self._client.send_goal(goal.serialize(), _fb, _res)
        if gid is not None:
            self._last_goal = gid
        return gid is not None

    def cancel_goal(self) -> bool:
        """Request cancellation of the most recently accepted goal (ROS 2 or
        native). The server observes it via GoalHandle.is_canceling(). Returns
        True if the cancel request was queued."""
        if self._last_goal is None:
            return False
        return self._client.cancel_goal(self._last_goal)

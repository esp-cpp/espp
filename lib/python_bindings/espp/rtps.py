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

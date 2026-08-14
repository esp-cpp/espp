RTPS Services & Actions (RMI / AMI)
***********************************

The ``rtps_embedded`` component's :cpp:class:`espp::RtpsParticipant` facade adds
request/reply (**RMI** — Remote Method Invocation) and goal-oriented (**AMI** —
Asynchronous Method Invocation) messaging on top of its RTPS pub/sub, in two
flavours:

- **ROS 2-interoperable** services and actions — byte-compatible with
  ``rmw_fastrtps`` (validated against live ROS 2 Jazzy nodes, both directions).
- **Native** (espp ↔ espp) services and actions — a deliberately lean protocol
  that trades ROS interop for a smaller footprint and simpler wire.

Both flavours are *composition over the same reliable RTPS pub/sub* — no separate
transport. The full design and the wire-format captures that back it are in
``components/rtps_embedded/RMI_AMI_DESIGN.md``.

Why services and actions matter
===============================

Pub/sub is fire-and-forget: a publisher never learns whether anyone acted on a
sample. Many robotics interactions are *requests* ("add these two ints", "move
to this pose") that need a **correlated reply**, and some are *long-running
goals* that need **progress feedback**, a **final result**, and **cancellation**.
ROS 2 models these as **services** and **actions**; this component provides both,
so an espp device can be a first-class ROS 2 service/action server or client, or
talk to another espp device with the leaner native protocol.

The key structural fact (and why this was cheap to build): in ROS 2 a **service
is two topics + reply correlation**, and an **action is three services + two
topics** — *no new wire primitive*. Everything here is library code over pub/sub
plus one addition to the engine (carry a ``related_sample_identity`` inline-QoS on
a reply, so a client can match replies to requests).

Two API levels
==============

Like pub/sub (raw ``publish()``/``on_sample`` vs the typed ``Publisher<T>`` /
``Subscriber<T>``), the RMI/AMI layer has two levels:

- **Typed, espp-idiomatic wrappers** (recommended): ``espp::ServiceServer<Req,
  Resp>`` / ``ServiceClient<Req, Resp>`` and ``ActionServer<Goal, Result,
  Feedback>`` / ``ActionClient<...>`` in ``rtps_service.hpp`` / ``rtps_action.hpp``.
  Reflectable structs are (de)serialized to CDR by the ``cdr`` component - your
  code never touches bytes. Each takes a ``RtpsProtocol`` (``ROS2`` or ``NATIVE``),
  so one class covers both flavours.
- **Byte-level methods** on ``RtpsParticipant`` (``add_service_server`` etc.):
  CDR-encapsulated ``std::span<const uint8_t>`` in/out. Use these for dynamic
  types, or when you already have the bytes.

.. code-block:: cpp

   struct AddReq  { int64_t a, b; };
   struct AddResp { int64_t sum; };
   espp::ServiceServer<AddReq, AddResp> server(participant, {
       .service = "/add_two_ints",
       .type_name = "example_interfaces::srv::dds_::AddTwoInts",
       .handler = [](const AddReq &r) { return AddResp{r.a + r.b}; }});

   espp::ServiceClient<AddReq, AddResp> client(participant, {
       .service = "/add_two_ints",
       .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
   if (auto resp = client.call(AddReq{7, 35}, 1s)) use(resp->sum);   // -> 42

   // Same classes, native protocol - just set .protocol:
   espp::ServiceClient<AddReq, AddResp> native(participant,
       {.service = "/mul", .type_name = "espp::native::Mul",
        .protocol = espp::RtpsProtocol::NATIVE});

The rest of this page shows the byte-level API to explain the wire; the typed
wrappers are thin layers over exactly these calls.

Services (RMI)
==============

A service is a request topic (``rq/<name>Request``) + a reply topic
(``rr/<name>Reply``), with each reply correlated to its request. Payloads are
CDR-encapsulated bytes, exactly like ``publish()`` / ``on_sample``.

.. code-block:: cpp

   // Server: handler(request_cdr) -> reply_cdr
   participant.add_service_server(
       {"/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts"},
       [](std::span<const uint8_t> req) -> std::vector<uint8_t> {
         return make_reply_cdr(a + b);
       });

   // Client: three call styles.
   auto client = participant.add_service_client(
       {"/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts"});
   auto reply  = client->call(req_cdr, 1s);              // (1) blocking (RMI)
   client->call_async(req_cdr, [](auto reply){ ... });   // (2) callback (AMI)
   std::future<...> f = client->call_future(req_cdr);    // (3) promise (AMI)

The client offers **all three ergonomics** so callers pick what fits: block a
worker for a quick RPC, register a callback, or hold a ``std::future``.

For a response that is not ready when the request arrives (e.g. an action's
``get_result``), use :cpp:func:`add_service_server_deferred`: the handler receives
a ``ServiceResponder`` it can store and fulfil later from any thread, so a slow
response never blocks an engine worker (which would deadlock other traffic).

Correlation (ROS interop)
-------------------------

``rmw_fastrtps`` correlates a reply to its request with a **related sample
identity** carried as inline QoS, under **both** PID ``0x0083`` (OMG standard) and
PID ``0x800f`` (eProsima legacy). The client stamps its request with
``{its reply-reader GUID, sequence = UNKNOWN}``; the server echoes
``{that GUID, the request's RTPS sequence number}`` on the reply; the client
matches replies against its pending table. This is confirmed byte-for-byte
against a live capture — and no SEDP type-hash exchange is needed: an espp
service even shows up in ``ros2 service list``.

Actions (AMI)
=============

An action expands to the standard five ROS 2 endpoints — three services
(``send_goal``, ``cancel_goal``, ``get_result``) plus two topics (``feedback``,
``status``) — driven by a goal state machine. All of it is library code over the
services above.

.. code-block:: cpp

   // Server
   participant.add_action_server(
       {"/fibonacci", "example_interfaces::action::dds_::Fibonacci"},
       [](const auto &goal_id, std::span<const uint8_t> goal) { return true; }, // accept?
       [](espp::RtpsParticipant::ActionGoalHandle h) {                          // execute (own thread)
         h.publish_feedback(fb_cdr);
         h.succeed(result_cdr);        // or h.abort(...) / h.canceled(...)
       });

   // Client
   auto action = participant.add_action_client(
       {"/fibonacci", "example_interfaces::action::dds_::Fibonacci"});
   action->send_goal(goal_cdr,
       [](std::span<const uint8_t> fb) { ... },              // per-feedback
       [](int8_t status, std::span<const uint8_t> res) { ... }); // terminal result

Goals are identified by a 16-byte UUID; feedback, status, and the result are all
routed to the right goal by that id. The result rides ``get_result`` (a deferred
service reply that completes when the goal terminates).

.. note::

   **Action result alignment limitation.** The action ``get_result`` envelope is
   ``status:int8`` followed by the result message. The framework splices the
   result's CDR bytes immediately after ``status`` + 3 bytes of padding (offset
   4), which is byte-exact only when the result message's **first field is at most
   4-byte aligned** (int32/uint32, arrays, strings, and structs of those - e.g.
   Fibonacci's ``int32[]``). A result whose first field needs 8-byte alignment
   (``int64``/``uint64``/``float64`` as the first member) would be placed at
   offset 4 instead of the CDR-correct offset 8 and would mis-decode on a ROS 2
   peer. This affects both the byte-level and typed action APIs. If your result
   starts with an 8-byte-aligned field, reorder it (put a 4-byte field first) or
   wrap it in a leading 4-byte field. Goal and feedback payloads are unaffected
   (they follow the 16-byte UUID, which is already 8-aligned). Services (request/
   reply) are unaffected. This is a known v1 limitation of the byte-splice
   envelope; a future revision may build the envelope via full CDR serialization.

Native (espp ↔ espp) services & actions
=======================================

When both ends are espp and ROS interop is not needed, the **native** protocol is
leaner. Correlation is a **20-byte in-band header** prepended to the payload —
``client_prefix(12) + request_id(4) + op(1) + flags(1) + reserved(2)`` — so it
needs *no inline-QoS engine support* and rides plain reliable pub/sub on
``es_rq/`` / ``es_rr/`` topics (a distinct prefix, so it never aliases the ROS
``rq/`` / ``rr/`` topics). A native action collapses ROS's ~10 endpoints to ~3:
one goal service + one feedback topic that also carries the terminal result.

.. code-block:: cpp

   participant.add_native_service_server({"/mul", "espp::native::Mul"}, handler);
   auto c = participant.add_native_service_client({"/mul", "espp::native::Mul"});
   auto r = c->call(req_cdr, 1s);   // same call / call_async ergonomics

   participant.add_native_action_server({"/countup", "espp::native::CountUp"},
                                        on_goal, execute);
   auto a = participant.add_native_action_client({"/countup", "espp::native::CountUp"});
   a->send_goal(goal_cdr, on_feedback, on_result);

Choosing a flavour
------------------

.. list-table::
   :header-rows: 1

   * - Need
     - Use
   * - Talk to ROS 2 nodes / rclcpp / rclpy
     - ``add_service_*`` / ``add_action_*`` (ROS-interoperable)
   * - espp ↔ espp only, minimise endpoints / RAM
     - ``add_native_service_*`` / ``add_native_action_*``

Python
======

The same APIs are exposed through the ``espp`` Python module (pybind11).
Callbacks run on engine threads but are invoked GIL-correctly, so plain Python
callables work. See ``python/rtps_rpc_demo.py`` for a runnable showcase of all
four mechanisms; the essence:

.. code-block:: python

   import espp
   p = espp.RtpsParticipant(espp.RtpsParticipant.Config())
   p.start()
   p.add_service_server("/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts",
                        lambda req: make_reply(req))
   svc = p.add_service_client("/add_two_ints", "example_interfaces::srv::dds_::AddTwoInts")
   reply = svc.call(request_bytes, timeout=1.0)  # bytes or None

Payloads are CDR-encapsulated ``bytes`` (a 4-byte encapsulation header + CDR
body). Pack them with :mod:`struct` for simple fields, or with the ``cdr``
component / ``pycdr2`` for real ROS 2 message types.

Testing
=======

Every mechanism is covered end-to-end:

- **In-process loopbacks** (host, docker-free): ``rtps_service_loopback``,
  ``rtps_action_loopback``, ``rtps_native_service_loopback``,
  ``rtps_native_action_loopback``, and ``rtps_typed_rpc_loopback`` (the typed
  ``ServiceServer/Client`` + ``ActionServer/Client`` wrappers, both protocols) —
  plus wire-format unit tests (``rtps_service_naming``, ``rtps_action_naming``,
  ``rtps_action_types``) checked byte-for-byte against live ROS 2 captures.
- **On-device**: the ``components/rtps_embedded/example`` (esp32) hosts a typed
  ``/add_two_ints`` service and a ``/fibonacci`` action a ROS 2 client can drive.
- **Live ROS 2 interop** (dockerised ``rmw_fastrtps``, both directions):
  ``ros2 service call`` ↔ espp server, espp client ↔ rclpy server, and the same
  for actions (``ros2 action send_goal`` ↔ espp, espp ↔ rclpy). See
  ``components/rtps_embedded/interop/``.
- **Python**: ``python/rtps_rpc_demo.py`` exercises all four mechanisms.

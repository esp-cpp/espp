Socket Component
****************

The network APIs provide a useful abstraction over POSIX sockets, making it
easier to build client/server communication flows and bind callback-driven
receive handlers on embedded targets.

Currently, UDP and TCP sockets are supported.

Base Socket
-----------

The base ``Socket`` abstraction wraps socket file descriptor lifecycle,
configuration helpers, and endpoint conversion utilities shared by both UDP and
TCP transports.


UDP Socket
----------

UDP sockets provide unreliable, unordered communication over IP network
sockets.

UDP sockets can be used in unicast (point to point), multicast (one to many and
many to one), and broadcast (one to all).

The ``UdpSocket`` API supports both one-shot sends and long-running receive
tasks:

- ``send(...)`` can optionally block waiting for a response with a timeout and
  response callback
- ``start_receiving(...)`` starts a task that continuously receives datagrams
  and optionally sends a callback-produced response
- ``stop_receiving()`` cleanly stops an in-flight receive task and closes the
  socket, which is especially useful for teardown paths on embedded targets


TCP Socket
----------

TCP sockets provide reliable, ordered communication over IP network sockets and
have built-in protocols for handling packet acknowledgement as well as
transmission speed / bandwidth adjustment.

TCP sockets cannot be used with multicast (many to one, one to many).

The ``TcpSocket`` API covers both client and server patterns:

- ``connect(...)`` plus ``transmit(...)`` for client-style request/response
- optional blocking response waits with callback delivery
- ``bind(...)``, ``listen(...)``, and ``accept()`` for server-side flows
- explicit ``close()`` / ``reinit()`` helpers used by teardown and reconnect
  paths


Example
-------

The ``network/socket_example`` page shows the use of the
classes provided by the ``socket`` component and runs a scenario-driven
self-test which covers teardown, timeout, and reconnect behavior, including:

* ``UdpSocket`` as both client and server, including unicast and multicast
  configurations
* ``TcpSocket`` as both client and server
* scope-based UDP/TCP teardown while tasks are active or blocked
* request/response callbacks and timeout handling
* reconnect behavior after TCP session shutdown

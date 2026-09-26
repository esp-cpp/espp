TCP Sockets
***********

TCP sockets provide reliable, ordered communication over IP network sockets and
have built-in protocols for handling packet acknowledgement as well as
transmission speed / bandwidth adjustment.

TCP sockets cannot be used with multicast (many to one, one to many).

The implementation supports both client-style request/response flows and
server-style ``bind(...)`` / ``listen(...)`` / ``accept()`` loops. ``close()``
and ``reinit()`` are used by the updated teardown and reconnect paths to ensure
connection state is reset cleanly.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/tcp_socket.inc

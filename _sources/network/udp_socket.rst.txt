UDP Sockets
***********

UDP sockets provide unreliable, unordered communication over IP network
sockets.

UDP sockets can be used in unicast (point to point), multicast (one to many and
many to one), and broadcast (one to all).

The implementation supports both blocking request/response sends and background
receive tasks. In addition to ``start_receiving(...)``, the class exposes
``stop_receiving()`` so applications can reliably stop a blocked receive task
during teardown.

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/udp_socket.inc

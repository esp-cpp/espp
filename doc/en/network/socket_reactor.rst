Socket Reactor
**************

The ``SocketReactor`` multiplexes many receiver sockets on a single
``select()`` event-loop thread and dispatches each socket's read + user callback
onto a shared :doc:`ThreadPool <../core/thread_pool>`, instead of
dedicating one thread (one ``espp::Task``) to every receiving socket. This
collapses "N receiver threads" into "1 loop thread + a small fixed pool", which
can additionally be shared across several subsystems.

To keep per-socket handling correct under level-triggered ``select()``, the
reactor is one-shot: when a socket is reported readable it is *disarmed* and a
job is submitted to the pool, and it is *re-armed* only after that job completes.
This guarantees at most one in-flight handler per socket (so per-socket ordering
is preserved and there is no concurrent ``recv`` on one fd), while different
sockets' handlers run concurrently on the pool. If the pool is saturated the
socket is simply left for the next ``select()`` to re-report - the datagram stays
buffered, giving natural backpressure with no loss.

Registration changes, re-arming, and stop are made immediately responsive by a
loopback UDP "wakeup" socket that is also in the select set.

The reactor drives:

* **UDP receivers** via ``add_udp_receiver()`` (binds the socket, calls the
  receive callback on a pool worker, and sends any returned response), replacing
  a per-socket ``UdpSocket::start_receiving()`` thread.
* **TCP listeners** via ``add_tcp_listener()`` (accepts connections and hands
  each new client to a callback) and **TCP streams** via ``add_tcp_stream()``
  (reads a connected socket, invokes a data callback, and auto-unregisters on
  disconnect) - so a whole TCP server runs with no accept thread and no
  thread-per-client.

A low-level ``add_fd()`` / ``remove()`` pair is also available.

.. note::

   Lifetime: registered sockets and callbacks must outlive their registration.
   ``stop()`` (and the destructor) waits for any in-flight handler to finish, so
   the guaranteed-safe teardown order is to ``stop()`` / destroy the reactor
   first, then destroy the sockets. ``remove()`` is *asynchronous* with respect
   to a handler that is already running for that id.

.. note::

   The ``select()`` backend uses ``fd_set``, so on POSIX/lwip a registered file
   descriptor must be below ``FD_SETSIZE``; registration rejects an fd at or
   above that limit. On lwip, socket fds occupy
   ``[FD_SETSIZE - CONFIG_LWIP_MAX_SOCKETS, FD_SETSIZE)``, so this is only hit if
   those limits are raised past the compiled ``FD_SETSIZE``.

.. ------------------------------- Example -------------------------------------

Code examples for the reactor are provided in the ``socket`` example folder (the
"Socket reactor" and "TCP reactor" scenarios).

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/socket_reactor.inc

Network APIs
************

.. toctree::
    :maxdepth: 1

    socket_example
    socket
    udp_socket
    tcp_socket

The network APIs provide a useful abstraction over POSIX sockets enabling easily
starting client/server sockets and allowing their use with std::function
callbacks for servers.

Currently, UDP and TCP sockets are supported.

Code examples for the network API are provided in the `socket` example folder..

"""SocketReactor example.

Demonstrates espp.SocketReactor: a single select()-based event loop plus a small
thread pool that services many receiver sockets, instead of one thread per
socket. Here one reactor multiplexes two UDP echo receivers (on two ports); each
received datagram is handled on a pool worker by a Python callback that returns
the reversed payload, which the reactor sends back to the sender.

Run (from this folder, with the `espp` package installed - see README):

    python socket_reactor.py
"""

import socket
import time

import espp

PORTS = [6101, 6102]


def make_echo_callback(port):
    def on_receive(data, sender):
        # Runs on a reactor thread-pool worker. Return bytes to reply, or None.
        print(f"  [port {port}] received {len(data)} bytes from "
              f"{sender.address}:{sender.port}")
        return bytes(reversed(data))
    return on_receive


def main():
    # One reactor (one select() loop + a 2-worker pool) for BOTH receivers.
    reactor = espp.SocketReactor(worker_count=2, log_level=espp.Logger.Verbosity.warn)

    # Keep the receiver sockets alive for as long as they are registered.
    servers = []
    for port in PORTS:
        server = espp.UdpSocket(espp.UdpSocket.Config(espp.Logger.Verbosity.warn))
        rid = reactor.add_udp_receiver(server, port, 1024, make_echo_callback(port))
        if rid == espp.SocketReactor.INVALID_ID:
            print(f"failed to register receiver on port {port}")
            return 1
        servers.append(server)
    print(f"reactor running with {reactor.num_registered()} UDP receivers on ports {PORTS}")

    # Send a datagram to each receiver and print the reversed reply.
    for port in PORTS:
        client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        client.settimeout(1.0)
        payload = bytes(f"hello:{port}".encode())
        client.sendto(payload, ("127.0.0.1", port))
        reply, _ = client.recvfrom(1024)
        print(f"  [port {port}] sent {payload!r}, got reply {reply!r}")
        client.close()

    time.sleep(0.2)
    reactor.stop()  # stop the loop and wait for in-flight handlers before teardown
    print("reactor stopped")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

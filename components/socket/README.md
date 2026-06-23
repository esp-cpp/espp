# Socket Component

[![Badge](https://components.espressif.com/components/espp/socket/badge.svg)](https://components.espressif.com/components/espp/socket)

The network APIs provide a useful abstraction over POSIX sockets enabling easily
starting client/server sockets and allowing their use with std::function
callbacks for servers.

Currently, UDP and TCP sockets are supported.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Socket Component](#socket-component)
  - [Base Socket](#base-socket)
  - [UDP Socket](#udp-socket)
  - [TCP Socket](#tcp-socket)
  - [Example](#example)

<!-- markdown-toc end -->

## Base Socket

The socket provides the base abstraction around the socket file descriptor and
includes some initialization, cleanup, and conversion utilities.

The socket class is subclassed into UdpSocket and TcpSocket.

## UDP Socket

UDP sockets provide unreliable, unordered communication over IP network sockets.

UDP sockets can be used in unicast (point to point), multicast (one to many and
many to one), and broadcast (one to all).

The `UdpSocket` API supports both one-shot sends and long-running receive tasks:

* `send(...)` can optionally wait for a response with a timeout and callback
* `start_receiving(...)` starts a task that continuously receives datagrams and
  can optionally send a callback-produced response
* `stop_receiving()` cleanly stops a blocked receive task during teardown

## TCP Socket

TCP sockets provide reliable, ordered communication over IP network sockets and
have built-in protocols for handling packet acknowledgement as well as
transmission speed / bandwidth adjustment.

TCP sockets cannot be used with multicast (many to one, one to many).

The `TcpSocket` API covers both client and server patterns:

* `connect(...)` plus `transmit(...)` for client-style request/response flows
* optional blocking response waits with callback delivery
* `bind(...)`, `listen(...)`, and `accept()` for server-side flows
* `close()` / `reinit()` helpers for teardown and reconnect paths

## Example

The [example](./example) shows the use of the classes provided by the `socket`
component and runs a scenario-driven self-test which covers teardown, timeout,
and reconnect behavior, including:

* `UdpSocket` as both `client` and `server`, including unicast and multicast configurations
* `TcpSocket` as both `client` and `server`
* scope-based teardown while tasks are active or blocked
* request/response callbacks and timeout handling
* reconnect behavior after TCP session shutdown

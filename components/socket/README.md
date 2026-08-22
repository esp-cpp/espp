# Socket Component

[![Badge](https://components.espressif.com/components/espp/socket/badge.svg)](https://components.espressif.com/components/espp/socket)

The network APIs provide a useful abstraction over POSIX sockets enabling easily
starting client/server sockets and allowing their use with std::function
callbacks for servers.

Currently, UDP and TCP sockets are supported. A `SocketReactor` is also provided
for servicing many receiver sockets on a single `select()` loop plus a thread
pool, instead of one thread per socket.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Socket Component](#socket-component)
  - [Base Socket](#base-socket)
  - [UDP Socket](#udp-socket)
  - [TCP Socket](#tcp-socket)
  - [Socket Reactor](#socket-reactor)
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
* `bind(...)` binds the socket for a server without starting a thread, so it can
  be driven by a `SocketReactor` (see below) instead of its own receive task

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

## Socket Reactor

The `SocketReactor` multiplexes many receiver sockets on a single `select()`
event-loop thread and dispatches each socket's read + user callback onto a shared
`ThreadPool`, instead of dedicating one thread (one `espp::Task`) to every
receiving socket. This turns "N receiver threads" into "1 loop thread + a small
fixed pool" that can be shared across subsystems.

To stay correct under level-triggered `select()`, the reactor is one-shot: a
readable socket is disarmed, a job is submitted to the pool, and the socket is
re-armed only after that job completes - so there is at most one in-flight
handler per socket (per-socket ordering is preserved and there is no concurrent
`recv` on one fd), while different sockets run concurrently. A loopback UDP
"wakeup" socket keeps registration changes, re-arming, and stop responsive.

The `SocketReactor` API drives:

* `add_udp_receiver(...)` - binds a `UdpSocket` and receives on it, replacing a
  per-socket `UdpSocket::start_receiving()` thread
* `add_tcp_listener(...)` - accepts connections and hands each new client to a
  callback (no accept thread)
* `add_tcp_stream(...)` - reads a connected `TcpSocket`, invokes a data callback,
  and auto-unregisters on disconnect (no thread-per-client)
* low-level `add_fd(...)` / `remove(...)`

The thread pool may be owned (built from `Config`) or an external shared pool.
Note: registered sockets must outlive their registration - `stop()` / destroy the
reactor before destroying the sockets (`stop()` waits for in-flight handlers).

### Priority bands and DSCP

Each registration carries an `espp::QosBand` (`Critical` / `High` / `Normal` /
`Low`; `Normal` by default, preserving the pre-band FIFO behavior): when several
sockets are readable in one `select()` round the ready set is dispatched
most-urgent-first, and each handler is submitted to the `ThreadPool` at its
band, so a `Critical` socket's handler overtakes queued lower-band handlers even
on a saturated pool. `UdpSocket::ReceiveConfig::band` sets it for UDP receivers;
`add_tcp_listener(...)` / `add_tcp_stream(...)` / `add_fd(...)` take a band
argument. UDP receivers can additionally set `UdpSocket::ReceiveConfig::dscp`
(0-63) to mark their *transmitted* replies with a DSCP code point (applied as
`IP_TOS`, best-effort) - network / driver treatment for outgoing traffic,
orthogonal to the local `band` scheduling.

## Example

The [example](./example) shows the use of the classes provided by the `socket`
component and runs a scenario-driven self-test which covers teardown, timeout,
and reconnect behavior, including:

* `UdpSocket` as both `client` and `server`, including unicast and multicast configurations
* `TcpSocket` as both `client` and `server`
* scope-based teardown while tasks are active or blocked
* request/response callbacks and timeout handling
* reconnect behavior after TCP session shutdown
* `SocketReactor` multiplexing UDP receivers and TCP listeners/streams on one
  select loop + thread pool (shared-pool, dynamic remove, and multi-client cases)
* `SocketReactor` priority bands: a Critical-band receiver (with DSCP-marked
  replies) staying responsive during a Low-band flood

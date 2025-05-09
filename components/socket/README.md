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

## TCP Socket

TCP sockets provide reliable, ordered communication over IP network sockets and
have built-in protocols for handling packet acknowledgement as well as
transmission speed / bandwidth adjustment.

TCP sockets cannot be used with multicast (many to one, one to many).

## Example

The [example](./example) shows the use of the classes provided by the `socket`
component, including:

* `UdpSocket` (as both `client` and `server`, including unicast and multicast configurations)
* `TcpSocket` (as both `client` and `server`)


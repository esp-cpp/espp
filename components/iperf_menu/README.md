# Iperf Menu Component
[![Badge](https://components.espressif.com/components/espp/wifi/badge.svg)](https://components.espressif.com/components/espp/iperf_menu)

The `iperf_menu` component simply provides a menu for the `espressif/iperf`
component, enabling you to easily use the `iperf` network performance testing
tool on your ESP device using the `cli` component.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [Iperf Menu Component](#iperf-menu-component)
  - [Menu Functions:](#menu-functions)
  - [Example](#example)

<!-- markdown-toc end -->

## Menu Functions:

The `iperf_menu` component provides a simple command-line interface (CLI) for
interacting with the `iperf` tool. The available commands are:

```console
iperf> help
Commands available:
 - help
	This help message
 - exit
	Quit the session
 - udp
	Get the current UDP mode setting.
 - udp <udp>
	Set the UDP mode for iperf tests.
 - ipv6
	Get the current IPv6 setting.
 - ipv6 <ipv6>
	Set the IPv6 mode for iperf tests.
 - interval
	Get the current reporting interval setting.
 - interval <interval>
	Set the reporting interval for iperf tests.
 - time
	Get the current time (duration) setting.
 - time <time>
	Set the time for iperf tests.
 - format
	Get the current formatting setting (kibits/sec or mbits/sec).
 - format <format>
	Set the output format for iperf tests.
 - length
	Get the current buffer length setting.
 - length <length>
	Set the buffer length for iperf tests.
 - abort
	Abort the current iperf test if running. This will stop the test and print a message.
 - client <host>
	Run an iperf client, connecting to the specified host, using the default port.
 - client <host> <port>
	Run an iperf client, connecting to the specified host and port.
 - client <host> <port> <bandwidth>
	Run an iperf client, connecting to the specified host and port. Uses the passed length as the buffer length for data transmission.
 - server
	Run an iperf server, listening on the default port.
 - server <port>
	Run an iperf server, listening on the specified port.
 - example
	(menu)
```

![CleanShot 2025-07-07 at 15 29 33](https://github.com/user-attachments/assets/da0f1b47-3db5-4ec4-8673-f8a61fa736e3)

## Example

The [example](./example) shows the use of the `espp::IperfMenu` (using WiFi station). 


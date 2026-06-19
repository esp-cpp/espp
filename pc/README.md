# ESPP PC Code

This folder contains some test python scripts for loading the `espp` static
library built within the `espp/lib` folder into c++ on a host system running
Linux, MacOS, or Windows.

<!-- markdown-toc start - Don't edit this section. Run M-x markdown-toc-refresh-toc -->
**Table of Contents**

- [ESPP PC Code](#espp-pc-code)
  - [Setup](#setup)
  - [Quick Start](#quick-start)

<!-- markdown-toc end -->

## Setup

First, ensure that you have built the shared objects in the `espp/lib` folder.
If you haven't done so yet, navigate to the `espp/lib` folder and run the
following:

```console
# if macos/linux:
./build.sh
# if windows
./build.ps1
```

## Quick Start

To run a test, you can simply run the executable from the terminal:

```console
# if windows:
./build/Release/udp_client.exe
# if macos / linux:
./build/udp_client
```

### RTPS tests

`rtps_pubsub`, `rtps_publisher`, and `rtps_subscriber` exercise the `rtps`
component end to end (SPDP/SEDP discovery + best-effort CDR-over-RTPS user data).

- `rtps_pubsub [advertised_ipv4] [run_seconds]` is self-contained: it runs a
  publisher and a subscriber in one process and exits 0 if samples were received.
- `rtps_publisher [topic] [advertised_ipv4] [period_ms]` and
  `rtps_subscriber [topic] [advertised_ipv4]` are standalone and interoperate
  with each other, with the Python `rtps_publisher.py`/`rtps_subscriber.py` (and
  `rtps_host.py`) in `../python`, and with embedded ESPP RTPS participants.

```console
# self-contained smoke test (auto-detects the local IPv4):
./build/rtps_pubsub
# cross-process / cross-language (use a real interface IP on both sides):
./build/rtps_subscriber espp/test/counter 192.168.1.50   # terminal 1
./build/rtps_publisher  espp/test/counter 192.168.1.50   # terminal 2
```

Note: RTPS discovery is multicast, so these require a multicast-capable network.

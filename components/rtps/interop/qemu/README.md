# RTPS interop firmware for QEMU

The device side of the QEMU interop matrix (`../run_qemu_interop.sh`,
`.github/workflows/rtps_qemu.yml`): the embedded RTPS engine, with the esp32
configuration profile, lwIP and FreeRTOS, running in Espressif's QEMU on the
OpenCores Ethernet MAC through the [`espp::Qemu`](../../../qemu) BSP. The host
side drives ROS 2 (FastDDS and CycloneDDS) and the espp host engine against it
on a tap device and asserts on the `RTPSTEST ...` lines this firmware logs.

What it runs, all at once:

| endpoint | topic (ROS 2 name) | QoS |
|---|---|---|
| publisher, `"qemu <n>"` every 200 ms | `/qemu_tx` | reliable |
| subscriber | `/qemu_rx` | reliable |
| publisher, `"qemu_be <n>"` every 200 ms | `/qemu_be_tx` | best effort |
| subscriber | `/qemu_be_rx` | best effort |
| subscriber for fragmented samples | `/qemu_big` | reliable |
| service server (`example_interfaces/srv/AddTwoInts`) | `/qemu_add_two_ints` | |
| action server (`example_interfaces/action/Fibonacci`) | `/qemu_fibonacci` | one execute worker (`ActionConfig::execute_workers`), whose task is part of the "endpoints" memory stage |

and a status line every 2 s with the counters, free heap and the heap low-water
mark. The markers are listed at the top of `main/rtps_qemu.cpp`.

## Build

```bash
# as CI builds it: a static address on the harness's tap network (192.168.100.2)
idf.py -DSDKCONFIG_DEFAULTS="sdkconfig.defaults;sdkconfig.ci.defaults" build
# the flash image the harness runs
cd build && python -m esptool --chip esp32 merge-bin --fill-flash-size 4MB -o qemu_flash.bin @flash_args
```

Without `sdkconfig.ci.defaults` the firmware is a DHCP client, which is what a
developer wants on `idf.py qemu`'s user-mode NAT:

```bash
idf.py build
idf.py qemu monitor
```

That run has no peers (user-mode networking carries no multicast), but shows the
engine coming up on the target: PSRAM (4 MB, emulated), the address, the
participant, the publisher ticking, the status lines. For the full matrix use
`../run_qemu.sh` (Linux + Docker) or the CI workflow.

The CI defaults also raise the remote-participant pool to 24 through the
menuconfig knob (`CONFIG_RTPS_LIMIT_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS`): the
many-participants case puts twelve unrelated ROS 2 participants on the domain,
more than the embedded profile's ten, which is both what an application on a
busy domain has to do and what the case proves works. The per-endpoint proxy
knobs go up with it (each builtin discovery endpoint holds one proxy per
remote participant), and the lease cap, the unmatched-endpoint pools and the
name-length caps are raised for the same reason; `sdkconfig.ci.defaults` says
why for each.

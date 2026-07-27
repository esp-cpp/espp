# Meshtastic Example

This example builds a minimal Meshtastic-compatible node on either the LilyGo
T-Deck or the M5Stack Cardputer-Adv (with the LoRa+GPS Cap), selectable via
menuconfig. It brings up the board's SX1262 radio through the BSP, applies the
LongFast modem configuration for the selected region, listens for Meshtastic
traffic, periodically broadcasts its node info and a text ping, and prints
received text messages, node info, and positions.

Point a stock Meshtastic device (set to the same region and the default
LongFast channel) at it and the two should see each other: your espp node will
appear in the device's node list, and text messages will flow both ways.

## How to use example

### Hardware Required

- LilyGo T-Deck / T-Deck Plus, **or** M5Stack Cardputer-Adv with the LoRa+GPS
  Cap (U201 / U214)
- Ideally a second Meshtastic device (or another espp node) to talk to

### Configure

Run `idf.py menuconfig` and set, under `Example Configuration`:

- **Hardware** - your board
- **LoRa region** - this MUST match the other Meshtastic devices and be legal
  where you are
- Optionally the node name and ping intervals

> ⚠️ Always attach the antenna before transmitting, and make sure the region
> you select is legal for you.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

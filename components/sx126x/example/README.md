# SX126x Example

This example shows how to use the `espp::Sx126x` component to configure an
SX1262 LoRa radio and send / receive packets. Each board running the example
transmits a ping packet periodically and prints any packets it receives
(with RSSI / SNR), so flashing it onto two boards gives a simple link test.

## How to use example

### Hardware Required

One (ideally two) of:

- LilyGo T-Deck / T-Deck Plus (SX1262 on the shared SPI bus)
- M5Stack Cardputer-Adv with the LoRa+GPS Cap (U201 / U214, SX1262)
- Custom hardware with an SX1261 / SX1262 / LLCC68 connected via SPI

Select the board with `idf.py menuconfig` (`Example Configuration >
Hardware`), and make sure the configured frequency matches your hardware
variant and is legal in your region (default: 906.875 MHz, the Meshtastic US
LongFast frequency slot; EU boards should use 869.525 MHz). Always attach the
antenna before transmitting.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

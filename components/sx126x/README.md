# SX126x Component

[![Badge](https://components.espressif.com/components/espp/sx126x/badge.svg)](https://components.espressif.com/components/espp/sx126x)

The `Sx126x` component provides a driver for the Semtech SX126x series of
sub-GHz LoRa radio transceivers (SX1261, SX1262) and compatible chips such as
the LLCC68. These radios are found on many popular development boards,
including the LilyGo T-Deck (SX1262) and the M5Stack Cardputer-Adv LoRa+GPS
Cap (SX1262).

Features:

- LoRa modem configuration (spreading factor, bandwidth, coding rate,
  preamble length, sync word, CRC, IQ inversion)
- Interrupt-driven (DIO1) or polled operation
- Blocking and non-blocking transmit
- Continuous receive with RSSI / SNR packet status
- Channel activity detection (CAD)
- TCXO (DIO3) and RF-switch (DIO2) control
- Time-on-air calculation
- Datasheet errata workarounds (TX modulation quality, TX clamp, IQ polarity)

The radio is a command-based SPI peripheral; the driver is written against the
`espp::BasePeripheral` interface and requires a `write_then_read`
implementation which keeps chip select asserted across the write and read
phases (e.g. a single full-duplex transfer via `espp::Spi::Device::transfer`),
which allows it to share an SPI bus with other devices (as on the T-Deck,
where the radio shares the bus with the display and uSD card).

## Example

The [example](./example) shows how to use the `espp::Sx126x` component to
configure the radio and send / receive LoRa packets, configurable (via
menuconfig) for the LilyGo T-Deck, the M5Stack Cardputer-Adv with the
LoRa+GPS Cap, or custom hardware. Flash it onto two boards and they will
ping each other.

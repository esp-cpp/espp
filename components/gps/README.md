# GPS Component

[![Badge](https://components.espressif.com/components/espp/gps/badge.svg)](https://components.espressif.com/components/espp/gps)

The `Gps` component provides a driver for UART-attached GNSS receivers which
output standard NMEA-0183 sentences, such as the ATGM336H (found on the
M5Stack Cardputer-Adv LoRa+GPS Cap) and the u-blox MIA-M10Q (found on the
LilyGo T-Deck Plus).

It contains two classes:

- `espp::NmeaParser`: a platform-independent, incremental NMEA-0183 parser
  (RMC / GGA, any talker id) with checksum validation, which maintains a
  `espp::GpsFix` (position, altitude, speed, course, satellites, hdop, UTC
  date/time with unix-timestamp conversion).
- `espp::Gps`: a UART-attached wrapper which reads the NMEA stream in a
  background task and provides the latest fix thread-safely (or via
  callback). It also supports writing raw configuration commands to the
  receiver.

## Example

The [example](./example) shows how to use the `espp::Gps` component to parse
GNSS data, configurable (via menuconfig) for the M5Stack Cardputer-Adv
LoRa+GPS Cap, the LilyGo T-Deck Plus, or custom hardware.

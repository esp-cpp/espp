# RX8130CE Real-Time Clock (RTC)

[![Badge](https://components.espressif.com/components/espp/rx8130ce/badge.svg)](https://components.espressif.com/components/espp/rx8130ce)

This component provides a driver for the RX8130CE Real-Time Clock chip from EPSON.

## Features

- I2C interface (address 0x32)
- Real-time clock with calendar (seconds, minutes, hours, day, month, year, weekday)
- Automatic leap year correction
- Temperature compensated crystal oscillator (TCXO)
- Alarm functions with interrupt output
- Timer functions (countdown timer)
- Square wave output with selectable frequencies
- Low power consumption
- Wide operating voltage range (1.6V to 5.5V)
- std::tm based time APIs
- Device-specific time structures for advanced features

## Datasheet

- [RX8130CE Datasheet](https://support.epson.biz/td/api/doc_check.php?dl=brief_RX8130CE&lang=en)

## Example

The [example](./example) shows how to use the RX8130CE RTC driver to set and get
the time.

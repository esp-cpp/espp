# GPS Example

This example shows how to use the `espp::Gps` component to parse NMEA data
from a UART-attached GNSS receiver and print the resulting fixes.

## How to use example

### Hardware Required

One of:

- M5Stack Cardputer-Adv with the LoRa+GPS Cap (ATGM336H, 115200 baud)
- LilyGo T-Deck Plus (u-blox MIA-M10Q, 9600 baud)
- Custom hardware with a UART NMEA GNSS receiver

Select the board with `idf.py menuconfig` (`Example Configuration >
Hardware`). Note that GNSS receivers need sky view to get a fix - expect no
fix indoors.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

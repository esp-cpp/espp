# Audio Example

This example plays a 440 Hz tone through `espp::Audio` at a few volumes and
then muted, on an I2S amplifier with no codec (e.g. a MAX98357A), so the
volume and mute are applied in software.

The default pins (menuconfig, `Example Configuration`) are the LilyGo
T-Deck's: BCLK 7, WS 5, data out 6, with its peripheral power rail (GPIO 10)
switched on first.

## How to use example

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to
build projects.

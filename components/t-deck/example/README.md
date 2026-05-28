# T-Deck Example

This example shows how to use the `espp::TDeck` hardware abstraction component
to initialize the components on the LilyGo T-Deck.

It initializes the touch panel, display, keyboard, trackball, sound output, and
optional microSD card support. Touching the screen draws a cyan trail overlay,
the delete key clears the trail, the space key or on-screen refresh button
rotates the display, and the keyboard also exposes simple mute/volume controls.

https://github.com/user-attachments/assets/5d7e7086-fc2c-4477-8948-07b5bab3e51f

https://github.com/esp-cpp/espp/assets/213467/dc476c3d-dd9e-4b65-8c2d-9eda3ff3f33f

![image](https://github.com/esp-cpp/espp/assets/213467/4744d6ee-33bd-4907-8c58-3f3c2e5b7ba6)

## How to use example

### Hardware Required

This example is designed to run on the LilyGo T-Deck.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Features Demonstrated

- Shared SPI LCD transport via `espp::Spi` and `SpiPanelIo`
- Touch drawing with a rotation-aware LVGL overlay
- T-Keyboard input for clearing / rotating and audio control
- Trackball initialization and callback wiring
- Optional uSD card mounting over SDSPI on the same SPI host
- WAV playback through the onboard audio path

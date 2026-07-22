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

## Configuration notes

This example raises some espp BSP task stack sizes above their component
defaults via `sdkconfig.defaults`, because the example does more work in
those tasks than the defaults assume:

- **Interrupt / touch task stack (`CONFIG_TDECK_INTERRUPT_STACK_SIZE` = 8192, up from the 4 KB
  BSP default).** The interrupt task services the touch controller over
  I2C; if an I2C transaction errors, the error is logged through espp's
  `fmt`-based (colorized) logger, whose formatting path needs several KB of
  stack. With only 4 KB this can overflow and corrupt memory. If you base
  your own project on this example (or reproduce its functionality), keep
  this override in your `sdkconfig` / `sdkconfig.defaults`.

The example also enables `CONFIG_FREERTOS_WATCHPOINT_END_OF_STACK`, which
turns any future task-stack overflow into an immediate, clearly-named panic
instead of silent heap corruption.

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
- Microphone recording and playback: the on-screen audio row (or the 'r' /
  'p' keys) records from the ES7210 into a PSRAM-preferred buffer and streams
  it back through the mono speaker, with speaker / microphone volume buttons;
  the measured effective capture rate is logged when a recording stops.

  Note on audio quality: the T-Deck's microphone is very low-sensitivity
  (LilyGO's own firmware reads only ~200 counts for loud speech), and its
  capture picks up random electrical impulse noise. The example therefore
  post-processes each recording before playback: it de-glitches the impulse
  noise (mark-and-interpolate), removes the DC offset, and applies an
  RMS-normalized software makeup gain (the analog gain alone is not enough).
  Only one microphone (MIC1) is used and mirrored to both channels for the
  mono speaker. Even so, recordings are quiet and noisy - this is the limit
  of the board's microphone hardware, not a configuration issue.

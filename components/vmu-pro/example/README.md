# VMU Pro Example

This example shows how to use the `espp::VmuPro` hardware abstraction component
to initialize the components on the [8BitMods VMU
Pro](https://8bitmods.com/vmupro-handheld-visual-memory-card-for-dreamcast-classic-white/).

It initializes the display, buttons, audio, and uSD card subsystems and builds
a simple LVGL UI using a `Gui` class:

- The D-pad moves a cursor around the screen
- The A button draws a circle at the cursor (and plays a beep)
- The B button clears the circles (and plays a lower beep)
- The Mode button rotates the display through 0/90/180/270 degrees
- The Bottom button cycles the backlight brightness (25/50/75/100%)
- The Power button toggles the audio mute

> **Warning:** The GPIO assignments in the `espp::VmuPro` component are
> UNVERIFIED placeholders (the VMU Pro's schematic is not publicly available),
> so this example will build but is not expected to function on real hardware
> until the pins in `components/vmu-pro/include/vmu-pro.hpp` are corrected.

## How to use example

### Hardware Required

This example is designed to run on the 8BitMods VMU Pro.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

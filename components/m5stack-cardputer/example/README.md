# M5Stack Cardputer Example

This example shows how to use the `espp::M5StackCardputer` hardware abstraction
component to initialize and use the components on the M5Stack Cardputer:

- The display (with LVGL, via a small `Gui` class that implements a
  keyboard-driven text editor)
- The 56-key QWERTY keyboard (typing, shift layer, and the fn layer's arrow /
  delete / esc keys)
- The speaker (a key-click beep for every keypress)
- The RGB LED (the G0 / BOOT button cycles its color)
- The uSD card (mounted at startup if inserted)
- The battery voltage measurement (shown in the status bar)

## How to use example

### Controls

| Input | Action |
|-------|--------|
| Printable keys (with shift layer) | Type into the text area |
| Backspace / Enter / Tab | Edit the text area |
| Fn + `;` / `,` / `.` / `/` | Move the cursor (up / left / down / right) |
| Fn + backspace | Delete forward |
| Fn + `` ` `` | Clear the text area |
| Fn + number row | Show F1-F12 in the status bar |
| G0 (BOOT) button | Cycle the RGB LED color |

The status bar at the bottom of the screen shows the most recent key /
modifier activity and is updated with the battery voltage every few seconds.

### Hardware Required

This example is designed to run on the M5Stack Cardputer (K132), with an
optional uSD card inserted.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

Serial output showing initialization of the subsystems, followed by log lines
for keyboard / button activity. The display shows the text editor with the
status bar along the bottom.

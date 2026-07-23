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
- The battery voltage / state-of-charge measurement (shown in the status bar)
- The IMU on the Cardputer ADV (live accelerometer / gyroscope readings shown
  in the top-right corner)

## How to use example

### Controls

| Input | Action |
|-------|--------|
| Printable keys (with shift layer) | Type into the text area |
| Backspace / Enter / Tab | Edit the text area |
| Fn + `;` / `,` / `.` / `/` | Move the cursor (up / left / down / right) |
| Fn + backspace | Delete forward |
| Fn + `` ` `` | Clear the text area |
| Fn + 1 (F1) | Show / hide the controls help popup |
| Fn + 2 (F2) | Show / hide the IMU overlay (ADV) |
| Fn + 3 (F3) | Start / stop recording from the microphone (ADV) |
| Fn + 4 (F4) | Play back / stop the recording (ADV) |
| Fn + 5 / 6 (F5 / F6) | Speaker volume down / up |
| Fn + 7 / 8 (F7 / F8) | Microphone volume down / up (75% = 0 dB) |
| Fn + number row | Show F1-F12 in the status bar |
| G0 (BOOT) button | Cycle the RGB LED color |

The controls are also printed to the log at startup.

On the ADV the ES8311 codec runs the speaker and microphone in full duplex,
so recording works while the speaker is active (key clicks and all). The
recording buffer prefers PSRAM when present; since neither Cardputer variant
ships with PSRAM, it normally falls back to a few seconds of voice-rate
(16 kHz mono) audio in internal RAM. The original Cardputer cannot record in
this example: its PDM microphone clock shares GPIO 43 with the speaker
word-select, and the example uses the speaker.

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

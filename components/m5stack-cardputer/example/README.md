# M5Stack Cardputer Example

This example shows how to use the `espp::M5StackCardputer` hardware abstraction
component to initialize and use the components on the M5Stack Cardputer:

- The display (with LVGL, via a small `Gui` class that presents a **tabbed
  interface** switched from the keyboard - there is no touchscreen)
- The 56-key QWERTY keyboard (typing, shift layer, and the fn layer's arrow /
  delete / esc keys)
- The speaker (a key-click beep for every keypress)
- The RGB LED (the G0 / BOOT button cycles its color)
- The uSD card (mounted at startup if inserted)
- The battery voltage / state-of-charge measurement
- The IMU on the Cardputer ADV (live accelerometer / gyroscope readings)
- The SX1262 LoRa radio on the LoRa+GPS Cap (send / receive text; Cardputer ADV
  accessory)
- The ATGM336H GNSS receiver on the same Cap (position / satellites / UTC time
  shown on the GPS tab)

The UI is a tabview with five tabs, switched with **fn+Tab** (or jumped to
directly with fn+1 / fn+2 / fn+9):

| Tab | Contents |
|-----|----------|
| **Text** | The text editor - type here; this is also where LoRa messages are composed |
| **LoRa** | Radio status and a scrolling log of sent / received messages with RSSI |
| **IMU** | Live accelerometer / gyroscope readings (ADV only) |
| **GPS** | GNSS fix status (position, satellites, UTC time) from the LoRa+GPS Cap |
| **Sys** | Board variant, battery, and speaker / mic volumes |
| **Help** | The list of controls |

A slim status bar along the bottom stays visible on every tab for transient
messages (key names, volume changes, "Sending..."). While the LoRa tab is
active, the status bar instead mirrors the message you are composing (the text
box lives on the Text tab), so you can see what you are about to send.

## How to use example

### Controls

| Input | Action |
|-------|--------|
| **Fn + Tab** | **Switch to the next tab** |
| Printable keys (with shift layer) | Type into the text area (Text tab) |
| Backspace / Enter / Tab | Edit the text area |
| Fn + `;` / `,` / `.` / `/` | Move the cursor (up / left / down / right) |
| Fn + backspace | Delete forward |
| Fn + `` ` `` | Clear the text area |
| Fn + 1 (F1) | Jump to the Help tab |
| Fn + 2 (F2) | Jump to the IMU tab (ADV) |
| Fn + 9 (F9) | Jump to the LoRa tab |
| Fn + 0 (F10) | Send the text area's contents over LoRa |
| Fn + 3 (F3) | Start / stop recording from the microphone (ADV) |
| Fn + 4 (F4) | Play back / stop the recording (ADV) |
| Fn + 5 / 6 (F5 / F6) | Speaker volume down / up |
| Fn + 7 / 8 (F7 / F8) | Microphone volume down / up (75% = 0 dB) |
| G0 (BOOT) button | Cycle the RGB LED color |

The controls are also printed to the log at startup, and are shown on the Help
tab.

### LoRa (with the LoRa+GPS Cap)

The **LoRa** tab drives the SX1262 radio on the M5Stack LoRa+GPS Cap (a
Cardputer ADV accessory). Type a message on the Text tab, press **fn+0** to
transmit it, and received packets appear in the LoRa tab's log with their RSSI.

This uses the same raw-LoRa settings as the T-Deck example (US LongFast
modulation - 906.875 MHz, SF11/BW250 - on a private sync word `0x12`), so a
Cardputer with the Cap and a T-Deck each running their example will exchange
text messages. This is *not* Meshtastic (that uses sync word `0x2B`) - see the
`meshtastic` component for Meshtastic interoperability. Attach the antenna
before transmitting, and make sure the frequency is legal in your region. If
the Cap is not attached (or the board is not an ADV), the LoRa tab reports that
the radio is unavailable.

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

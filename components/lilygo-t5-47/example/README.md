# LilyGo T5 4.7" e-paper Example

This example shows how to use the `espp::LilyGoT547` hardware abstraction
component to bring up the LilyGo T5 4.7" (ESP32-S3) e-paper board.

It initializes the e-paper display and an LVGL grayscale UI, then the on-board
peripherals: the BOOT button, GT911 capacitive touch (registered as an LVGL
input device), the PCF8563 RTC, the BQ27220 battery fuel gauge, the PCA9535 I/O
expander (and the IO48 button), the frontlight, and the microSD card.

The UI (in the `Gui` class, `gui.hpp`/`gui.cpp`) shows a live RTC clock and date,
and a toggle-able stats panel with the battery voltage / state-of-charge /
current, the last touch coordinates, the IO48 and home button states, and the
LoRa status. It has six touch buttons:

* **Light** - toggle the frontlight on/off.
* **Rotate** - cycle the display rotation. The layout (a flex column of a
  header, the stats rows, a spacer, and a wrapping button row) re-flows to the
  new landscape/portrait dimensions, and touch stays aligned.
* **Stats** - show/hide the stats panel. The panel update mode follows it: crisp
  grayscale (`GC16`) while shown so the small text is legible, and fast mono
  (`DU`) while hidden for a snappy clock-only view. (`DU` is too coarse to render
  the small stats text.)
* **LoRa TX** - transmit a test message over the SX1262 radio. Received packets
  (from another unit on the same frequency / sync word) are logged and shown in
  the stats panel's LoRa row. Set the frequency for your region in the example's
  `RadioConfig` (default US 906.875 MHz).
* **Refresh** - do a full-screen refresh to clear e-paper ghosting.
* **Off** - power the board down (BQ25896 ship mode); press PWR to turn it back
  on. Only works on battery (not while USB is connected).

The BOOT button also triggers a full refresh, and the app does one automatically
every 30 s. Because e-paper refreshes are slow and flash, the UI only redraws
labels whose value actually changed.

## How to use example

### Hardware Required

This example requires a LilyGo T5 4.7" (ESP32-S3) e-paper board.

### Build and Flash

Build the project and flash it to the board, then run the monitor tool to view
serial output. The ESP-IDF component manager must be enabled (it is, in the
example's CMakeLists) so the epdiy dependency is fetched:

```
cd example
idf.py set-target esp32s3
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

## Notes

E-paper updates are slow (hundreds of ms) and the panel needs its high-voltage
rails powered only during an update. The default update mode is `MODE_GC16`
(smooth 16-level grayscale); switch to a mono mode such as `MODE_DU` via
`set_lvgl_update_mode()` for faster but rougher black-and-white updates. Do a
periodic `full_refresh()` to clear ghosting.

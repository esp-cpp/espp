# M5Stack Tab5 Board Support Package (BSP) Component

[![Badge](https://components.espressif.com/components/espp/m5stack-tab5/badge.svg)](https://components.espressif.com/components/espp/m5stack-tab5)

The M5Stack Tab5 is a highly expandable, portable smart-IoT terminal development device featuring a dual-chip architecture with rich hardware resources. The main controller uses the **ESP32-P4** SoC based on the RISC-V architecture with 16 MB Flash and 32 MB PSRAM. The wireless module uses the ESP32-C6-MINI-1U, supporting Wi-Fi 6.

The `espp::M5StackTab5` component provides a singleton hardware abstraction for initializing and managing all the Tab5's subsystems including display, touch, audio, camera, IMU, power management, and expansion interfaces.

## Key Features

### Display & Touch
- 5″ 1280 × 720 IPS TFT screen via MIPI-DSI
- **Automatic display controller detection** (supports ILI9881, ST7123, or ST7121)
- Multi-touch (GT911 on ILI9881 units; integrated Sitronix TDDI touch on ST7123/ST7121 units)
- Adjustable backlight brightness control

### Audio System
- Dual audio codecs: ES8388 + ES7210 AEC front-end
- Dual-microphone array for voice recognition
- 1W speaker + 3.5mm headphone jack
- Hi-Fi recording and playback capabilities

### Camera
- MIPI-CSI camera (SC202CS) brought up via Espressif's `esp_video` (V4L2)
  pipeline (CSI receiver + ISP), with the ISP converting the sensor's RAW8
  output to RGB565
- `initialize_camera(callback)` streams frames to the callback from a capture
  task; the sensor's SCCB shares the internal I2C bus
- See the example's Camera tab for a live-feed display

### Sensors & IMU
- BMI270 6-axis sensor (accelerometer + gyroscope)
- Interrupt wake-up capability
- Real-time orientation and motion tracking

### Power Management
- Removable NP-F550 Li-ion battery
- MP4560 buck-boost converter
- IP2326 charge management (`set_charging_enabled()`, off by default; only
  enable it when `read_battery_status().is_present` is true, since the charger
  retrying to charge an empty battery bay pops the speaker periodically; the
  enabled charger also pops on every speaker transient, so the BSP pauses
  charging during playback and resumes it 3 s after the last sound, see
  `set_charging_pause_on_playback_enabled()`)
- INA226 real-time power monitoring
- Multiple power modes for efficiency

### Communication & Expansion
- ESP32-C6 wireless module (Wi-Fi 6, Thread, ZigBee)
- USB-A Host + USB-C OTG ports
- RS-485 industrial interface with switchable 120Ω terminator
- Grove and M5-Bus expansion headers
- microSD card slot
- STAMP expansion pads for additional modules

### Real-Time Clock
- RX8130CE RTC with timed interrupt wake-up
- Battery-backed time keeping
- Programmable wake-up alarms

## Hardware Specifications

| Component | Specification |
|-----------|---------------|
| Main SoC | ESP32-P4NRW32 (RISC-V 32-bit dual-core 400 MHz + LP single-core 40 MHz) |
| Wireless SoC | ESP32-C6-MINI-1U (Wi-Fi 6 @ 2.4 GHz / Thread / ZigBee) |
| Flash | 16 MB |
| PSRAM | 32 MB |
| Display | 5-inch IPS TFT (1280 × 720) |
| Touch | GT911 (ILI9881 units) / integrated Sitronix TDDI (ST7123/ST7121 units) |
| Camera | SC2356 @ 2 MP (1600 × 1200) |
| Audio | ES8388 codec + ES7210 AEC |
| IMU | BMI270 6-axis (accelerometer + gyroscope) |
| Battery | NP-F550 2000mAh removable |
| Expansion | Grove, M5-Bus, STAMP pads, GPIO headers |

## Display Controller Auto-Detection

The M5Stack Tab5 hardware has shipped with three different MIPI-DSI display revisions:
- **ILI9881** panel + separate GT911 touch controller (original units)
- **ST7123** TDDI — integrated display + touch (units manufactured after Oct 2025)
- **ST7121** TDDI — integrated display + touch (newest units)

The BSP automatically detects which display controller is present during initialization:
1. It polls the Sitronix TDDI touch interface (I²C `0x55`) for its firmware
   version register: **1 → ST7121**, **3 → ST7123**. (The two ST parts need
   different init sequences and DSI lane rates and cannot be told apart by
   their DSI ID.)
2. If instead a GT911 touch controller ACKs (I²C `0x14`), the unit is the
   original **ILI9881** revision.
3. The detected controller type is logged for debugging.

This means your application code works seamlessly across all three hardware variants without any code changes. You can optionally query the detected controller type:

```cpp
auto& tab5 = espp::M5StackTab5::get();
tab5.initialize_lcd();

// Query the detected controller
auto controller_type = tab5.get_display_controller();
const char* controller_name = tab5.get_display_controller_name();
```

## Rendering into the panel's frame buffers

The DPI panel scans a frame buffer in PSRAM out to the display continuously. By
default the BSP creates one (`CONFIG_M5STACK_TAB5_DPI_NUM_FBS` = 1): LVGL
renders into separate draw buffers and every flush copies the area into that
frame buffer. With 2 (or 3) frame buffers an application can render straight
into them and show a finished frame by switching the panel to its buffer, with
no copy:

- `frame_buffers()` returns the panel's frame buffers (after
  `initialize_lcd()`), each `frame_buffer_size()` bytes, cache-line aligned.
  `esp_lcd_panel_draw_bitmap()` with a pointer inside one of them makes the
  panel switch to that buffer instead of copying it.
- `present_frame(frame_buffer)` switches the panel to a finished frame and
  waits until the panel scans it out (at most one refresh: about 17 ms, 21 ms
  on the ILI9881 panel), so the previous buffer is free to draw into:
  tear-free double buffering. The wait uses the DMA's frame-done event;
  `on_color_trans_done` cannot serve, because for a frame buffer the driver
  calls it before the switch. Tear-free therefore depends on that wait
  succeeding. On a timeout (default 100 ms) the switch stays queued and the
  call returns `false` rather than stalling: the caller carries on and the
  next frame may tear. After three consecutive timeouts (a frame-done event
  that never arrives, e.g. a panel / ESP-IDF combination whose callback does
  not fire) `present_frame()` disables its wait itself, with a warning, so
  the UI is not held at one frame per timeout for good; frames may then tear
  until `set_vsync_enabled(true)` re-arms it. `set_vsync_enabled(false)` skips
  the wait deliberately. The DIRECT-mode flush ignores the return value for
  the same reason: it never blocks LVGL on a missing event.
- `initialize_display({.render_to_frame_buffers_enabled = true})` sets LVGL up in DIRECT
  mode on two of them, with the BSP's flush calling `present_frame()` at the end
  of each frame. No draw buffers and no rotation scratch buffer are allocated.
  The display rotation must stay at 0 (LVGL cannot rotate in DIRECT mode).

```cpp
// menuconfig: M5Stack Tab5 Configuration -> Number of DPI frame buffers = 2
auto &tab5 = espp::M5StackTab5::get();
tab5.initialize_lcd();
tab5.initialize_display({.render_to_frame_buffers_enabled = true});
```

Measured on a Tab5 in an LVGL application (full-screen redraw, 720 x 1280
RGB565): 75.6 ms with full-frame double draw buffers copied into one frame buffer,
48.6 ms rendering into two frame buffers. Memory: two frame buffers take 3.7 MB of
PSRAM, against 5.5 MB for one frame buffer plus two full-frame draw buffers (and
the 1.8 MB rotation scratch buffer is not needed either). The frame buffers are
cache-line aligned, so they also satisfy a raised `LV_DRAW_BUF_ALIGN` (e.g. 128
for the PPA draw unit), which a plain `heap_caps_malloc()` draw buffer may not.

## Tear-free flushes with rotation: the back-buffer flush

DIRECT mode cannot rotate. The back-buffer flush keeps LVGL's normal partial
rendering into draw buffers (so the PPA rotation of each area, and the panel's
own 180-degree flip, keep working) and gets rid of tearing differently:

- Compiled in with `CONFIG_M5STACK_TAB5_BACK_BUFFER_FLUSH` (off by default:
  an application that does not enable it carries none of the code, and its
  flush path is unchanged). Then `initialize_display({.flush_to_back_buffer_enabled
  = true})` (needs `CONFIG_M5STACK_TAB5_DPI_NUM_FBS` >= 2 and the PPA) turns it
  on per display; nothing is allocated or installed until it is set. Every flushed area is
  copied, rotated when needed, by the PPA into the frame buffer the panel is
  *not* scanning out; nothing is drawn into the buffer on screen.
- When LVGL's last area of a frame has landed, the flush switches the panel to
  that buffer with `present_frame()` at the frame boundary, then has the PPA
  copy the region this frame changed from the new front buffer into the new
  back buffer, left in flight while LVGL renders the next frame, so the back
  buffer is current when the next frame's areas are written into it. (With
  three buffers each one is brought up to date when it becomes the back
  buffer.)
- The panel never shows a half-drawn frame, whatever the draw-buffer size.
  That is what makes small draw buffers viable again: with a single frame
  buffer a full-screen redraw flushes strip by strip into the buffer on
  screen, visible as a progressive wipe, which is why the example used two
  whole-frame draw buffers (3.7 MB); with the back-buffer flush the strips are
  assembled off screen and shown at once.

```cpp
// menuconfig: M5Stack Tab5 Configuration -> Number of DPI frame buffers = 2,
//             Compile in the back-buffer LVGL flush = y
auto &tab5 = espp::M5StackTab5::get();
tab5.initialize_lcd();
tab5.initialize_display({.pixel_buffer_size = 720 * 128, // 128-row strips (184 KB each)
                         .flush_to_back_buffer_enabled = true});
```

Costs: the flush of a frame's last area waits for the frame-done event (one
refresh, like DIRECT mode), and the PPA does one extra copy of the changed
region per frame. Pixels written with `write_lcd_lines()` land in the buffer on
screen and are not carried over to the next frame: draw through LVGL.

Measured with the example's display benchmark (a local test aid, not in the
repository: four phases of 8 s at 0 and at 90 degrees, same unit as the earlier
tables) on a Tab5 (ST71xx, ESP-IDF v6.1). "Stock" is one frame buffer with two
whole-frame draw buffers (the example's default); the back-buffer flush uses two
frame buffers, with whole-frame or 128-row (184 KB) draw buffers:

| phase | stock (tears) | back-buffer, whole-frame draw buffers | back-buffer, 128-row strips |
| --- | --- | --- | --- |
| small update, 0 deg | 993 fps (unthrottled) | 57.8 (locked to the 58 Hz panel) | 57.8 |
| sweeping bar, 0 deg | 350 | 57.7, no tearing | 57.5, no tearing |
| full-screen redraw, 0 deg | 38.2 | 28.9 | 28.9 |
| heavy animation, 0 deg | 15.7 | 11.5 | 11.5 |
| small update, 90 deg | 589 | 57.8 | 57.8 |
| sweeping bar, 90 deg | 260 | 56.7 | 56.8 |
| full-screen redraw, 90 deg | 16.2 | 11.6 | 14.4 |
| heavy animation, 90 deg | 9.4 | 6.4 | 7.2 |
| free PSRAM after display init | 26.36 MB | 24.52 MB | 27.89 MB |

A full-screen redraw at 0 degrees takes two panel periods (34.5 ms): render plus
copy is about 26 ms, which misses one frame-done event and waits for the next -
the price of never showing a half-drawn frame, as in DIRECT mode. At 90 degrees
the strips are faster than whole-frame draw buffers because the PPA rotation of
one strip overlaps the rendering of the next. The 128-row configuration frees
1.5 MB of PSRAM against the stock example (two frame buffers plus two 184 KB
draw buffers, 4.1 MB, against one frame buffer plus two whole-frame draw
buffers, 5.5 MB).

## Example

The [example](./example) shows how to use the `espp::M5StackTab5` hardware abstraction component to initialize and use various subsystems of the Tab5.


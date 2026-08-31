# BLDC Haptics Example (USB / WebUSB controlled)

This example shows the use of the `BldcHaptics` component to drive a BLDC motor
(such as a tiny gimbal motor) as a user input / output device that provides
haptic feedback (such as might be used as a rotary encoder input).

On top of the haptic engine, the example exposes a **USB vendor-specific
(WebUSB) interface** (via the espp `usb_device` component) so the knob can be
controlled and visualized live from a Chromium browser — no driver, no app
install:

* **Live telemetry + dial visualization** — position / detent index, continuous
  knob value, shaft angle and velocity, streamed at a configurable rate and
  rendered on an animated dial (detents, end stops and the current detent
  marked).
* **Control** — enable/disable the haptics, move to a detent, play a haptic
  "click" with adjustable strength.
* **Mode switching** — select any of the built-in `espp::detail` detent presets
  (unbounded, bounded, multi-rev, on/off, coarse/fine, magnetic detents,
  return-to-center) from a dropdown.
* **Firmware update (OTA)** — upload a new `.bin` over the same USB interface
  (via the espp `ota` component), with progress, image validation (SHA-256) and
  bootloader rollback support.

The wire protocol is documented in [PROTOCOL.md](./PROTOCOL.md); the browser
console lives in [webapp/index.html](./webapp/index.html).

## How to use example

### Hardware Required

This example targets ESP32-S3 hardware (the native USB-OTG peripheral is
required for the vendor / WebUSB interface). Select the hardware via
`idf.py menuconfig` → `Example Configuration`:

* **MotorGo Mini** or **MotorGo Axis** — everything on-board (motor driver +
  SSI magnetic encoder); just connect a gimbal motor.
* **BLDC Motor Test Stand (TinyS3)** (default) — discrete wiring:
  * Magnetic encoder chip (this example uses `Mt6701`) over I2C
  * BLDC Motor Driver chip (tested with the `TMC6300 BOB` dev board)
  * Some mounting hardware to mount the motor, magnet, encoder, etc.
  * Motor powered via a benchtop power supply at 5V

:warning:
> NOTE: you MUST make sure that you run the example with the
> `zero_electrical_offset` value set to 0 (or not provided) at least once
> otherwise the sample will not work and could potentially damage your motor.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use; to exit the serial
monitor, type ``Ctrl-]``.)

The example uses an OTA-capable partition table (`partitions.csv`: `otadata` +
two 3 MB app slots on an 8 MB flash), so the very first flash must be a full
`idf.py flash` (not just `app-flash`) to lay down the partition table and
otadata.

> On boards whose only USB connector is the ESP32-S3's native USB (e.g. MotorGo
> Mini), TinyUSB takes over the connector once the app starts, so the
> USB-Serial-JTAG console (`idf.py monitor`) goes away. Runtime logs are still
> available over the same cable: the example exposes a CDC-ACM serial port and
> routes the system console to it, so attach any serial terminal (e.g.
> `screen /dev/tty.usbmodem*`) for live logs. Flashing also still works over
> the same connector via the ROM bootloader (hold BOOT while resetting, or
> just use `webapp/index.html` for OTA updates after the first flash).

### Web console

1. Flash and start the example, then connect the board's **native USB-OTG**
   port to your computer (on an S3 devkit this is the "USB" connector, not
   "UART").
2. Open `webapp/index.html` in a Chromium browser (Chrome / Edge / Opera). It
   is a single self-contained file and works from `file://`, `http://localhost`
   or any `https` origin. (Chrome may also offer the hosted console
   automatically via the WebUSB landing-page notification.)
3. Click **Connect** and pick "espp BLDC Haptics" (VID `0x1209`, PID `0x0d34`).
4. The dial starts animating from the telemetry stream. Use the controls to
   switch detent presets, enable/disable the motor, move to a detent, or play a
   haptic click.

### Firmware update (OTA) flow

1. Make a change and `idf.py build` (do not flash).
2. In the web console's **Firmware update** panel, pick
   `build/bldc_haptics_example.bin` (the app image — NOT the merged /
   bootloader image) and click **Upload**.
3. The device streams the image into the inactive OTA slot (progress + rate are
   shown), validates it (structure + SHA-256), switches the boot partition and
   reboots. Expect a USB disconnect; reconnect after the device re-enumerates.
4. Rollback: the freshly-booted image starts in `PENDING_VERIFY`; this example
   marks itself valid after its self-check (motor + haptics up). If the new
   image crashes before that, the bootloader automatically rolls back to the
   previous slot on the next reset.

The same OTA transfer can also be driven from the generic espp OTA console
(`components/ota/web/ota_console.html`), since the OTA subset of the protocol
is byte-compatible with the espp `ota` example.

## Example Behaviors

The detent presets can be switched at runtime from the web console (or by
editing the default in the code). Some examples:

### coarse values strong detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/a256b401-6e45-4284-89c7-2dec9a49daa7

### magnetic detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/ab1ace5c-f967-4cfc-b304-7736fdb35bcb

### On / Off Strong Detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/038d79b1-7cd9-4af9-b7e8-1b4daf6a363a

### Multi-rev no detents

https://github.com/esp-cpp/espp/assets/213467/2af81edb-67b8-488b-ae7a-3549be36b8cc

For more information, see the documentation or the original PR:
https://github.com/esp-cpp/espp/pull/60

## Troubleshooting

Make sure to run the example once with `zero_electrical_offset` set to 0 so that
the motor will go through a calibration / zero offset routine. At the end of
this startup routine it will print the measured zero electrical offset that you
can then provide within the code, at which point it will not need to run the
calibration routine.

You must run this calibration any time you change your hardware configuration
(such as by remounting your motor, magnet, encoder chip).

If the web console cannot see the device:

* WebUSB needs a Chromium-based browser and a secure context (`https`,
  `http://localhost` or `file://`).
* Make sure you connected the *native USB-OTG* port (not a UART bridge port).
* On Linux you may need a udev rule granting access to VID `0x1209`.
* Tick "show all USB devices" in the console to bypass the VID/PID filter.

## Example Breakdown

This example builds complex haptic behavior + connectivity using the following
components:

* `espp::Mt6701` (I2C on the test stand, SSI on the MotorGo boards)
* `espp::BldcDriver` / the MotorGo board components
* `espp::BldcMotor`
* `espp::BldcHaptics`
* `espp::UsbDevice` — native USB vendor interface with WebUSB + MS OS 2.0
  descriptors (driverless browser access)
* `espp::Ota` — transport-agnostic OTA engine fed from the USB protocol
* The `ota_stream` framing (`components/ota/include/detail/ota_stream_protocol.hpp`)
  reused as the framing layer for the haptics protocol
  (see [PROTOCOL.md](./PROTOCOL.md))

You combine the `Mt6701` and `BldcDriver` together when creating the `BldcMotor`
and then simply pass the `BldcMotor` to the `BldcHaptics` component. At that
point, you only have to interface to the `BldcHaptics` to read the input
position or reconfigure the haptics — which is exactly what the USB protocol
handlers do.

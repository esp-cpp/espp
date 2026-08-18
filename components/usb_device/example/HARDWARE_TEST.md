# Hardware test: ODrive-compatible USB device (ASCII on CDC + native/Fibre on vendor)

This example makes an ESP32-S3 (or -S2/-P4) enumerate as an **ODrive-compatible
USB device** on its native USB-OTG peripheral, presenting two interfaces from one
simulated motor state:

- **CDC serial** → the **ODrive ASCII** protocol (text; terminal / the Web Serial console).
- **vendor (0xFF, WebUSB)** → the **ODrive native (Fibre) binary** protocol — the one
  `odrivetool` / the `fibre` library auto-discover over USB.

VID/PID default to `0x1209 / 0x0d32` (ODrive v3-like). The log console stays on the
built-in USB-Serial-JTAG, separate from this device.

> Board note: on single-USB-connector S3 devkits the USB-OTG and USB-Serial-JTAG
> share pins. Use a board that exposes the USB-OTG D+/D- (a second connector or the
> OTG header), or set the console to UART, so the native USB device enumerates.

## 1. Flash

```sh
cd components/usb_device/example
idf.py set-target esp32s3
idf.py -p <PORT> flash monitor
```
The monitor prints the endpoint-tree size + `json_crc` once USB is up.

## 2. Verify the native (Fibre) interface over USB — the real gate

`odrivetool` from `pip install odrive` (0.6+) uses the **new** libfibre/protocol and
will **not** talk to this legacy-protocol device. Use the **reference legacy fibre**
(pure python), which is exactly what the codec targets and what the interop harness
already clones.

```sh
# libusb + a venv with pyusb (the fibre USB backend uses pyusb)
brew install libusb                      # macOS  (Linux: apt install libusb-1.0-0)
python3 -m venv .venv-usb && . .venv-usb/bin/activate
pip install pyusb appdirs

# run the probe -- it fetches the reference fibre for you with --clone
python odrive_usb_probe.py --clone
```
The reference fibre is pure-python (only needs pyserial/pyusb). `--clone` shallow-
clones ODrive `fw-v0.5.1` into `./odrive-ref` and uses `Firmware/fibre/python` from
it. Alternatives to `--clone`:
- if you've already run `components/odrive_native/interop/run.sh`, the probe
  **auto-detects** the clone it made — just `python odrive_usb_probe.py`;
- or clone it yourself and point at it:
  ```sh
  git clone --depth 1 -b fw-v0.5.1 https://github.com/odriverobotics/ODrive /tmp/odrive-ref
  python odrive_usb_probe.py --fibre-path /tmp/odrive-ref/Firmware/fibre/python
  ```

Expected: it discovers the board over USB, downloads endpoint 0, enumerates
`vbus_voltage / axis0.* / serial_number`, reads values, and writes-then-reads
`input_pos` and `vel_limit` — `ALL PROBE ASSERTIONS PASSED`.

On **Linux** you may need `sudo` or a udev rule to claim the vendor interface. On
**Windows** the device must bind WinUSB (the firmware advertises MS-OS-2.0, so it
should bind automatically).

Legacy `odrivetool` (`pip install 'odrive==0.5.6'` in a Python ≤3.10 env) should also
auto-discover it as `odrv0`; the reference-fibre probe above avoids that install.

## 3. Verify the ASCII interface (CDC)

The device also shows up as a **serial/CDC port**. Send ODrive ASCII lines with any
terminal, e.g. `r axis0.encoder.pos_estimate`, `p 0 1.0`, `f 0`. Or open the hosted
**Web Serial console** and pick this CDC port. (Writes/setpoints are silent by
default — ODrive semantics; only `r`/`f` respond.)

## 4. Verify WebUSB (browser)

Open `components/odrive_ascii/web/odrive_control_panel.html` (native-protocol control
panel) in Chromium and Connect; it claims the vendor interface directly (no driver) and
shows the endpoint tree with live reads/plots. The firmware's WebUSB landing-page
descriptor also points a browser at the hosted console.

## 5. Verify the HID gamepad (WebHID)

The device also enumerates as a **HID gamepad**: the firmware animates both analog
sticks and toggles two buttons at ~10 Hz. Your OS will see a gamepad. To visualize the
input reports directly in the browser, open `components/odrive_ascii/web/hid_visualizer.html`
in Chromium and Connect (pick the espp gamepad) — you should see the sticks circling and
buttons blinking. WebHID reads the HID interface directly (no driver); it works on the
same composite device without disturbing the CDC/vendor interfaces.

## What "good" looks like
- The USB probe prints the full endpoint tree and `ALL PROBE ASSERTIONS PASSED`.
- The CDC port answers `r`/`f`.
- If all three work, the ASCII + native + USB stack is validated end-to-end on real
  hardware — then it's safe to open the PRs.

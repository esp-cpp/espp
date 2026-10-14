# VL53LXX Time of Flight (Tof) Distance Sensor Peripheral Component

[![Badge](https://components.espressif.com/components/espp/vl53l/badge.svg)](https://components.espressif.com/components/espp/vl53l)

Drivers for ST's VL53L Time-of-Flight (ToF) sensors that talk over I2C with
16-bit register addresses:

| Class | Sensors | What it measures |
| --- | --- | --- |
| `espp::Vl53l` (`vl53l.hpp`) | VL53L4CD, VL53L4CX | one distance, up to 1.3 m (CD) or 6 m (CX), 1 mm resolution |
| `espp::Vl53l5cx` (`vl53l5cx.hpp`) | VL53L5CX | a 4x4 or 8x8 grid of distances up to 4 m over a 63° diagonal field of view, at up to 60 Hz (4x4) or 15 Hz (8x8) |

The sensors use a laser to send a pulse of light and measure the time it takes
for the light to reflect back. They can struggle outdoors, as sunlight
interferes with the measurement, and cannot measure through glass or other
transparent materials.

> **Note:** Neither driver is compatible with the older VL53L0X or VL53L1X
> sensors, which use a different (8-bit) register map and communication
> protocol.

## VL53L4CD / VL53L4CX (`espp::Vl53l`)

Single-zone ranging: timing budget, inter-measurement period, continuous or
autonomous mode, data-ready polling and interrupt clearing.

## VL53L5CX (`espp::Vl53l5cx`)

Multizone ranging. The sensor has no firmware in ROM: `initialize()` (run by
the constructor by default) reboots it and uploads the ~84 KB VL53L5CX firmware
over I2C, which takes about two seconds at 400 kHz (the sensor supports 1 MHz
Fast Mode Plus). The driver then offers:

- resolution (4x4 / 8x8), ranging frequency, integration time, sharpener,
  target order (closest / strongest), ranging mode (continuous / autonomous),
  power mode, I2C address change;
- `start_ranging()` / `stop_ranging()`, `is_data_ready()` polling and
  `get_ranging_data()`, which fills a `RangingData` with, per zone, the
  distance, sigma, signal and ambient rates, SPAD count, reflectance estimate,
  number of targets and a target status (5 and 9 mean a valid range);
- the motion indicator (`enable_motion_indicator()`).

Zones are reported in row-major order (`zone = row * width + col`); see ST's
UM2884 for the physical orientation of zone 0. One target per zone is
supported (the ULD default). The crosstalk calibration and detection threshold
plugins of ST's driver are not ported yet; the default crosstalk shape is
loaded.

The I2C device's transaction timeout must cover the driver's longest transfers
(512-byte firmware chunks, ~1.4 KB result frames): a few tens of milliseconds
at 400 kHz, more at 100 kHz.

### License note

The VL53L5CX firmware image and default configuration (`src/vl53l5cx_firmware.cpp`)
and the sensor sequences in `src/vl53l5cx.cpp` come from STMicroelectronics'
VL53L5CX Ultra Lite Driver (ULD 1.1.2, as redistributed by
[stm32duino/VL53L5CX](https://github.com/stm32duino/VL53L5CX)) and are
redistributed under their BSD-3-Clause license (see the file headers). The
rest of the component is MIT like the rest of espp.

## Examples

- [example](./example): `Vl53l` with a VL53L4CD, printing the distance.
- [vl53l5cx_example](./vl53l5cx_example): `Vl53l5cx` printing the 8x8 grid of
  distances at 15 Hz.

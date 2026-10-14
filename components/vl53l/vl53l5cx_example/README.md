# VL53L5CX Multizone ToF Example

This example shows the use of the `Vl53l5cx` component to communicate with a
VL53L5CX multizone time-of-flight sensor.

The example initializes the sensor (which uploads the sensor's firmware over
I2C, about two seconds at 400 kHz), configures the resolution (8x8 zones by
default) and the ranging frequency (15 Hz by default), starts ranging and then
prints every frame as a grid of distances in millimeters, with `----` for the
zones that have no valid target.

## Hardware Required

This example is designed to work with any ESP32 which has I2C pins exposed, but
is pre-configured to run on QtPy boards such as the QtPy ESP32 Pico and QtPy
ESP32-S3.

It requires a VL53L5CX breakout, such as the [SparkFun Qwiic ToF Imager
VL53L5CX](https://www.sparkfun.com/products/18642) or ST's
[VL53L5CX-SATEL](https://www.st.com/en/evaluation-tools/vl53l5cx-satel.html).
The sensor needs its `LPn` pin high (it is pulled up on the breakouts) and its
I2C address is 0x29 at power-up.

> **Note:** the sensor's firmware is uploaded at every boot and result frames
> are up to 1.4 KB, so the I2C bus timeout must cover
> the driver's longest transfers: the example derives it from the clock with
> `espp::Vl53l5cx::i2c_timeout_ms()`.

## How to use example

### Configure the project

```
idf.py menuconfig
```

`Example Configuration` has the I2C pins and clock speed, the resolution (4x4
or 8x8) and the ranging frequency.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view
serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

```
I (337) VL53L5CX example: Starting example!
I (347) VL53L5CX example: Creating I2C on port 0 with SDA 41 and SCL 40 at 400000 Hz
I (357) Vl53l5cx: Uploading the VL53L5CX firmware (86016 bytes)
I (2567) Vl53l5cx: Initialized (4x4, autonomous, 1 Hz)
I (2797) Vl53l5cx: Resolution set to 8x8
I (2857) Vl53l5cx: Ranging started (64 zones, 1440 byte frames)
  812   804   799   801   806   817   835   855
  798   792   788   790   795   804   822   843
  790   783   777   779   785   795   811   832
  788   780   774   776   781   790   806   826
  791   782   776   778   783   792   808   828
  801   791   785   787   792   801   817   838
  817   806   800   802   807   816   833   854
 ----   826   820   822   827   836   853  ----
```

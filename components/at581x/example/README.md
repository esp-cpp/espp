# AT581X Example

This example demonstrates the use of the `espp::At581x` driver to configure the
AT581X 5.8 GHz radar presence/motion sensor (e.g. the MS58-3909S68U4 module on
the ESP32-S3-BOX-3 sensor dock) over I2C.

It:

- creates an I2C bus (default SDA=GPIO41, SCL=GPIO40 for the BOX-3 sensor dock),
- constructs the `At581x` driver, which writes the configuration to the chip,
- periodically changes the sensing distance (sensitivity) at runtime, and
- attaches an `espp::Interrupt` to the radar output GPIO
  (`CONFIG_EXAMPLE_RADAR_OUTPUT_GPIO`, default GPIO21 on the BOX-3 dock) and
  prints presence/motion transitions. Set it to -1 to disable.

When the **ESP32-S3-BOX-3** hardware profile is selected (the default on
esp32s3), the example also uses the `esp-box` BSP to show the live radar status
on the screen — a color-coded presence label plus the detection count and
configured sensing distance. The radar is placed on `I2C_NUM_1` in this mode so
it doesn't collide with the box's internal I2C bus (`I2C_NUM_0`, GPIO8/18).

On the ESP32-S3-BOX-3 with the sensor dock attached, the defaults (SDA=41,
SCL=40, radar output=21) should work as-is.

## Configuration

Use `idf.py menuconfig` → *Example Configuration* to set the I2C pins and the
radar output GPIO for your hardware.

## Build and Flash

```sh
idf.py set-target esp32s3
idf.py build flash monitor
```

## Output

The example logs the configuration it applies and, when the radar output GPIO is
wired up, `Radar presence DETECTED` / `Radar presence cleared` as the sensor
sees motion within its configured range.

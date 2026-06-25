# ST7123Touch Example

This example shows how to use the ST7123 integrated touch controller driver.
It demonstrates:

- Probing the I2C bus for the ST7123 touch interface (default address 0x55)
- Creating an `espp::St7123Touch` instance
- Wrapping it as `std::shared_ptr<espp::ITouchDriver>` using the
  `espp::make_touch_driver` helper and the `espp::TouchDriverConcept`
- Polling for touch events via the type-erased interface in a background task

## How to use example

### Hardware Required

Any ESP32 board connected to an ST7123 TDDI panel via I2C.

> **Note:** The ST7123's touch engine is enabled by the **LCD_RST** pulse that
> is issued during display initialization. Do **not** toggle a separate
> `TP_RST` line — doing so may take the touch I2C endpoint offline.

### Configure

```
idf.py menuconfig
```

Set the `I2C SDA` and `I2C SCL` GPIO numbers for your board under
**Example Configuration**.

### Build and Flash

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type `Ctrl-]`.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to
build projects.

## Example Output

```
Starting st7123touch example
ST7123 touch probe: 1
         address:   0x55
num_touch_points: 1, x: 320, y: 240
num_touch_points: 0, x: 0, y: 0
```

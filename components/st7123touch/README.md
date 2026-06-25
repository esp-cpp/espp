# ST7123 Touch Component

Driver for the touch controller integrated into the **Sitronix ST7123** TDDI
(Touch and Display Driver Integration) chip.

## Overview

The ST7123 combines a MIPI-DSI display driver and a capacitive multi-touch
controller in a single IC. This component exposes the touch side only; the
display initialization sequence is handled by the `display_drivers` component.

Touch data is retrieved over I2C at the chip's address (default **0x55**) using
the following register map:

| Register | Width | Description |
|----------|-------|-------------|
| `0x0009` | 1 B   | Max simultaneous touch count (firmware configured) |
| `0x0010` | 1 B   | Advanced info — bit 3 = `with_coord` flag |
| `0x0014` | 7 B × N | Touch coordinate reports (N = max touches) |

Each 7-byte touch report packs:

```
Byte 0: [valid(7)] [reserved(6)] [x_h(5:0)]
Byte 1: x_l
Byte 2: y_h
Byte 3: y_l
Byte 4: contact area
Byte 5: intensity
Byte 6: reserved
```

### Important: TP_RST must NOT be toggled for ST7123

The ST7123's touch engine is enabled by the **LCD_RST** pulse that is issued
during display initialization. Toggling a separate `TP_RST` line (as used by
standalone controllers like the GT911) has no effect and, on some boards
(e.g. M5Stack Tab5), will take the touch I2C endpoint offline.

## Usage

```cpp
#include "st7123touch.hpp"

espp::St7123Touch touch({
    .write = std::bind_front(&espp::I2c::write, &i2c),
    .read  = std::bind_front(&espp::I2c::read,  &i2c),
});

std::error_code ec;
touch.update(ec);
uint8_t num = 0;
uint16_t x = 0, y = 0;
touch.get_touch_point(&num, &x, &y);
```

## Dependencies

- `espp/base_peripheral`

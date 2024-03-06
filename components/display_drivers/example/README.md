# Display Drivers Example

This example is designed to show how the `display_drivers` component can be used
to drive various different displays with LVGL and a simple GUI (that is
contained within the example: `main/gui.hpp`).

## How to use example

### Hardware Required

This example can be configured to run on the following dev boards:
* ESP32-WROVER-Kit
* TTGO T-Display
* ESP32-S3-BOX
* Smartknob-HA

### Configure the project

```
idf.py menuconfig
```

When configuring the project, select the `Display Drivers Example Configuration`
value that matches the board you've selected (must be one of the 4 boards
mentioned above.)

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Breakdown

The example has the following functionality:
* SPI pre and post transfer callbacks for handling the data/command (DC_LEVEL)
  GPIO for the screens and the lvgl flush flag management
* `lcd_write` for polling (blocking) transmit example
* `lcd_send_lines` and `lcd_wait_lines` for queued (non-blocking) transmit example
* `Gui` class (contained in `main/gui.hpp`) which encapsulates some very basic
  LVGL components into an object that manages gui update task and synchronization.

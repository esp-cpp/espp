# LED Strip Component 

[![Badge](https://components.espressif.com/components/espp/led_strip/badge.svg)](https://components.espressif.com/components/espp/led_strip)

The `LedStrip` component provides APIs to control LED strips. It supports
various LED strip types, such as WS2812, WS2811, WS2813, SK6812, APA102, etc.
You can use it directly with an SPI driver to talk to APA102 LED strips, or you
can use it with a RMT driver (such as the `Rmt` component) to talk to WS2812,
WS2811, WS2813, SK6812, etc. LED strips.

## Example

The [example](./example) shows the use of the `LedStrip` class to control a 5x5
array of RGB LEDs (SK6805).


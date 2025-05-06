# LilyGo T-Deck Board Support Package (BSP) Component 

The LilyGo T-Deck is a development board for the ESP32-S3 module. It features a
nice touchscreen display and expansion headers.

The `espp::TDeck` component provides a singleton hardware abstraction for
initializing the touch, display, audio, and micro-SD card subsystems.

## Example

The [example](./example) shows how to use the `espp::TDeck` hardware abstraction
component initialize the components on the LilyGo T-Deck.


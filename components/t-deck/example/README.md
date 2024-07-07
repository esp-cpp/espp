# T-Deck Example

This example shows how to use the `espp::TDeck` hardware abstraction component
initialize the components on the LilyGo T-Deck.

It initializes the touch, display, and t-keyboard subsystems. It reads the
touchpad state and each time you touch the screen it uses LVGL to draw a circle
where you touch. If you press the home button on the display, it will clear the
circles.

![image](https://github.com/esp-cpp/espp/assets/213467/4744d6ee-33bd-4907-8c58-3f3c2e5b7ba6)

## How to use example

### Hardware Required

This example is designed to run on the LilyGo T-Deck.

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

![CleanShot 2024-07-05 at 23 43 27@2x](https://github.com/esp-cpp/espp/assets/213467/03d1dad5-e9fa-461c-9eb2-1e5d314dcfdb)

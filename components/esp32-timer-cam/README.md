# ESP32-TIMER-CAM Board Support Package (BSP) Component

https://components.espressif.com/components/espp/esp32-timer-cam/badge.svg

The ESP32 TimerCam is a little usb-c webcamera using the ESP32. It has a little
blue LED, a real-time clock (RTC), and supports battery operation with battery
voltage monitoring as well.

The `espp::EspTimerCam` component provides a singleton hardware abstraction for
initializing the I2C, LED, RTC, and ADC subsystems.

## Example

The [example](./example) shows how to use the `espp::EspTimerCam` hardware
abstraction component to automatically detect and initialize components on the
ESP32-TimerCam.

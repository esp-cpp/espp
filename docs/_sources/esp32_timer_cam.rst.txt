ESP32 TimerCam
**************

Esp32-Timer-Cam
---------------

The ESP32 TimerCam is a little usb-c webcamera using the ESP32. It has a little
blue LED, a real-time clock (RTC), and supports battery operation with battery
voltage monitoring as well.

The `espp::EspTimerCam` component provides a singleton hardware abstraction for
initializing the I2C, LED, RTC, and ADC subsystems.

.. ------------------------------- Example -------------------------------------

.. toctree::

   esp32_timer_cam_example

.. ---------------------------- API Reference ----------------------------------

API Reference
-------------

.. include-build-file:: inc/esp32-timer-cam.inc
